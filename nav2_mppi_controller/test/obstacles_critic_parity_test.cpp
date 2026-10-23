// Copyright (c) 2026 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Locks the classical (CPU) scoring behavior of ObstaclesCritic against hand-computed
// expectations. Every expected value below is derived in the test from the center costs of
// the scenario, never read back from the critic.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};

RosLockGuard g_rclcpp;

namespace
{

// World coordinate at the center of a cell on the default map (origin 0, resolution 0.1)
float world_from_cell(unsigned int cell)
{
  return (static_cast<float>(cell) + 0.5f) * 0.1f;
}

// Costmap cost that stands for "no information", also what an off-map pose reads as
constexpr float kNoInformation = 255.0f;
constexpr float kLethal = 254.0f;
constexpr float kInscribed = 253.0f;

// Weights and distances the critic is configured with in every case
constexpr float kRepulsionWeight = 1.5f;
constexpr float kCriticalWeight = 20.0f;
constexpr float kCollisionCost = 100000.0f;
constexpr float kMarginDistance = 0.10f;
constexpr float kNearGoalDistance = 0.5f;

// Inflation layer values read back from the configured costmap
struct Inflation
{
  float scale{0.0f};
  float radius{0.0f};
  float inscribed{0.0f};
};

// One trajectory pose. cost is the cost the critic reads at the pose center. When paint is
// set, the harness writes cost to cell (mx, my); poses off the map are not painted and read
// as no information.
struct Step
{
  float x;
  float y;
  unsigned int mx;
  unsigned int my;
  float cost;
  bool paint;
};

Step cell_step(unsigned int mx, unsigned int my, float cost)
{
  return {world_from_cell(mx), world_from_cell(my), mx, my, cost, true};
}

Step world_step(float x, float y)
{
  return {x, y, 0u, 0u, kNoInformation, false};
}

struct Case
{
  bool inflation{true};
  bool track_unknown{false};
  int power{1};
  float local_path_length{10.0f};
  bool enabled{true};
  bool track_collisions{true};
  bool initial_fail{false};
  bool preseed_collision{false};
  int time_steps{1};
  // traj[trajectory][time]
  std::vector<std::vector<Step>> traj;
  // Expected per-trajectory collision, and the expected fail flag
  std::vector<bool> collided;
  bool fail{false};
};

// Distance to the obstacle for a center cost, footprint not used
float dist_center(float cell, float scale, float inscribed)
{
  return (scale * inscribed - std::log(cell) + std::log(253.0f)) / scale - inscribed;
}

// Collision rule for center costs with the circular robot
bool collides(float cost, bool track_unknown)
{
  if (cost == kLethal || cost == kInscribed) {
    return true;
  }
  if (cost == kNoInformation) {
    return !track_unknown;
  }
  return false;
}

void read_inflation(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, Inflation & out)
{
  auto layer = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(costmap_ros);
  ASSERT_NE(layer, nullptr);
  ASSERT_EQ(layer->getCostScalingFactor(), 10.0);
  ASSERT_EQ(layer->getInflationRadius(), 0.55);
  const double inscribed = costmap_ros->getLayeredCostmap()->getInscribedRadius();
  // The circular robot is a padded polygon in the layered costmap, so the inscribed radius
  // is near 0.1 but not exactly 0.1. The critic reads this same value, so use it as read.
  ASSERT_NEAR(inscribed, 0.1, 0.01);
  out.scale = static_cast<float>(layer->getCostScalingFactor());
  out.radius = static_cast<float>(layer->getInflationRadius());
  out.inscribed = static_cast<float>(inscribed);
}

// Builds the scenario, runs one critic named critic_name with the given use_cuda value, and
// returns the resulting costs, fail flag and collision flags without asserting the formula.
// When the case has an inflation layer and inflation is not null, it receives the layer values.
void score_case(
  const Case & c, const std::string & critic_name, bool use_cuda_flag,
  Eigen::ArrayXf & out_costs, bool & out_fail, std::vector<bool> & out_hits,
  Inflation * inflation = nullptr)
{
  using namespace mppi;  // NOLINT

  const int batch = static_cast<int>(c.traj.size());

  auto node = std::make_shared<nav2::LifecycleNode>(critic_name + "_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    critic_name + "_costmap", "", true);
  std::string handler_name = "parity";
  ParametersHandler param_handler(node, handler_name);

  // Costmap parameters must exist before the costmap reads them in on_configure
  costmap_ros->declare_parameter(
    "track_unknown_space", rclcpp::ParameterValue(c.track_unknown));
  if (!c.inflation) {
    costmap_ros->declare_parameter("plugins", std::vector<std::string>{});
  }

  // Critic parameters, declared with the same C++ types as ObstaclesCritic::initialize
  bool consider_footprint = false;
  int power = c.power;
  float repulsion_weight = kRepulsionWeight;
  float critical_weight = kCriticalWeight;
  float collision_cost = kCollisionCost;
  float collision_margin_distance = kMarginDistance;
  float near_goal_distance = kNearGoalDistance;
  bool enabled = c.enabled;
  bool use_cuda = use_cuda_flag;
  {
    auto getParam = param_handler.getParamGetter(critic_name);
    getParam(consider_footprint, "consider_footprint", false);
    getParam(power, "cost_power", c.power);
    getParam(repulsion_weight, "repulsion_weight", kRepulsionWeight);
    getParam(critical_weight, "critical_weight", kCriticalWeight);
    getParam(collision_cost, "collision_cost", kCollisionCost);
    getParam(collision_margin_distance, "collision_margin_distance", kMarginDistance);
    getParam(near_goal_distance, "near_goal_distance", kNearGoalDistance);
    getParam(enabled, "enabled", c.enabled);
    getParam(use_cuda, "use_cuda", use_cuda_flag);
  }

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  if (c.inflation && inflation != nullptr) {
    ASSERT_NO_FATAL_FAILURE(read_inflation(costmap_ros, *inflation));
  }

  // Zero the master grid, then paint the scenario
  auto * costmap = costmap_ros->getCostmap();
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, 0);
    }
  }
  for (const auto & row : c.traj) {
    for (const auto & step : row) {
      if (step.paint) {
        costmap->setCost(step.mx, step.my, static_cast<unsigned char>(step.cost));
      }
    }
  }

  mppi::critics::ObstaclesCritic critic;
  critic.on_configure(node, "mppi", critic_name, costmap_ros, &param_handler);

  models::State state;
  state.local_path_length = c.local_path_length;
  models::Trajectories trajectories;
  trajectories.reset(batch, c.time_steps);
  for (int i = 0; i < batch; ++i) {
    ASSERT_EQ(static_cast<int>(c.traj[i].size()), c.time_steps);
    for (int t = 0; t < c.time_steps; ++t) {
      trajectories.x(i, t) = c.traj[i][t].x;
      trajectories.y(i, t) = c.traj[i][t].y;
      trajectories.yaws(i, t) = 0.0f;
    }
  }
  models::Path path;
  geometry_msgs::msg::Pose goal;
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(batch);
  float model_dt = 0.1f;
  CriticData data =
  {state, trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt, {}};
  data.fail_flag = c.initial_fail;
  if (c.track_collisions) {
    data.trajectories_in_collision = std::vector<bool>(batch, c.preseed_collision);
  }

  critic.score(data);

  out_costs = costs;
  out_fail = data.fail_flag;
  out_hits.assign(
    data.trajectories_in_collision.begin(), data.trajectories_in_collision.end());
}

void expect_classical(const Case & c)
{
  const int batch = static_cast<int>(c.traj.size());
  ASSERT_EQ(static_cast<int>(c.collided.size()), batch);

  Eigen::ArrayXf costs;
  bool fail = false;
  std::vector<bool> hits;
  Inflation inflation;
  ASSERT_NO_FATAL_FAILURE(score_case(c, "classical", false, costs, fail, hits, &inflation));
  ASSERT_EQ(static_cast<int>(costs.size()), batch);

  if (!c.enabled) {
    // A disabled critic does not score at all
    for (int i = 0; i < batch; ++i) {
      EXPECT_EQ(costs(i), 0.0f) << "trajectory " << i;
    }
    EXPECT_EQ(fail, c.initial_fail);
    if (c.track_collisions) {
      ASSERT_EQ(static_cast<int>(hits.size()), batch);
      for (int i = 0; i < batch; ++i) {
        EXPECT_EQ(hits[i], c.preseed_collision) << "trajectory " << i;
      }
    } else {
      EXPECT_TRUE(hits.empty());
    }
    return;
  }

  const bool near_goal = c.local_path_length < kNearGoalDistance;

  // Raw (margin or collision) and repulsive sums per trajectory
  std::vector<float> raw(batch, 0.0f);
  std::vector<float> rep(batch, 0.0f);
  for (int i = 0; i < batch; ++i) {
    bool collided = false;
    float margin = 0.0f;
    for (int t = 0; t < c.time_steps; ++t) {
      const float cost = c.traj[i][t].cost;
      if (cost < 1.0f) {
        continue;
      }
      if (collides(cost, c.track_unknown)) {
        collided = true;
        break;
      }
      if (!c.inflation) {
        continue;
      }
      const float dist = dist_center(cost, inflation.scale, inflation.inscribed);
      if (dist < kMarginDistance) {
        margin += (kMarginDistance - dist);
      }
      if (!near_goal) {
        rep[i] += (inflation.radius - dist);
      }
    }
    EXPECT_EQ(collided, static_cast<bool>(c.collided[i])) << "trajectory " << i;
    raw[i] = collided ? kCollisionCost : margin;
  }

  float rep_min = 0.0f;
  for (int i = 0; i < batch; ++i) {
    rep_min = (i == 0) ? rep[i] : std::min(rep_min, rep[i]);
  }
  for (int i = 0; i < batch; ++i) {
    const float norm = (rep[i] - rep_min) / static_cast<float>(c.time_steps);
    // The build uses -mfma, so the compiler may fuse a multiply into the add. Rounding each
    // product on its own matches the critic's vectorized sum, which does not fuse the same way
    volatile float critical_term = kCriticalWeight * raw[i];
    volatile float repulsion_term = kRepulsionWeight * norm;
    float term = critical_term + repulsion_term;
    if (c.power > 1) {
      Eigen::ArrayXf powered(1);
      powered(0) = term;
      term = powered.pow(c.power)(0);
    }
    EXPECT_EQ(costs(i), term) << "trajectory " << i;
  }
  EXPECT_EQ(fail, c.fail);

  if (c.track_collisions) {
    ASSERT_EQ(static_cast<int>(hits.size()), batch);
    for (int i = 0; i < batch; ++i) {
      // The critic only ever sets a flag, so a pre-seeded flag stays set
      EXPECT_EQ(hits[i], static_cast<bool>(c.collided[i]) || c.preseed_collision)
        << "trajectory " << i;
    }
  } else {
    EXPECT_TRUE(hits.empty());
  }
}

// ---- Case factories -------------------------------------------------------------------------

// One trajectory, one time step on cell (5, 5)
Case single_case(float cost, bool collided)
{
  Case c;
  c.time_steps = 1;
  c.traj = {{cell_step(5, 5, cost)}};
  c.collided = {collided};
  c.fail = collided;
  return c;
}

Case FreeSpace_case()
{
  Case c;
  c.time_steps = 3;
  c.traj = {{cell_step(5, 5, 0.0f), cell_step(6, 5, 0.0f), cell_step(7, 5, 0.0f)}};
  c.collided = {false};
  c.fail = false;
  return c;
}

Case SecondColumnLethal_case()
{
  Case c;
  c.time_steps = 2;
  c.traj = {{cell_step(5, 5, 0.0f), cell_step(6, 5, kLethal)}};
  c.collided = {true};
  c.fail = true;
  return c;
}

Case Lethal_case()
{
  return single_case(kLethal, true);
}

Case RepulsionBeforeCollision_case()
{
  Case c;
  c.time_steps = 2;
  c.traj = {
    {cell_step(5, 5, 100.0f), cell_step(6, 5, kLethal)},
    {cell_step(5, 6, 0.0f), cell_step(6, 6, 0.0f)}};
  c.collided = {true, false};
  c.fail = false;
  return c;
}

Case InscribedCollides_case()
{
  return single_case(kInscribed, true);
}

Case Cost252Distance_case()
{
  return single_case(252.0f, false);
}

Case NearGoalKeepsMargin_case()
{
  Case c;
  c.local_path_length = 0.1f;
  c.time_steps = 1;
  c.traj = {{cell_step(5, 5, 100.0f)}, {cell_step(5, 6, 0.0f)}};
  c.collided = {false, false};
  c.fail = false;
  return c;
}

Case NearGoalLethal_case()
{
  Case c = single_case(kLethal, true);
  c.local_path_length = 0.1f;
  return c;
}

Case UnknownUntracked_case()
{
  return single_case(kNoInformation, true);
}

Case UnknownTrackedDistance_case()
{
  Case c = single_case(kNoInformation, false);
  c.track_unknown = true;
  return c;
}

Case OutOfMapLow_case()
{
  Case c;
  c.time_steps = 1;
  c.traj = {{world_step(-1.0f, 1.0f)}};
  c.collided = {true};
  c.fail = true;
  return c;
}

Case OutOfMapHigh_case()
{
  Case c;
  c.time_steps = 1;
  c.traj = {{world_step(100.0f, 1.0f)}};
  c.collided = {true};
  c.fail = true;
  return c;
}

Case NoInflationFree_case()
{
  Case c = single_case(100.0f, false);
  c.inflation = false;
  return c;
}

Case NoInflationLethal_case()
{
  Case c = single_case(kLethal, true);
  c.inflation = false;
  return c;
}

Case TwoTrajectoryPreference_case()
{
  Case c;
  c.time_steps = 1;
  c.traj = {{cell_step(5, 5, 200.0f)}, {cell_step(5, 6, 50.0f)}};
  c.collided = {false, false};
  c.fail = false;
  return c;
}

Case SingleTrajectoryPreferenceZero_case()
{
  return single_case(100.0f, false);
}

Case PowerTwo_case()
{
  Case c = single_case(100.0f, false);
  c.power = 2;
  return c;
}

Case MixedFailFlag_case()
{
  Case c;
  c.time_steps = 1;
  c.initial_fail = true;
  c.traj = {{cell_step(5, 5, kLethal)}, {cell_step(5, 6, 0.0f)}};
  c.collided = {true, false};
  c.fail = false;
  return c;
}

Case AllCollide_case()
{
  Case c;
  c.time_steps = 1;
  c.traj = {{cell_step(5, 5, kLethal)}, {cell_step(5, 6, kLethal)}};
  c.collided = {true, true};
  c.fail = true;
  return c;
}

Case EmptyCollisionVector_case()
{
  Case c = single_case(100.0f, false);
  c.track_collisions = false;
  return c;
}

Case PreseededFlagStays_case()
{
  Case c = single_case(0.0f, false);
  c.preseed_collision = true;
  return c;
}

Case DisabledCritic_case()
{
  Case c = single_case(kLethal, true);
  c.enabled = false;
  c.initial_fail = true;
  return c;
}

}  // namespace

TEST(ObstaclesCriticParity, FreeSpace)
{
  expect_classical(FreeSpace_case());
}

TEST(ObstaclesCriticParity, SecondColumnLethal)
{
  expect_classical(SecondColumnLethal_case());
}

TEST(ObstaclesCriticParity, Lethal)
{
  expect_classical(Lethal_case());
}

TEST(ObstaclesCriticParity, RepulsionBeforeCollision)
{
  expect_classical(RepulsionBeforeCollision_case());
}

TEST(ObstaclesCriticParity, InscribedCollides)
{
  expect_classical(InscribedCollides_case());
}

TEST(ObstaclesCriticParity, Cost252Distance)
{
  expect_classical(Cost252Distance_case());
}

TEST(ObstaclesCriticParity, NearGoalKeepsMargin)
{
  expect_classical(NearGoalKeepsMargin_case());
}

TEST(ObstaclesCriticParity, NearGoalLethal)
{
  expect_classical(NearGoalLethal_case());
}

TEST(ObstaclesCriticParity, UnknownUntracked)
{
  expect_classical(UnknownUntracked_case());
}

TEST(ObstaclesCriticParity, UnknownTrackedDistance)
{
  expect_classical(UnknownTrackedDistance_case());
}

TEST(ObstaclesCriticParity, OutOfMapLow)
{
  expect_classical(OutOfMapLow_case());
}

TEST(ObstaclesCriticParity, OutOfMapHigh)
{
  expect_classical(OutOfMapHigh_case());
}

TEST(ObstaclesCriticParity, NoInflationFree)
{
  expect_classical(NoInflationFree_case());
}

TEST(ObstaclesCriticParity, NoInflationLethal)
{
  expect_classical(NoInflationLethal_case());
}

TEST(ObstaclesCriticParity, TwoTrajectoryPreference)
{
  expect_classical(TwoTrajectoryPreference_case());
}

TEST(ObstaclesCriticParity, SingleTrajectoryPreferenceZero)
{
  expect_classical(SingleTrajectoryPreferenceZero_case());
}

TEST(ObstaclesCriticParity, PowerTwo)
{
  expect_classical(PowerTwo_case());
}

TEST(ObstaclesCriticParity, MixedFailFlag)
{
  expect_classical(MixedFailFlag_case());
}

TEST(ObstaclesCriticParity, AllCollide)
{
  expect_classical(AllCollide_case());
}

TEST(ObstaclesCriticParity, EmptyCollisionVector)
{
  expect_classical(EmptyCollisionVector_case());
}

TEST(ObstaclesCriticParity, PreseededFlagStays)
{
  expect_classical(PreseededFlagStays_case());
}

TEST(ObstaclesCriticParity, DisabledCritic)
{
  expect_classical(DisabledCritic_case());
}
