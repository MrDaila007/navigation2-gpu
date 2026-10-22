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

// Locks the classical (CPU) scoring behavior of CostCritic against hand-computed
// expectations. Every expected value below is derived in the test from the
// repulsive costs of the scenario, never read back from the critic.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_mppi_controller/critics/cost_critic_cuda.hpp"
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

struct PaintedCell
{
  unsigned int mx;
  unsigned int my;
  unsigned char cost;
};

// One trajectory pose: either the center of a map cell or an explicit world point
struct Sample
{
  float x;
  float y;
};

Sample at_cell(unsigned int mx, unsigned int my)
{
  return {world_from_cell(mx), world_from_cell(my)};
}

Sample at_world(float x, float y)
{
  return {x, y};
}

struct ClassicalCase
{
  bool track_unknown{false};
  bool consider_footprint{false};
  int power{1};
  int point_step{1};
  int time_steps{1};
  int batch{1};
  float local_path_length{10.0f};
  int near_collision_cost{253};
  float cost_weight{254.0f};
  float collision_cost{1000000.0f};
  float critical_cost{300.0f};
  float near_goal_distance{0.5f};
  bool enabled{true};
  bool track_collisions{true};
  // samples[trajectory][time]
  std::vector<std::vector<Sample>> samples;
  std::vector<PaintedCell> painted;
  std::vector<float> repulsive;
  bool fail{false};
  std::vector<bool> collided;
  // When non-empty and track_collisions is set, these flags are already in
  // trajectories_in_collision before score(). Empty means start all false.
  std::vector<bool> initial_collisions;
};

// Builds the scenario, runs one critic named critic_name with the given use_cuda value, and
// returns the resulting costs, fail flag and collision flags without asserting the formula
void score_case(
  const ClassicalCase & c, const std::string & critic_name, bool use_cuda_flag,
  Eigen::ArrayXf & out_costs, bool & out_fail, std::vector<bool> & out_hits)
{
  using namespace mppi;  // NOLINT

  ASSERT_EQ(static_cast<int>(c.samples.size()), c.batch);

  auto node = std::make_shared<nav2::LifecycleNode>(critic_name + "_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    critic_name + "_costmap", "", true);
  std::string handler_name = "parity";
  ParametersHandler param_handler(node, handler_name);

  // Costmap parameter must exist before the costmap reads it in on_configure
  costmap_ros->declare_parameter(
    "track_unknown_space", rclcpp::ParameterValue(c.track_unknown));
  if (c.consider_footprint) {
    costmap_ros->declare_parameter("plugins", std::vector<std::string>{});
    costmap_ros->declare_parameter(
      "footprint",
      std::string("[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]"));
  }

  // Critic parameters, declared with the same C++ types as CostCritic::initialize
  bool consider_footprint = c.consider_footprint;
  int power = c.power;
  float cost_weight = c.cost_weight;
  float critical_cost = c.critical_cost;
  int near_collision_cost = c.near_collision_cost;
  float collision_cost = c.collision_cost;
  float near_goal_distance = c.near_goal_distance;
  int point_step = c.point_step;
  bool enabled = c.enabled;
  bool use_cuda = use_cuda_flag;
  {
    auto getParam = param_handler.getParamGetter(critic_name);
    getParam(consider_footprint, "consider_footprint", c.consider_footprint);
    getParam(power, "cost_power", c.power);
    getParam(cost_weight, "cost_weight", c.cost_weight);
    getParam(critical_cost, "critical_cost", c.critical_cost);
    getParam(near_collision_cost, "near_collision_cost", c.near_collision_cost);
    getParam(collision_cost, "collision_cost", c.collision_cost);
    getParam(near_goal_distance, "near_goal_distance", c.near_goal_distance);
    getParam(point_step, "trajectory_point_step", c.point_step);
    getParam(enabled, "enabled", c.enabled);
    getParam(use_cuda, "use_cuda", use_cuda_flag);
  }

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  // Zero the master grid, then paint the scenario
  auto * costmap = costmap_ros->getCostmap();
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, 0);
    }
  }
  for (const auto & cell : c.painted) {
    costmap->setCost(cell.mx, cell.my, cell.cost);
  }

  mppi::critics::CostCritic critic;
  critic.on_configure(node, "mppi", critic_name, costmap_ros, &param_handler);

  models::State state;
  state.local_path_length = c.local_path_length;
  models::Trajectories trajectories;
  trajectories.reset(c.batch, c.time_steps);
  for (int i = 0; i < c.batch; ++i) {
    ASSERT_EQ(static_cast<int>(c.samples[i].size()), c.time_steps);
    for (int t = 0; t < c.time_steps; ++t) {
      trajectories.x(i, t) = c.samples[i][t].x;
      trajectories.y(i, t) = c.samples[i][t].y;
      // yaws stay zero
    }
  }
  models::Path path;
  geometry_msgs::msg::Pose goal;
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(c.batch);
  float model_dt = 0.1f;
  CriticData data =
  {state, trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt, {}};
  if (c.track_collisions) {
    if (c.initial_collisions.empty()) {
      data.trajectories_in_collision = std::vector<bool>(c.batch, false);
    } else {
      ASSERT_EQ(static_cast<int>(c.initial_collisions.size()), c.batch);
      data.trajectories_in_collision = c.initial_collisions;
    }
  }

  critic.score(data);

  out_costs = costs;
  out_fail = data.fail_flag;
  out_hits.assign(
    data.trajectories_in_collision.begin(), data.trajectories_in_collision.end());
}

void expect_classical(const ClassicalCase & c)
{
  ASSERT_EQ(static_cast<int>(c.samples.size()), c.batch);
  ASSERT_EQ(static_cast<int>(c.repulsive.size()), c.batch);
  ASSERT_EQ(static_cast<int>(c.collided.size()), c.batch);

  Eigen::ArrayXf costs;
  bool fail = false;
  std::vector<bool> hits;
  score_case(c, "classical", false, costs, fail, hits);
  ASSERT_EQ(static_cast<int>(costs.size()), c.batch);

  if (!c.enabled) {
    // A disabled critic does not score at all
    for (int i = 0; i < c.batch; ++i) {
      EXPECT_EQ(costs(i), 0.0f) << "trajectory " << i;
    }
    EXPECT_FALSE(fail);
    if (c.track_collisions) {
      ASSERT_EQ(static_cast<int>(hits.size()), c.batch);
      for (int i = 0; i < c.batch; ++i) {
        EXPECT_FALSE(hits[i]) << "trajectory " << i;
      }
    } else {
      EXPECT_TRUE(hits.empty());
    }
    return;
  }

  const int strided_cols =
    static_cast<int>(std::floor((c.time_steps - 1) / static_cast<double>(c.point_step))) + 1;
  const float scale = (c.cost_weight / 254.0f) / static_cast<float>(strided_cols);

  Eigen::ArrayXf expected(c.batch);
  for (int i = 0; i < c.batch; ++i) {
    expected(i) = c.repulsive[i] * scale;
  }
  if (c.power > 1) {
    expected = expected.pow(c.power);
  }

  for (int i = 0; i < c.batch; ++i) {
    EXPECT_EQ(costs(i), expected(i)) << "trajectory " << i;
  }
  EXPECT_EQ(fail, c.fail);

  if (c.track_collisions) {
    ASSERT_EQ(static_cast<int>(hits.size()), c.batch);
    for (int i = 0; i < c.batch; ++i) {
      EXPECT_EQ(hits[i], c.collided[i]) << "trajectory " << i;
    }
  } else {
    EXPECT_TRUE(hits.empty());
  }
}

// ---- Case factories -------------------------------------------------------------------------

// Builds a single trajectory case whose samples sit on the given cells (one per time step)
ClassicalCase single_traj_case(
  const std::vector<PaintedCell> & painted, float repulsive, bool collided, bool fail)
{
  ClassicalCase c;
  c.batch = 1;
  c.time_steps = static_cast<int>(painted.size());
  c.point_step = 1;
  c.power = 1;
  c.painted = painted;
  std::vector<Sample> row;
  for (const auto & cell : painted) {
    row.push_back(at_cell(cell.mx, cell.my));
  }
  c.samples.push_back(row);
  c.repulsive = {repulsive};
  c.collided = {collided};
  c.fail = fail;
  return c;
}

ClassicalCase FreeSpace_case()
{
  ClassicalCase c;
  c.batch = 1;
  c.time_steps = 3;
  c.samples = {{at_cell(5, 5), at_cell(5, 5), at_cell(5, 5)}};
  c.repulsive = {0.0f};
  c.collided = {false};
  c.fail = false;
  return c;
}

ClassicalCase SumInflated_case()
{
  return single_traj_case({{10, 10, 10}, {11, 10, 20}, {12, 10, 30}}, 60.0f, false, false);
}

ClassicalCase EarlyExitLethal_case()
{
  return single_traj_case(
    {{10, 10, 254}, {11, 10, 50}, {12, 10, 50}}, 1000000.0f, true, true);
}

ClassicalCase InscribedCollides_case()
{
  return single_traj_case({{4, 4, 253}}, 1000000.0f, true, true);
}

ClassicalCase Cost252AddsItself_case()
{
  return single_traj_case({{4, 4, 252}}, 252.0f, false, false);
}

ClassicalCase NearGoalDrop_case()
{
  auto c = single_traj_case({{10, 10, 10}, {11, 10, 20}, {12, 10, 30}}, 0.0f, false, false);
  c.local_path_length = 0.1f;
  return c;
}

ClassicalCase NearGoalCritical_case()
{
  auto c = single_traj_case({{10, 10, 10}, {11, 10, 150}, {12, 10, 30}}, 300.0f, false, false);
  c.local_path_length = 0.1f;
  c.near_collision_cost = 100;
  return c;
}

ClassicalCase NearGoalLethal_case()
{
  auto c = single_traj_case(
    {{10, 10, 254}, {11, 10, 10}, {12, 10, 10}}, 1000000.0f, true, true);
  c.local_path_length = 0.1f;
  return c;
}

ClassicalCase UnknownTracked_case()
{
  auto c = single_traj_case({{6, 6, 255}}, 300.0f, false, false);
  c.track_unknown = true;
  return c;
}

ClassicalCase UnknownUntracked_case()
{
  return single_traj_case({{6, 6, 255}}, 1000000.0f, true, true);
}

ClassicalCase OutOfMapLow_case()
{
  ClassicalCase c;
  c.batch = 1;
  c.time_steps = 1;
  c.samples = {{at_world(-1.0f, 1.0f)}};
  c.repulsive = {1000000.0f};
  c.collided = {true};
  c.fail = true;
  return c;
}

ClassicalCase OutOfMapHigh_case()
{
  ClassicalCase c;
  c.batch = 1;
  c.time_steps = 1;
  c.samples = {{at_world(100.0f, 1.0f)}};
  c.repulsive = {1000000.0f};
  c.collided = {true};
  c.fail = true;
  return c;
}

ClassicalCase StrideSkipsLethal_case()
{
  const unsigned char costs[5] = {10, 254, 20, 254, 30};
  ClassicalCase c;
  c.batch = 1;
  c.time_steps = 5;
  c.point_step = 2;
  std::vector<Sample> row;
  for (unsigned int t = 0; t < 5; ++t) {
    c.painted.push_back({10 + t, 10, costs[t]});
    row.push_back(at_cell(10 + t, 10));
  }
  c.samples = {row};
  c.repulsive = {60.0f};
  c.collided = {false};
  c.fail = false;
  return c;
}

ClassicalCase PowerTwo_case()
{
  auto c = single_traj_case({{7, 7, 10}}, 10.0f, false, false);
  c.power = 2;
  return c;
}

ClassicalCase MixedFailFlag_case()
{
  ClassicalCase c;
  c.batch = 2;
  c.time_steps = 1;
  c.painted = {{8, 8, 254}, {9, 9, 0}};
  c.samples = {{at_cell(8, 8)}, {at_cell(9, 9)}};
  c.repulsive = {1000000.0f, 0.0f};
  c.collided = {true, false};
  c.fail = false;
  return c;
}

ClassicalCase AllCollide_case()
{
  ClassicalCase c;
  c.batch = 2;
  c.time_steps = 1;
  c.painted = {{8, 8, 254}, {9, 9, 254}};
  c.samples = {{at_cell(8, 8)}, {at_cell(9, 9)}};
  c.repulsive = {1000000.0f, 1000000.0f};
  c.collided = {true, true};
  c.fail = true;
  return c;
}

ClassicalCase EmptyCollisionVector_case()
{
  auto c = SumInflated_case();
  c.track_collisions = false;
  return c;
}

ClassicalCase DisabledCritic_case()
{
  auto c = single_traj_case({{4, 4, 254}}, 0.0f, false, false);
  c.enabled = false;
  return c;
}

// Lethal cells on the border of the 5x5 block centered on cell (10, 10).
// Corners belong to both edges and are painted twice at the same cost.
void paint_lethal_ring(std::vector<PaintedCell> & painted)
{
  for (unsigned int my = 8; my <= 12; ++my) {
    painted.push_back({8, my, 254});
    painted.push_back({12, my, 254});
  }
  for (unsigned int mx = 8; mx <= 12; ++mx) {
    painted.push_back({mx, 8, 254});
    painted.push_back({mx, 12, 254});
  }
}

// One footprint sample at cell (10, 10). The center is painted after any ring.
ClassicalCase footprint_center_case(
  unsigned char center_cost, bool ring, float repulsive, bool collided, bool fail)
{
  ClassicalCase c;
  c.consider_footprint = true;
  c.cost_weight = 254.0f;
  c.batch = 1;
  c.time_steps = 1;
  c.point_step = 1;
  if (ring) {
    paint_lethal_ring(c.painted);
  }
  c.painted.push_back({10, 10, center_cost});
  c.samples = {{at_cell(10, 10)}};
  c.repulsive = {repulsive};
  c.collided = {collided};
  c.fail = fail;
  return c;
}

ClassicalCase FootprintHit_case()
{
  return footprint_center_case(200, true, 1000000.0f, true, true);
}

ClassicalCase FootprintClear_case()
{
  return footprint_center_case(200, false, 200.0f, false, false);
}

ClassicalCase FreeCenterSkipsFootprint_case()
{
  return footprint_center_case(0, true, 0.0f, false, false);
}

ClassicalCase InscribedCenterFootprintClear_case()
{
  return footprint_center_case(253, false, 300.0f, false, false);
}

ClassicalCase FootprintEarlyExit_case()
{
  ClassicalCase c = footprint_center_case(200, true, 1000000.0f, true, true);
  c.time_steps = 2;
  c.point_step = 1;
  c.samples = {{at_cell(10, 10), at_cell(20, 10)}};
  c.painted.push_back({20, 10, 40});
  return c;
}

// Free trajectory already marked by an earlier critic, plus a lethal trajectory.
// The free flag must stay true. The lethal trajectory must become true.
ClassicalCase PreseededFreeCollision_case()
{
  ClassicalCase c;
  c.batch = 2;
  c.time_steps = 1;
  c.painted = {{9, 9, 0}, {8, 8, 254}};
  c.samples = {{at_cell(9, 9)}, {at_cell(8, 8)}};
  c.initial_collisions = {true, false};
  c.collided = {true, true};
  return c;
}

#if NAV2_MPPI_HAS_CUDA
void expect_cuda_matches_classical(const ClassicalCase & c)
{
  using namespace mppi::critics;  // NOLINT
  if (!cost_critic_cuda_device_available()) {
    GTEST_SKIP();
  }
  ASSERT_TRUE(cost_critic_cuda_compiled());
  ASSERT_TRUE(cost_critic_cuda_device_available());
  const int before = cost_critic_cuda_launch_count();
  const std::string case_name =
    ::testing::UnitTest::GetInstance()->current_test_info()->name();

  Eigen::ArrayXf cpu_costs;
  Eigen::ArrayXf cuda_costs;
  bool cpu_fail = false;
  bool cuda_fail = false;
  std::vector<bool> cpu_hits;
  std::vector<bool> cuda_hits;
  score_case(c, "classical", false, cpu_costs, cpu_fail, cpu_hits);
  score_case(c, "cuda_classical", true, cuda_costs, cuda_fail, cuda_hits);

  for (int i = 0; i < cpu_costs.size(); ++i) {
    EXPECT_EQ(cpu_costs(i), cuda_costs(i)) << case_name << " trajectory " << i;
  }
  EXPECT_EQ(cpu_fail, cuda_fail);
  EXPECT_EQ(cpu_hits, cuda_hits);
  const bool enabled = c.enabled;
  if (enabled) {
    EXPECT_GT(cost_critic_cuda_launch_count(), before);
  } else {
    EXPECT_EQ(cost_critic_cuda_launch_count(), before);
  }
}
#endif

}  // namespace

TEST(CostCriticParity, FreeSpace) {expect_classical(FreeSpace_case());}
TEST(CostCriticParity, SumInflated) {expect_classical(SumInflated_case());}
TEST(CostCriticParity, EarlyExitLethal) {expect_classical(EarlyExitLethal_case());}
TEST(CostCriticParity, InscribedCollides) {expect_classical(InscribedCollides_case());}
TEST(CostCriticParity, Cost252AddsItself) {expect_classical(Cost252AddsItself_case());}
TEST(CostCriticParity, NearGoalDrop) {expect_classical(NearGoalDrop_case());}
TEST(CostCriticParity, NearGoalCritical) {expect_classical(NearGoalCritical_case());}
TEST(CostCriticParity, NearGoalLethal) {expect_classical(NearGoalLethal_case());}
TEST(CostCriticParity, UnknownTracked) {expect_classical(UnknownTracked_case());}
TEST(CostCriticParity, UnknownUntracked) {expect_classical(UnknownUntracked_case());}
TEST(CostCriticParity, OutOfMapLow) {expect_classical(OutOfMapLow_case());}
TEST(CostCriticParity, OutOfMapHigh) {expect_classical(OutOfMapHigh_case());}
TEST(CostCriticParity, StrideSkipsLethal) {expect_classical(StrideSkipsLethal_case());}
TEST(CostCriticParity, PowerTwo) {expect_classical(PowerTwo_case());}
TEST(CostCriticParity, MixedFailFlag) {expect_classical(MixedFailFlag_case());}
TEST(CostCriticParity, AllCollide) {expect_classical(AllCollide_case());}
TEST(CostCriticParity, EmptyCollisionVector) {expect_classical(EmptyCollisionVector_case());}
TEST(CostCriticParity, DisabledCritic) {expect_classical(DisabledCritic_case());}
TEST(CostCriticParity, FootprintHit) {expect_classical(FootprintHit_case());}
TEST(CostCriticParity, FootprintClear) {expect_classical(FootprintClear_case());}
TEST(CostCriticParity, FreeCenterSkipsFootprint) {expect_classical(FreeCenterSkipsFootprint_case());}
TEST(CostCriticParity, InscribedCenterFootprintClear) {
  expect_classical(InscribedCenterFootprintClear_case());
}
TEST(CostCriticParity, FootprintEarlyExit) {expect_classical(FootprintEarlyExit_case());}

TEST(CostCriticParity, CudaFlagOffDoesNotLaunch)
{
  using namespace mppi::critics;  // NOLINT
  reset_cost_critic_cuda_launch_count();
  const int before = cost_critic_cuda_launch_count();
  // Score SumInflated through a critic whose use_cuda parameter is false.
  expect_classical(SumInflated_case());
#if !NAV2_MPPI_HAS_CUDA
  EXPECT_FALSE(cost_critic_cuda_compiled());
#endif
  EXPECT_EQ(cost_critic_cuda_launch_count(), before);
}

TEST(CostCriticParity, CudaFlagOnMatchesCpuWhenDeviceMissing)
{
  using namespace mppi::critics;  // NOLINT
  if (cost_critic_cuda_device_available()) {
    GTEST_SKIP();
  }
  reset_cost_critic_cuda_launch_count();
  const int before = cost_critic_cuda_launch_count();
  Eigen::ArrayXf cpu_costs;
  Eigen::ArrayXf cuda_costs;
  bool cpu_fail = false;
  bool cuda_fail = false;
  std::vector<bool> cpu_hits;
  std::vector<bool> cuda_hits;
  score_case(SumInflated_case(), "cpu_critic", false, cpu_costs, cpu_fail, cpu_hits);
  score_case(SumInflated_case(), "cuda_critic", true, cuda_costs, cuda_fail, cuda_hits);
  ASSERT_EQ(cpu_costs.size(), cuda_costs.size());
  for (int i = 0; i < cpu_costs.size(); ++i) {
    EXPECT_EQ(cpu_costs(i), cuda_costs(i));
  }
  EXPECT_EQ(cpu_fail, cuda_fail);
  EXPECT_EQ(cpu_hits, cuda_hits);
  EXPECT_EQ(cost_critic_cuda_launch_count(), before);
}

#if NAV2_MPPI_HAS_CUDA
TEST(CostCriticParity, CudaFreeSpace) {expect_cuda_matches_classical(FreeSpace_case());}
TEST(CostCriticParity, CudaSumInflated) {expect_cuda_matches_classical(SumInflated_case());}
TEST(CostCriticParity, CudaEarlyExitLethal) {expect_cuda_matches_classical(EarlyExitLethal_case());}
TEST(CostCriticParity, CudaInscribedCollides) {expect_cuda_matches_classical(InscribedCollides_case());}
TEST(CostCriticParity, CudaCost252AddsItself) {expect_cuda_matches_classical(Cost252AddsItself_case());}
TEST(CostCriticParity, CudaNearGoalDrop) {expect_cuda_matches_classical(NearGoalDrop_case());}
TEST(CostCriticParity, CudaNearGoalCritical) {expect_cuda_matches_classical(NearGoalCritical_case());}
TEST(CostCriticParity, CudaNearGoalLethal) {expect_cuda_matches_classical(NearGoalLethal_case());}
TEST(CostCriticParity, CudaUnknownTracked) {expect_cuda_matches_classical(UnknownTracked_case());}
TEST(CostCriticParity, CudaUnknownUntracked) {expect_cuda_matches_classical(UnknownUntracked_case());}
TEST(CostCriticParity, CudaOutOfMapLow) {expect_cuda_matches_classical(OutOfMapLow_case());}
TEST(CostCriticParity, CudaOutOfMapHigh) {expect_cuda_matches_classical(OutOfMapHigh_case());}
TEST(CostCriticParity, CudaStrideSkipsLethal) {expect_cuda_matches_classical(StrideSkipsLethal_case());}
TEST(CostCriticParity, CudaPowerTwo) {expect_cuda_matches_classical(PowerTwo_case());}
TEST(CostCriticParity, CudaMixedFailFlag) {expect_cuda_matches_classical(MixedFailFlag_case());}
TEST(CostCriticParity, CudaAllCollide) {expect_cuda_matches_classical(AllCollide_case());}
TEST(CostCriticParity, CudaEmptyCollisionVector) {
  expect_cuda_matches_classical(EmptyCollisionVector_case());
}
TEST(CostCriticParity, CudaDisabledCritic) {expect_cuda_matches_classical(DisabledCritic_case());}
TEST(CostCriticParity, CudaFootprintHit) {expect_cuda_matches_classical(FootprintHit_case());}
TEST(CostCriticParity, CudaFootprintClear) {expect_cuda_matches_classical(FootprintClear_case());}
TEST(CostCriticParity, CudaFreeCenterSkipsFootprint) {
  expect_cuda_matches_classical(FreeCenterSkipsFootprint_case());
}
TEST(CostCriticParity, CudaInscribedCenterFootprintClear) {
  expect_cuda_matches_classical(InscribedCenterFootprintClear_case());
}
TEST(CostCriticParity, CudaFootprintEarlyExit) {
  expect_cuda_matches_classical(FootprintEarlyExit_case());
}

TEST(CostCriticParity, CudaResizeUsesNewGrid)
{
  using namespace mppi;  // NOLINT
  using namespace mppi::critics;  // NOLINT
  if (!cost_critic_cuda_device_available()) {
    GTEST_SKIP();
  }

  // One costmap and one CPU and one CUDA critic, all kept alive across both scores
  auto node = std::make_shared<nav2::LifecycleNode>("resize_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "resize_costmap", "", true);
  std::string handler_name = "parity";
  ParametersHandler param_handler(node, handler_name);
  costmap_ros->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));

  const std::string cpu_name = "resize_cpu";
  const std::string cuda_name = "resize_cuda";
  for (const auto & name : {cpu_name, cuda_name}) {
    auto getParam = param_handler.getParamGetter(name);
    bool consider_footprint = false;
    float cost_weight = 254.0f;
    int point_step = 1;
    bool use_cuda = false;
    getParam(consider_footprint, "consider_footprint", false);
    getParam(cost_weight, "cost_weight", 254.0f);
    getParam(point_step, "trajectory_point_step", 1);
    getParam(use_cuda, "use_cuda", name == cuda_name);
  }

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto * costmap = costmap_ros->getCostmap();

  auto zero_grid = [costmap]() {
      for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
        for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
          costmap->setCost(mx, my, 0);
        }
      }
    };

  CostCritic cpu_critic;
  cpu_critic.on_configure(node, "mppi", cpu_name, costmap_ros, &param_handler);
  CostCritic cuda_critic;
  cuda_critic.on_configure(node, "mppi", cuda_name, costmap_ros, &param_handler);

  const float collision_cost = 1000000.0f;
  const float expected_cost = collision_cost * (254.0f / 254.0f) / 1.0f;

  // Scores one sample through both critics on the current grid and checks parity
  auto score_both = [&](float x, float y, bool lethal, const char * which) {
      const float want_cost = lethal ? expected_cost : 0.0f;
      auto run = [&](CostCritic & critic, Eigen::ArrayXf & costs, bool & fail,
        std::vector<bool> & hits) {
        models::State state;
        state.local_path_length = 10.0f;
        models::Trajectories trajectories;
        trajectories.reset(1, 1);
        trajectories.x(0, 0) = x;
        trajectories.y(0, 0) = y;
        models::Path path;
        geometry_msgs::msg::Pose goal;
        costs = Eigen::ArrayXf::Zero(1);
        float model_dt = 0.1f;
        CriticData data =
        {state, trajectories, path, goal, costs, model_dt,
          false, nullptr, nullptr, std::nullopt, std::nullopt, {}};
        data.trajectories_in_collision = std::vector<bool>(1, false);
        critic.score(data);
        fail = data.fail_flag;
        hits.assign(
          data.trajectories_in_collision.begin(), data.trajectories_in_collision.end());
      };

      Eigen::ArrayXf cpu_costs;
      Eigen::ArrayXf cuda_costs;
      bool cpu_fail = false;
      bool cuda_fail = false;
      std::vector<bool> cpu_hits;
      std::vector<bool> cuda_hits;
      const int before = cost_critic_cuda_launch_count();
      run(cpu_critic, cpu_costs, cpu_fail, cpu_hits);
      EXPECT_EQ(cost_critic_cuda_launch_count(), before) << which;
      run(cuda_critic, cuda_costs, cuda_fail, cuda_hits);
      EXPECT_GT(cost_critic_cuda_launch_count(), before) << which;

      ASSERT_EQ(cpu_costs.size(), 1) << which;
      ASSERT_EQ(cuda_costs.size(), 1) << which;
      EXPECT_EQ(cpu_costs(0), want_cost) << which;
      EXPECT_EQ(cuda_costs(0), want_cost) << which;
      EXPECT_EQ(cpu_costs(0), cuda_costs(0)) << which;
      EXPECT_EQ(cpu_fail, lethal) << which;
      EXPECT_EQ(cuda_fail, lethal) << which;
      EXPECT_EQ(cpu_hits, std::vector<bool>{lethal}) << which;
      EXPECT_EQ(cuda_hits, std::vector<bool>{lethal}) << which;
    };

  // First score: default grid, lethal cell (10, 10)
  zero_grid();
  costmap->setCost(10, 10, 254);
  const unsigned int old_size_x = costmap->getSizeInCellsX();
  score_both(world_from_cell(10), world_from_cell(10), true, "before resize");
  if (::testing::Test::HasFatalFailure()) {
    return;
  }

  // Second score: smaller grid with a different resolution and origin.
  // mx = (-0.3 - (-1.0)) / 0.2 = 3.5, which casts to cell 3.
  costmap->resizeMap(15, 15, 0.2, -1.0, -1.0);
  ASSERT_EQ(costmap->getSizeInCellsX(), 15u);
  ASSERT_LT(costmap->getSizeInCellsX(), old_size_x);
  zero_grid();
  costmap->setCost(3, 3, 254);
  score_both(-0.3f, -0.3f, true, "after resize");
  if (::testing::Test::HasFatalFailure()) {
    return;
  }
  // (-0.3, -0.3) is also off the old map, so the lethal score alone cannot tell stale
  // geometry from new. A free sample at (1.0, 1.0) is cell (10, 10) on the new grid
  // and sits inside the old 5 m map, so a stale origin or resolution would misread it.
  score_both(1.0f, 1.0f, false, "after resize, free cell");
}
#endif

TEST(CostCriticParity, PreseededFreeCollisionStaysTrue)
{
  using namespace mppi::critics;  // NOLINT
  const ClassicalCase c = PreseededFreeCollision_case();

  auto expect_hits = [](const std::vector<bool> & hits, const char * which) {
    ASSERT_EQ(hits.size(), 2u) << which;
    EXPECT_TRUE(hits[0]) << which << " free trajectory lost a pre-seeded collision";
    EXPECT_TRUE(hits[1]) << which << " lethal trajectory was not marked";
  };

  Eigen::ArrayXf cpu_costs;
  bool cpu_fail = false;
  std::vector<bool> cpu_hits;
  score_case(c, "preseed_cpu", false, cpu_costs, cpu_fail, cpu_hits);
  expect_hits(cpu_hits, "cpu");

#if NAV2_MPPI_HAS_CUDA
  const bool device = cost_critic_cuda_device_available();
  const int before = cost_critic_cuda_launch_count();
#endif
  Eigen::ArrayXf cuda_costs;
  bool cuda_fail = false;
  std::vector<bool> cuda_hits;
  score_case(c, "preseed_cuda", true, cuda_costs, cuda_fail, cuda_hits);
  expect_hits(cuda_hits, "cuda");
#if NAV2_MPPI_HAS_CUDA
  if (device) {
    EXPECT_GT(cost_critic_cuda_launch_count(), before);
  }
#endif
}
