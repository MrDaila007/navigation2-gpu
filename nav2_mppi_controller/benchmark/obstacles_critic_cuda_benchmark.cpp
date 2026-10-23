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

#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic_cuda.hpp"
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

// Times a full ObstaclesCritic::score() with use_cuda enabled. There is no time or byte limit.
static void BM_ObstaclesCriticCudaScore(benchmark::State & state)
{
  using namespace mppi;  // NOLINT

  const int batch = 2000;
  const int time_steps = 56;
  const unsigned int cell = 5;
  const float center = (static_cast<float>(cell) + 0.5f) * 0.1f;

  auto node = std::make_shared<nav2::LifecycleNode>("obstacles_critic_cuda_bench_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "obstacles_critic_cuda_bench_costmap", "", true);
  std::string handler_name = "bench";
  ParametersHandler param_handler(node, handler_name);

  costmap_ros->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));

  const std::string critic_name = "cuda_bench";
  {
    auto getParam = param_handler.getParamGetter(critic_name);
    bool consider_footprint = false;
    bool use_cuda = true;
    getParam(consider_footprint, "consider_footprint", false);
    getParam(use_cuda, "use_cuda", true);
  }

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto * costmap = costmap_ros->getCostmap();
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, 0);
    }
  }

  critics::ObstaclesCritic critic;
  critic.on_configure(node, "mppi", critic_name, costmap_ros, &param_handler);

  models::State mppi_state;
  mppi_state.local_path_length = 10.0f;
  models::Trajectories trajectories;
  trajectories.reset(batch, time_steps);
  for (int i = 0; i < batch; ++i) {
    for (int t = 0; t < time_steps; ++t) {
      trajectories.x(i, t) = center;
      trajectories.y(i, t) = center;
      trajectories.yaws(i, t) = 0.0f;
    }
  }
  models::Path path;
  geometry_msgs::msg::Pose goal;
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(batch);
  float model_dt = 0.1f;
  CriticData data =
  {mppi_state, trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt, {}};
  data.trajectories_in_collision = std::vector<bool>(batch, false);

  for (auto _ : state) {
    costs.setZero();
    data.fail_flag = false;
    data.trajectories_in_collision.assign(batch, false);
    critic.score(data);
    benchmark::DoNotOptimize(costs.data());
  }

  if (critics::obstacles_critic_cuda_device_available()) {
    const auto timings = critics::obstacles_critic_cuda_last_timings();
    state.counters["costmap_h2d_ms"] = timings.costmap_h2d_ms;
    state.counters["trajectory_h2d_ms"] = timings.trajectory_h2d_ms;
    state.counters["kernel_ms"] = timings.kernel_ms;
    state.counters["d2h_ms"] = timings.d2h_ms;
  }
}

BENCHMARK(BM_ObstaclesCriticCudaScore)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
