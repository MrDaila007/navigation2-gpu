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

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "nav2_mppi_controller/critics/cost_critic_cuda.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic_cuda.hpp"
#include "factory.hpp"

namespace
{
using Command = std::array<double, 3>;
constexpr int kBatch = 2000;
constexpr int kHorizon = 56;
constexpr int kIterations = 2;
constexpr int kWarmup = 50;

struct Result
{
  std::vector<double> wall_us;
  std::vector<Command> commands;
  double cpu_us_per_call;
  int launches;
};

int parseCount(const char * text, int limit)
{
  int value = 0;
  const auto end = text + std::strlen(text);
  const auto parsed = std::from_chars(text, end, value);
  const bool converted = parsed.ec == std::errc{} && parsed.ptr == end;
  if (!converted || value < 1 || value > limit) {
    throw std::invalid_argument("sample/repetition count is out of range");
  }
  return value;
}

struct ControllerCleanup
{
  nav2_mppi_controller::MPPIController & controller;
  ~ControllerCleanup()
  {
    controller.deactivate();
    controller.cleanup();
  }
};

Result measure(
  bool cuda, const std::string & critic, bool footprint,
  const std::string & scene, int samples)
{
  using namespace mppi::critics;  // NOLINT
  reset_cost_critic_cuda_launch_count();
  reset_obstacles_critic_cuda_launch_count();
  TestOptimizerSettings settings{kBatch, kHorizon, kIterations, 10.0, "diff_drive", footprint};
  std::vector<std::string> critics{
    "ConstraintCritic", critic, "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
    "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"};
  std::vector<rclcpp::Parameter> params;
  setUpOptimizerParams(settings, critics, params);
  setUpControllerParams(false, params);
  params.emplace_back("dummy." + critic + ".use_cuda", cuda);
  params.emplace_back("dummy." + critic + ".consider_footprint", footprint);
  params.emplace_back("dummy.regenerate_noises", false);
  params.emplace_back("dummy.publish_optimal_trajectory", false);
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  auto node = getDummyNode(options);
  TestCostmapSettings map_settings;
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "baseline_costmap", "", true);
  // The footprint parameter must precede on_configure: setting just the polygon
  // afterward does not clear getUseRadius(), and CostCritic rejects that mismatch.
  costmap_ros->declare_parameter("footprint_padding", 0.0);
  costmap_ros->declare_parameter("global_frame", std::string("odom"));
  if (footprint) {
    costmap_ros->declare_parameter(
      "footprint",
      std::string("[[0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]"));
  } else {
    costmap_ros->declare_parameter("robot_radius", map_settings.footprint_size);
  }
  if (costmap_ros->on_configure(rclcpp_lifecycle::State{}) != nav2::CallbackReturn::SUCCESS) {
    throw std::runtime_error("costmap configuration failed");
  }
  auto * costmap = costmap_ros->getCostmap();
  *costmap = *getDummyCostmap(map_settings);
  // Free forward corridor at y=2; obstacle/inflation bands lie to its side.
  for (unsigned int y = 0; y < map_settings.cells_y; ++y) {
    for (unsigned int x = 0; x < map_settings.cells_x; ++x) {
      const unsigned char cost = scene == "bands" ?
        (y < 10 ? 254 : (y < 15 ? 220 : (y < 18 ? 80 : 0))) : 0;
      costmap->setCost(x, y, cost);
    }
  }
  auto tf_buffer = nav2::create_transform_buffer(node);
  nav2_mppi_controller::MPPIController controller;
  controller.configure(node, node->get_name(), tf_buffer, costmap_ros);
  controller.activate();
  ControllerCleanup cleanup{controller};
  auto pose = getDummyPointStamped(node, TestPose{2.0, 2.0});
  pose.pose.orientation.w = 1.0;
  auto path = getIncrementalDummyPath(node, TestPathSettings{{2.0, 2.0}, 21u, 0.05, 0.0});
  for (auto & point : path.poses) {
    point.pose.orientation.w = 1.0;
  }
  const auto goal = path.poses.back();
  const auto velocity = getDummyTwist();
  Result result;
  result.wall_us.reserve(samples);
  result.commands.reserve(samples + kWarmup);
  std::clock_t cpu_begin = 0;
  for (int call = 0; call < samples + kWarmup; ++call) {
    if (call == kWarmup) {
      cpu_begin = std::clock();
    }
    const auto begin = std::chrono::steady_clock::now();
    const auto cmd = controller.computeVelocityCommands(pose, velocity, nullptr, path, goal);
    const auto end = std::chrono::steady_clock::now();
    const Command values{cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z};
    for (double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("non-finite control command");
      }
    }
    result.commands.push_back(values);
    if (call >= kWarmup) {
      result.wall_us.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
    }
  }
  const auto cpu_end = std::clock();
  if (cpu_begin == static_cast<std::clock_t>(-1) ||
    cpu_end == static_cast<std::clock_t>(-1))
  {
    throw std::runtime_error("process CPU clock is unavailable");
  }
  result.cpu_us_per_call =
    1e6 * static_cast<double>(cpu_end - cpu_begin) / CLOCKS_PER_SEC / samples;
  result.launches = critic == "CostCritic" ?
    cost_critic_cuda_launch_count() : obstacles_critic_cuda_launch_count();
  const int expected = cuda ? (samples + kWarmup) * kIterations : 0;
  if (result.launches != expected) {
    throw std::runtime_error("backend launch count differs from expected iteration count");
  }
  return result;
}

void printResult(const Result & result, const std::string & backend, int repetition)
{
  auto sorted = result.wall_us;
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&](double p) {
      return sorted[static_cast<size_t>(std::ceil(p * sorted.size())) - 1];
    };
  double sum = 0.0;
  for (double sample : sorted) {
    sum += sample;
  }
  std::cout << "SUMMARY," << backend << ',' << repetition << ',' << sorted.size() << ',' <<
    sum / sorted.size() << ',' << percentile(0.5) << ',' << percentile(0.95) << ',' <<
    percentile(0.99) << ',' << sorted.back() << ',' << result.cpu_us_per_call << ',' <<
    result.launches << '\n';
  for (size_t i = 0; i < result.wall_us.size(); ++i) {
    std::cout << "SAMPLE," << backend << ',' << repetition << ',' << i << ',' <<
      result.wall_us[i] << '\n';
  }
}

void compareCommands(const Result & cpu, const Result & gpu)
{
  if (cpu.commands.size() != gpu.commands.size()) {
    throw std::runtime_error("command sequence sizes differ");
  }
  for (size_t i = 0; i < cpu.commands.size(); ++i) {
    for (size_t axis = 0; axis < cpu.commands[i].size(); ++axis) {
      if (std::memcmp(&cpu.commands[i][axis], &gpu.commands[i][axis], sizeof(double)) != 0) {
        throw std::runtime_error("CPU/GPU command bits differ at call " + std::to_string(i));
      }
    }
  }
  std::cout << "PARITY,command_bits," << cpu.commands.size() << ",PASS\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 5 || argc > 7) {
      throw std::invalid_argument(
              "usage: mppi_gpu_baseline <cpu|pair> <cost|obstacles> <0|1> <free|bands> "
              "[samples=1000] [repetitions=5]");
    }
    const std::string mode = argv[1];
    const std::string kind = argv[2];
    const std::string footprint_arg = argv[3];
    const std::string scene = argv[4];
    if ((mode != "cpu" && mode != "pair") || (kind != "cost" && kind != "obstacles") ||
      (footprint_arg != "0" && footprint_arg != "1") ||
      (scene != "free" && scene != "bands"))
    {
      throw std::invalid_argument("invalid mode, critic, footprint, or scene");
    }
    const int samples = argc > 5 ? parseCount(argv[5], 100000) : 1000;
    const int repeats = argc > 6 ? parseCount(argv[6], 20) : 5;
    if (mode == "pair" &&
      (!mppi::critics::cost_critic_cuda_device_available() ||
      !mppi::critics::obstacles_critic_cuda_device_available()))
    {
      throw std::runtime_error("pair mode requires CUDA; CPU fallback is not a GPU measurement");
    }
    rclcpp::init(0, nullptr);
    std::cout << std::setprecision(12);
    std::cout << "CONFIG," << kind << ',' << footprint_arg << ',' << scene << ',' << kBatch <<
      ',' << kHorizon << ',' << kIterations << ',' << kWarmup << '\n';
    std::cout << "FIELDS,backend,repetition,samples,mean_us,p50_us,p95_us,p99_us,max_us,"
      "process_cpu_us_per_call,launches_including_warmup\n";
    const std::string critic = kind == "cost" ? "CostCritic" : "ObstaclesCritic";
    for (int rep = 0; rep < repeats; ++rep) {
      // Alternate run order to reduce systematic thermal/order bias.
      Result cpu;
      Result gpu;
      if (mode == "pair" && rep % 2 == 1) {
        gpu = measure(true, critic, footprint_arg == "1", scene, samples);
        cpu = measure(false, critic, footprint_arg == "1", scene, samples);
      } else {
        cpu = measure(false, critic, footprint_arg == "1", scene, samples);
        if (mode == "pair") {
          gpu = measure(true, critic, footprint_arg == "1", scene, samples);
        }
      }
      printResult(cpu, "cpu", rep);
      if (mode == "pair") {
        compareCommands(cpu, gpu);
        printResult(gpu, "cuda", rep);
      }
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "BASELINE_ERROR: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
