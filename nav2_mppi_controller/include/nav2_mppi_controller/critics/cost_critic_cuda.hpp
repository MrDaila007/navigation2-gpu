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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_CUDA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_CUDA_HPP_

namespace mppi::critics
{

/**
 * @struct mppi::critics::CostCriticDeviceScore
 * @brief Scalar inputs for scoring trajectory centers against a costmap on the device
 */
struct CostCriticDeviceScore
{
  float origin_x;
  float origin_y;
  float resolution;
  unsigned int size_x;
  unsigned int size_y;
  float collision_cost;
  float critical_cost;
  unsigned int near_collision_cost;
  bool is_tracking_unknown;
  bool near_goal;
  int rows;
  int time_cols;
  int point_step;
  bool track_collisions;
};

/**
 * @struct mppi::critics::CostCriticCudaTimings
 * @brief Stage timings in milliseconds of the most recent device scoring call
 */
struct CostCriticCudaTimings
{
  float costmap_h2d_ms;
  float trajectory_h2d_ms;
  float kernel_ms;
  float d2h_ms;
};

/**
 * @class mppi::critics::CostCriticCuda
 * @brief Scores trajectory center poses against a costmap on a CUDA device.
 * Without CUDA support compiled in, every call reports that no result was produced.
 */
class CostCriticCuda
{
public:
  CostCriticCuda();
  ~CostCriticCuda();
  CostCriticCuda(const CostCriticCuda &) = delete;
  CostCriticCuda & operator=(const CostCriticCuda &) = delete;

  /**
   * @brief Score the center point of each trajectory
   * @param x Column-major x samples, sample (row, time) is at time * rows + row
   * @param y Column-major y samples, same layout as x
   * @param costmap Costmap cells, row-major with size_x cells per row
   * @param params Scoring parameters
   * @param repulsive [out] Repulsive cost per trajectory
   * @param collided [out] Collision flag per trajectory
   * @param all_collided [out] True if every trajectory collided
   * @return false if the device did not produce a buffer the caller may read
   */
  bool score_center(
    const float * x, const float * y,
    const unsigned char * costmap,
    const CostCriticDeviceScore & params,
    float * repulsive, unsigned char * collided, bool * all_collided);

  /**
   * @brief Gather the center cost of every sample without any reduction
   * @param pose_costs [out] Cost per sample, same layout as x
   * @return false if the device did not produce a buffer the caller may read
   */
  bool gather_center(
    const float * x, const float * y,
    const unsigned char * costmap,
    const CostCriticDeviceScore & params,
    float * pose_costs);
};

/// @brief True if this library was built with CUDA scoring
bool cost_critic_cuda_compiled();
/// @brief True if a usable CUDA device is present
bool cost_critic_cuda_device_available();
/// @brief Number of device kernel launches since the last reset
int cost_critic_cuda_launch_count();
/// @brief Reset the kernel launch counter to zero
void reset_cost_critic_cuda_launch_count();
/// @brief Timings of the most recent device scoring call
CostCriticCudaTimings cost_critic_cuda_last_timings();

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_CUDA_HPP_
