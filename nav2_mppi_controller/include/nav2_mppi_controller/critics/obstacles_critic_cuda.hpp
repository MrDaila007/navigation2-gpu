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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_CUDA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_CUDA_HPP_

namespace mppi::critics
{

/**
 * @struct mppi::critics::ObstaclesCriticDeviceGather
 * @brief Scalar inputs for gathering trajectory center costs from a costmap on the device
 */
struct ObstaclesCriticDeviceGather
{
  double origin_x;
  double origin_y;
  double resolution;
  unsigned int size_x;
  unsigned int size_y;
  int rows;
  int time_cols;
};

/**
 * @struct mppi::critics::ObstaclesCriticCudaTimings
 * @brief Stage timings in milliseconds of the most recent device gather call
 */
struct ObstaclesCriticCudaTimings
{
  float costmap_h2d_ms;
  float trajectory_h2d_ms;
  float kernel_ms;
  float d2h_ms;
};

/**
 * @class mppi::critics::ObstaclesCriticCuda
 * @brief Gathers the costmap cost under each trajectory center pose on a CUDA device.
 * Without CUDA support compiled in, every call reports that no result was produced.
 */
class ObstaclesCriticCuda
{
public:
  ObstaclesCriticCuda();
  ~ObstaclesCriticCuda();
  ObstaclesCriticCuda(const ObstaclesCriticCuda &) = delete;
  ObstaclesCriticCuda & operator=(const ObstaclesCriticCuda &) = delete;

  /**
   * @brief Gather the center cost of every sample without any reduction
   * @param x Column-major x samples, sample (row, time) is at time * rows + row
   * @param y Column-major y samples, same layout as x
   * @param costmap Costmap cells, row-major with size_x cells per row
   * @param params Gather parameters
   * @param pose_costs [out] Same layout as x, one entry per time column
   * @return false if the caller must not read pose_costs
   */
  bool gather_center(
    const float * x, const float * y,
    const unsigned char * costmap,
    const ObstaclesCriticDeviceGather & params,
    float * pose_costs);

private:
  struct DeviceState;
  DeviceState * state_{nullptr};
};

/// @brief True if this library was built with CUDA gathering
bool obstacles_critic_cuda_compiled();
/// @brief True if a usable CUDA device is present
bool obstacles_critic_cuda_device_available();
/// @brief Number of device kernel launches since the last reset
int obstacles_critic_cuda_launch_count();
/// @brief Reset the kernel launch counter to zero
void reset_obstacles_critic_cuda_launch_count();
/// @brief Timings of the most recent device gather call
ObstaclesCriticCudaTimings obstacles_critic_cuda_last_timings();

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_CUDA_HPP_
