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

// Used when the device kernel is not linked. It includes no CUDA headers, reports that no
// device is compiled in or available, counts no launches, and produces no buffer.

#include "nav2_mppi_controller/critics/obstacles_critic_cuda.hpp"

namespace mppi::critics
{

ObstaclesCriticCuda::ObstaclesCriticCuda() = default;
ObstaclesCriticCuda::~ObstaclesCriticCuda() = default;

bool ObstaclesCriticCuda::gather_center(
  const float *, const float *, const unsigned char *, const ObstaclesCriticDeviceGather &,
  float *)
{
  return false;
}

bool obstacles_critic_cuda_compiled() {return false;}

bool obstacles_critic_cuda_device_available() {return false;}

int obstacles_critic_cuda_launch_count() {return 0;}

void reset_obstacles_critic_cuda_launch_count() {}

ObstaclesCriticCudaTimings obstacles_critic_cuda_last_timings() {return {0.0f, 0.0f, 0.0f, 0.0f};}

}  // namespace mppi::critics
