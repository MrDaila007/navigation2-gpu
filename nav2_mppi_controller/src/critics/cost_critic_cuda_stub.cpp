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

// Used when the device kernel is not linked. With NAV2_MPPI_HAS_CUDA the library
// reports that it was compiled and whether a device exists, but score_center
// still produces no buffer and does not count a launch.

#include "nav2_mppi_controller/critics/cost_critic_cuda.hpp"

#if NAV2_MPPI_HAS_CUDA
#include <cuda_runtime_api.h>
#endif

namespace mppi::critics
{

CostCriticCuda::CostCriticCuda() = default;
CostCriticCuda::~CostCriticCuda() = default;

bool CostCriticCuda::score_center(
  const float *, const float *, const unsigned char *, const CostCriticDeviceScore &,
  float *, unsigned char *, bool *)
{
  return false;
}

bool CostCriticCuda::gather_center(
  const float *, const float *, const unsigned char *, const CostCriticDeviceScore &,
  float *)
{
  return false;
}

#if NAV2_MPPI_HAS_CUDA
bool cost_critic_cuda_compiled() {return true;}

bool cost_critic_cuda_device_available()
{
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess) {
    return false;
  }
  return count > 0;
}
#else
bool cost_critic_cuda_compiled() {return false;}

bool cost_critic_cuda_device_available() {return false;}
#endif

int cost_critic_cuda_launch_count() {return 0;}

void reset_cost_critic_cuda_launch_count() {}

CostCriticCudaTimings cost_critic_cuda_last_timings() {return {0.0f, 0.0f, 0.0f, 0.0f};}

}  // namespace mppi::critics
