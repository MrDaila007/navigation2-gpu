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

#include <cstddef>

#include <cuda_runtime_api.h>

#include "nav2_mppi_controller/critics/obstacles_critic_cuda.hpp"

namespace mppi::critics
{
namespace
{

constexpr int kBlockSize = 128;

int g_launch_count = 0;
ObstaclesCriticCudaTimings g_last_timings{0.0f, 0.0f, 0.0f, 0.0f};

template<typename T>
bool grow_n(T * & ptr, size_t & cap, size_t need)
{
  if (need <= cap) {
    return true;
  }
  if (ptr != nullptr) {
    cudaFree(ptr);
  }
  ptr = nullptr;
  cap = 0;
  if (need == 0) {
    return true;
  }
  if (cudaMalloc(reinterpret_cast<void **>(&ptr), need * sizeof(T)) != cudaSuccess) {
    ptr = nullptr;
    return false;
  }
  cap = need;
  return true;
}

float elapsed_ms(cudaEvent_t begin, cudaEvent_t end)
{
  float ms = 0.0f;
  if (cudaEventElapsedTime(&ms, begin, end) != cudaSuccess) {
    return 0.0f;
  }
  return ms;
}

// A device-to-host copy is already queued. Wait for that write, then clear the
// sticky error, before returning so the caller can reuse the host buffer.
bool fail_after_d2h(cudaStream_t stream)
{
  cudaStreamSynchronize(stream);
  cudaGetLastError();
  return false;
}

// One thread per sample. The value is the center-cell cost only, using the double
// origin and resolution: worldToMap, or 255 outside the map. No collision test and no log.
__global__ void gather_center_kernel(
  const float * __restrict__ x,
  const float * __restrict__ y,
  const unsigned char * __restrict__ costmap,
  ObstaclesCriticDeviceGather params,
  float * __restrict__ pose_costs)
{
  const int sample_count = params.rows * params.time_cols;
  const int idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= sample_count) {
    return;
  }

  const int i = idx % params.rows;
  const int j = idx / params.rows;
  const int addr = j * params.rows + i;
  const double wx = static_cast<double>(x[addr]);
  const double wy = static_cast<double>(y[addr]);

  float pose_cost = 255.0f;
  if (wx >= params.origin_x && wy >= params.origin_y) {
    const unsigned int mx =
      static_cast<unsigned int>((wx - params.origin_x) / params.resolution);
    const unsigned int my =
      static_cast<unsigned int>((wy - params.origin_y) / params.resolution);
    if (mx < params.size_x && my < params.size_y) {
      pose_cost = static_cast<float>(
        costmap[static_cast<size_t>(my) * params.size_x + mx]);
    }
  }

  pose_costs[addr] = pose_cost;
}

}  // namespace

struct ObstaclesCriticCuda::DeviceState
{
  cudaStream_t stream{nullptr};
  cudaEvent_t ev_costmap_begin{nullptr};
  cudaEvent_t ev_traj_begin{nullptr};
  cudaEvent_t ev_kernel_begin{nullptr};
  cudaEvent_t ev_d2h_begin{nullptr};
  cudaEvent_t ev_d2h_end{nullptr};
  bool ready{false};

  unsigned char * costmap{nullptr};
  size_t costmap_cap{0};
  float * x{nullptr};
  size_t x_cap{0};
  float * y{nullptr};
  size_t y_cap{0};
  float * pose_costs{nullptr};
  size_t pose_costs_cap{0};

  bool init()
  {
    if (ready) {
      return true;
    }
    if (cudaSetDevice(0) != cudaSuccess) {
      return false;
    }
    if (stream == nullptr && cudaStreamCreate(&stream) != cudaSuccess) {
      stream = nullptr;
      return false;
    }
    cudaEvent_t * events[] = {
      &ev_costmap_begin, &ev_traj_begin, &ev_kernel_begin, &ev_d2h_begin, &ev_d2h_end};
    for (cudaEvent_t * event : events) {
      if (*event == nullptr && cudaEventCreate(event) != cudaSuccess) {
        *event = nullptr;
        return false;
      }
    }
    ready = true;
    return true;
  }

  void release()
  {
    if (costmap != nullptr) {cudaFree(costmap);}
    if (x != nullptr) {cudaFree(x);}
    if (y != nullptr) {cudaFree(y);}
    if (pose_costs != nullptr) {cudaFree(pose_costs);}
    cudaEvent_t events[] = {
      ev_costmap_begin, ev_traj_begin, ev_kernel_begin, ev_d2h_begin, ev_d2h_end};
    for (cudaEvent_t event : events) {
      if (event != nullptr) {cudaEventDestroy(event);}
    }
    if (stream != nullptr) {cudaStreamDestroy(stream);}
    costmap = nullptr;
    x = nullptr;
    y = nullptr;
    pose_costs = nullptr;
    ev_costmap_begin = nullptr;
    ev_traj_begin = nullptr;
    ev_kernel_begin = nullptr;
    ev_d2h_begin = nullptr;
    ev_d2h_end = nullptr;
    stream = nullptr;
    ready = false;
  }
};

ObstaclesCriticCuda::ObstaclesCriticCuda()
: state_(new DeviceState)
{
}

ObstaclesCriticCuda::~ObstaclesCriticCuda()
{
  if (state_ == nullptr) {
    return;
  }
  state_->release();
  delete state_;
  state_ = nullptr;
}

bool ObstaclesCriticCuda::gather_center(
  const float * x, const float * y, const unsigned char * costmap,
  const ObstaclesCriticDeviceGather & params, float * pose_costs)
{
  if (state_ == nullptr) {
    return false;
  }
  if (params.rows < 0 || params.time_cols < 1 || !(params.resolution > 0.0)) {
    return false;
  }
  const size_t costmap_bytes =
    static_cast<size_t>(params.size_x) * static_cast<size_t>(params.size_y);
  const size_t traj_floats =
    static_cast<size_t>(params.rows) * static_cast<size_t>(params.time_cols);
  const int sample_count = params.rows * params.time_cols;
  if (sample_count > 0 && (x == nullptr || y == nullptr || pose_costs == nullptr)) {
    return false;
  }
  if (costmap_bytes > 0 && costmap == nullptr) {
    return false;
  }

  if (!state_->init()) {
    return false;
  }
  if (!grow_n(state_->costmap, state_->costmap_cap, costmap_bytes) ||
    !grow_n(state_->x, state_->x_cap, traj_floats) ||
    !grow_n(state_->y, state_->y_cap, traj_floats) ||
    !grow_n(state_->pose_costs, state_->pose_costs_cap, traj_floats))
  {
    return false;
  }

  cudaStream_t stream = state_->stream;
  if (cudaEventRecord(state_->ev_costmap_begin, stream) != cudaSuccess) {
    return false;
  }
  if (costmap_bytes > 0 &&
    cudaMemcpyAsync(
      state_->costmap, costmap, costmap_bytes, cudaMemcpyHostToDevice, stream) != cudaSuccess)
  {
    return false;
  }
  if (cudaEventRecord(state_->ev_traj_begin, stream) != cudaSuccess) {
    return false;
  }
  if (traj_floats > 0 &&
    (cudaMemcpyAsync(
      state_->x, x, traj_floats * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess ||
    cudaMemcpyAsync(
      state_->y, y, traj_floats * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess))
  {
    return false;
  }
  if (cudaEventRecord(state_->ev_kernel_begin, stream) != cudaSuccess) {
    return false;
  }

  if (sample_count > 0) {
    const int grid = (sample_count + kBlockSize - 1) / kBlockSize;
    gather_center_kernel<<<grid, kBlockSize, 0, stream>>>(
      state_->x, state_->y, state_->costmap, params, state_->pose_costs);
    if (cudaGetLastError() != cudaSuccess) {
      return false;
    }
  }
  if (cudaEventRecord(state_->ev_d2h_begin, stream) != cudaSuccess) {
    return false;
  }
  if (traj_floats > 0 &&
    cudaMemcpyAsync(
      pose_costs, state_->pose_costs, traj_floats * sizeof(float),
      cudaMemcpyDeviceToHost, stream) != cudaSuccess)
  {
    return false;
  }
  if (cudaEventRecord(state_->ev_d2h_end, stream) != cudaSuccess) {
    return fail_after_d2h(stream);
  }
  // The host pose-cost pointer is written by the copy above. Wait until that
  // write is visible before the caller reads it.
  if (cudaStreamSynchronize(stream) != cudaSuccess) {
    return fail_after_d2h(stream);
  }

  g_last_timings.costmap_h2d_ms = elapsed_ms(state_->ev_costmap_begin, state_->ev_traj_begin);
  g_last_timings.trajectory_h2d_ms = elapsed_ms(state_->ev_traj_begin, state_->ev_kernel_begin);
  g_last_timings.kernel_ms = elapsed_ms(state_->ev_kernel_begin, state_->ev_d2h_begin);
  g_last_timings.d2h_ms = elapsed_ms(state_->ev_d2h_begin, state_->ev_d2h_end);
  if (sample_count > 0) {
    ++g_launch_count;
  }
  return true;
}

bool obstacles_critic_cuda_compiled() {return true;}

bool obstacles_critic_cuda_device_available()
{
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess) {
    return false;
  }
  return count > 0;
}

int obstacles_critic_cuda_launch_count() {return g_launch_count;}

void reset_obstacles_critic_cuda_launch_count() {g_launch_count = 0;}

ObstaclesCriticCudaTimings obstacles_critic_cuda_last_timings() {return g_last_timings;}

}  // namespace mppi::critics
