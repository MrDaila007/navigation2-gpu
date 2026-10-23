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

#include "nav2_mppi_controller/critics/cost_critic_cuda.hpp"

namespace mppi::critics
{
namespace
{

constexpr int kBlockSize = 128;
constexpr unsigned char kLethal = 254;
constexpr unsigned char kInscribed = 253;
constexpr unsigned char kNoInformation = 255;

int g_launch_count = 0;
CostCriticCudaTimings g_last_timings{0.0f, 0.0f, 0.0f, 0.0f};

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

__device__ bool center_in_collision(float pose_cost, bool is_tracking_unknown)
{
  const unsigned char score_cost = static_cast<unsigned char>(pose_cost);
  if (score_cost == kLethal || score_cost == kInscribed) {
    return true;
  }
  if (score_cost == kNoInformation) {
    return !is_tracking_unknown;
  }
  return false;
}

__global__ void score_center_kernel(
  const float * __restrict__ x,
  const float * __restrict__ y,
  const unsigned char * __restrict__ costmap,
  CostCriticDeviceScore params,
  float * __restrict__ repulsive,
  unsigned char * __restrict__ collided,
  unsigned int * __restrict__ all_collided)
{
  const int i = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (i >= params.rows) {
    return;
  }

  const int strided_cols = ((params.time_cols - 1) / params.point_step) + 1;
  float traj_cost = 0.0f;
  bool trajectory_collide = false;

  for (int j = 0; j < strided_cols; ++j) {
    const int time = j * params.point_step;
    const int addr = time * params.rows + i;
    const float tx = x[addr];
    const float ty = y[addr];

    float pose_cost;
    if (tx < params.origin_x || ty < params.origin_y) {
      pose_cost = 255.0f;
    } else {
      const unsigned int mx =
        static_cast<unsigned int>((tx - params.origin_x) / params.resolution);
      const unsigned int my =
        static_cast<unsigned int>((ty - params.origin_y) / params.resolution);
      if (mx < params.size_x && my < params.size_y) {
        pose_cost = static_cast<float>(costmap[static_cast<size_t>(my) * params.size_x + mx]);
        if (pose_cost < 1.0f) {
          continue;
        }
      } else {
        pose_cost = 255.0f;
      }
    }

    if (center_in_collision(pose_cost, params.is_tracking_unknown)) {
      traj_cost = params.collision_cost;
      trajectory_collide = true;
      break;
    }

    if (pose_cost >= static_cast<float>(params.near_collision_cost)) {
      traj_cost += params.critical_cost;
    } else if (!params.near_goal) {
      traj_cost += pose_cost;
    }
  }

  repulsive[i] = traj_cost;
  collided[i] = trajectory_collide ? 1u : 0u;
  if (!trajectory_collide) {
    atomicAnd(all_collided, 0u);
  }
}

// One thread per strided sample. The value is the center-cell cost only:
// worldToMapFloat / getIndex, or 255 outside the map. No collision or footprint rule.
__global__ void gather_center_kernel(
  const float * __restrict__ x,
  const float * __restrict__ y,
  const unsigned char * __restrict__ costmap,
  CostCriticDeviceScore params,
  float * __restrict__ pose_costs)
{
  const int strided_cols = ((params.time_cols - 1) / params.point_step) + 1;
  const int sample_count = params.rows * strided_cols;
  const int idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= sample_count) {
    return;
  }

  const int i = idx % params.rows;
  const int j = idx / params.rows;
  const int time = j * params.point_step;
  const int addr = time * params.rows + i;
  const float tx = x[addr];
  const float ty = y[addr];

  float pose_cost;
  if (tx < params.origin_x || ty < params.origin_y) {
    pose_cost = 255.0f;
  } else {
    const unsigned int mx =
      static_cast<unsigned int>((tx - params.origin_x) / params.resolution);
    const unsigned int my =
      static_cast<unsigned int>((ty - params.origin_y) / params.resolution);
    if (mx < params.size_x && my < params.size_y) {
      pose_cost = static_cast<float>(costmap[static_cast<size_t>(my) * params.size_x + mx]);
    } else {
      pose_cost = 255.0f;
    }
  }

  pose_costs[static_cast<size_t>(j) * static_cast<size_t>(params.rows) + static_cast<size_t>(i)] =
    pose_cost;
}

}  // namespace

struct CostCriticCuda::DeviceState
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
  float * repulsive{nullptr};
  size_t repulsive_cap{0};
  float * pose_costs{nullptr};
  size_t pose_costs_cap{0};
  unsigned char * collided{nullptr};
  size_t collided_cap{0};
  unsigned int * all_collided{nullptr};

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
    if (all_collided == nullptr &&
      cudaMalloc(reinterpret_cast<void **>(&all_collided), sizeof(unsigned int)) != cudaSuccess)
    {
      all_collided = nullptr;
      return false;
    }
    ready = true;
    return true;
  }

  void release()
  {
    if (costmap != nullptr) {cudaFree(costmap);}
    if (x != nullptr) {cudaFree(x);}
    if (y != nullptr) {cudaFree(y);}
    if (repulsive != nullptr) {cudaFree(repulsive);}
    if (pose_costs != nullptr) {cudaFree(pose_costs);}
    if (collided != nullptr) {cudaFree(collided);}
    if (all_collided != nullptr) {cudaFree(all_collided);}
    cudaEvent_t events[] = {
      ev_costmap_begin, ev_traj_begin, ev_kernel_begin, ev_d2h_begin, ev_d2h_end};
    for (cudaEvent_t event : events) {
      if (event != nullptr) {cudaEventDestroy(event);}
    }
    if (stream != nullptr) {cudaStreamDestroy(stream);}
    costmap = nullptr;
    x = nullptr;
    y = nullptr;
    repulsive = nullptr;
    pose_costs = nullptr;
    collided = nullptr;
    all_collided = nullptr;
    ev_costmap_begin = nullptr;
    ev_traj_begin = nullptr;
    ev_kernel_begin = nullptr;
    ev_d2h_begin = nullptr;
    ev_d2h_end = nullptr;
    stream = nullptr;
    ready = false;
  }
};

CostCriticCuda::CostCriticCuda()
: state_(new DeviceState)
{
}

CostCriticCuda::~CostCriticCuda()
{
  if (state_ == nullptr) {
    return;
  }
  state_->release();
  delete state_;
  state_ = nullptr;
}

bool CostCriticCuda::score_center(
  const float * x, const float * y, const unsigned char * costmap,
  const CostCriticDeviceScore & params,
  float * repulsive, unsigned char * collided, bool * all_collided)
{
  if (all_collided == nullptr || state_ == nullptr) {
    return false;
  }
  if (params.rows < 0 || params.time_cols < 1 || params.point_step < 1 ||
    !(params.resolution > 0.0f))
  {
    return false;
  }
  if (params.rows == 0) {
    *all_collided = true;
    return true;
  }
  if (x == nullptr || y == nullptr || repulsive == nullptr || collided == nullptr) {
    return false;
  }

  const size_t costmap_bytes =
    static_cast<size_t>(params.size_x) * static_cast<size_t>(params.size_y);
  if (costmap_bytes > 0 && costmap == nullptr) {
    return false;
  }
  const size_t traj_floats =
    static_cast<size_t>(params.rows) * static_cast<size_t>(params.time_cols);
  const size_t rows = static_cast<size_t>(params.rows);

  if (!state_->init()) {
    return false;
  }
  if (!grow_n(state_->costmap, state_->costmap_cap, costmap_bytes) ||
    !grow_n(state_->x, state_->x_cap, traj_floats) ||
    !grow_n(state_->y, state_->y_cap, traj_floats) ||
    !grow_n(state_->repulsive, state_->repulsive_cap, rows) ||
    !grow_n(state_->collided, state_->collided_cap, rows))
  {
    return false;
  }

  cudaStream_t stream = state_->stream;
  const unsigned int none_missing = 1u;
  if (cudaMemcpyAsync(
      state_->all_collided, &none_missing, sizeof(none_missing),
      cudaMemcpyHostToDevice, stream) != cudaSuccess)
  {
    return false;
  }
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
  if (cudaMemcpyAsync(
      state_->x, x, traj_floats * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess ||
    cudaMemcpyAsync(
      state_->y, y, traj_floats * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess)
  {
    return false;
  }
  if (cudaEventRecord(state_->ev_kernel_begin, stream) != cudaSuccess) {
    return false;
  }

  const int grid = (params.rows + kBlockSize - 1) / kBlockSize;
  score_center_kernel<<<grid, kBlockSize, 0, stream>>>(
    state_->x, state_->y, state_->costmap, params,
    state_->repulsive, state_->collided, state_->all_collided);
  if (cudaGetLastError() != cudaSuccess) {
    return false;
  }
  if (cudaEventRecord(state_->ev_d2h_begin, stream) != cudaSuccess) {
    return false;
  }
  if (cudaMemcpyAsync(
      repulsive, state_->repulsive, rows * sizeof(float),
      cudaMemcpyDeviceToHost, stream) != cudaSuccess)
  {
    return false;
  }
  if (cudaMemcpyAsync(
      collided, state_->collided, rows * sizeof(unsigned char),
      cudaMemcpyDeviceToHost, stream) != cudaSuccess)
  {
    return fail_after_d2h(stream);
  }
  unsigned int all_flag = 0u;
  if (cudaMemcpyAsync(
      &all_flag, state_->all_collided, sizeof(all_flag),
      cudaMemcpyDeviceToHost, stream) != cudaSuccess)
  {
    return fail_after_d2h(stream);
  }
  if (cudaEventRecord(state_->ev_d2h_end, stream) != cudaSuccess) {
    return fail_after_d2h(stream);
  }
  // The host repulsive pointer is written by the copy above. Wait until that
  // write is visible before the caller reads it.
  if (cudaStreamSynchronize(stream) != cudaSuccess) {
    return fail_after_d2h(stream);
  }

  g_last_timings.costmap_h2d_ms = elapsed_ms(state_->ev_costmap_begin, state_->ev_traj_begin);
  g_last_timings.trajectory_h2d_ms = elapsed_ms(state_->ev_traj_begin, state_->ev_kernel_begin);
  g_last_timings.kernel_ms = elapsed_ms(state_->ev_kernel_begin, state_->ev_d2h_begin);
  g_last_timings.d2h_ms = elapsed_ms(state_->ev_d2h_begin, state_->ev_d2h_end);
  *all_collided = all_flag != 0u;
  ++g_launch_count;
  return true;
}

bool CostCriticCuda::gather_center(
  const float * x, const float * y, const unsigned char * costmap,
  const CostCriticDeviceScore & params, float * pose_costs)
{
  if (state_ == nullptr) {
    return false;
  }
  if (params.rows < 0 || params.time_cols < 1 || params.point_step < 1 ||
    !(params.resolution > 0.0f))
  {
    return false;
  }
  if (params.rows == 0) {
    return true;
  }
  if (x == nullptr || y == nullptr || pose_costs == nullptr) {
    return false;
  }

  const size_t costmap_bytes =
    static_cast<size_t>(params.size_x) * static_cast<size_t>(params.size_y);
  if (costmap_bytes > 0 && costmap == nullptr) {
    return false;
  }
  const size_t traj_floats =
    static_cast<size_t>(params.rows) * static_cast<size_t>(params.time_cols);
  const int strided_cols = ((params.time_cols - 1) / params.point_step) + 1;
  const size_t pose_n =
    static_cast<size_t>(params.rows) * static_cast<size_t>(strided_cols);

  if (!state_->init()) {
    return false;
  }
  if (!grow_n(state_->costmap, state_->costmap_cap, costmap_bytes) ||
    !grow_n(state_->x, state_->x_cap, traj_floats) ||
    !grow_n(state_->y, state_->y_cap, traj_floats) ||
    !grow_n(state_->pose_costs, state_->pose_costs_cap, pose_n))
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
  if (cudaMemcpyAsync(
      state_->x, x, traj_floats * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess ||
    cudaMemcpyAsync(
      state_->y, y, traj_floats * sizeof(float), cudaMemcpyHostToDevice, stream) != cudaSuccess)
  {
    return false;
  }
  if (cudaEventRecord(state_->ev_kernel_begin, stream) != cudaSuccess) {
    return false;
  }

  const int grid = (static_cast<int>(pose_n) + kBlockSize - 1) / kBlockSize;
  gather_center_kernel<<<grid, kBlockSize, 0, stream>>>(
    state_->x, state_->y, state_->costmap, params, state_->pose_costs);
  if (cudaGetLastError() != cudaSuccess) {
    return false;
  }
  if (cudaEventRecord(state_->ev_d2h_begin, stream) != cudaSuccess) {
    return false;
  }
  if (cudaMemcpyAsync(
      pose_costs, state_->pose_costs, pose_n * sizeof(float),
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
  ++g_launch_count;
  return true;
}

bool cost_critic_cuda_compiled() {return true;}

bool cost_critic_cuda_device_available()
{
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess) {
    return false;
  }
  return count > 0;
}

int cost_critic_cuda_launch_count() {return g_launch_count;}

void reset_cost_critic_cuda_launch_count() {g_launch_count = 0;}

CostCriticCudaTimings cost_critic_cuda_last_timings() {return g_last_timings;}

}  // namespace mppi::critics
