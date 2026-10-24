// Package-private CUDA bridge contract prototype; host code must not include it.
#ifndef MPPI_R2_DEVICE_CONTRACT_PROTOTYPE_HPP_
#define MPPI_R2_DEVICE_CONTRACT_PROTOTYPE_HPP_
#include "interface-designer-api.hpp"
#include <cuda_runtime_api.h>
namespace mppi { namespace gpu { namespace native {
struct DeviceView {
  cudaStream_t stream = nullptr;
  const std::uint8_t * map = nullptr;
  const float * x = nullptr;
  const float * y = nullptr;
  float * result = nullptr;
  std::uint8_t * collisions = nullptr;
  std::uint32_t * all_collided = nullptr;
  std::uint32_t rows = 0, steps = 0, width = 0, height = 0;
  double origin_x = 0, origin_y = 0, resolution = 0;
};
class Lease final
{
public:
  Lease() noexcept;
  ~Lease() noexcept;
  Lease(const Lease &) = delete;
  Lease & operator=(const Lease &) = delete;
  Lease(Lease &&) noexcept;
  Lease & operator=(Lease &&) noexcept;
private:
  void * state_ = nullptr;
  friend Status acquire(Context &, const BatchToken &, const ResultSpec &,
    Lease &, DeviceView &) noexcept;
  friend Status complete(Lease &, std::int32_t, ResultToken &) noexcept;
  friend Status abandon(Lease &) noexcept;
};
Status acquire(Context &, const BatchToken &, const ResultSpec &, Lease &, DeviceView &) noexcept;
Status complete(Lease &, std::int32_t launch_status, ResultToken &) noexcept;
Status abandon(Lease &) noexcept;
}}}
#endif
