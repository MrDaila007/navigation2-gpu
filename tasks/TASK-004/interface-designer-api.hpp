// R2 declaration-only contract prototype, not a production header.
#ifndef MPPI_R2_CONTRACT_PROTOTYPE_HPP_
#define MPPI_R2_CONTRACT_PROTOTYPE_HPP_
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace mppi { namespace gpu {
inline constexpr std::uint32_t kContractVersion = 1;
namespace detail { struct Identity; struct Access; }
struct ControlTag {};
struct BatchTag {};
struct ResultTag {};
class Context;
template<class Tag> class Token
{
public:
  Token() noexcept = default;
  Token(const Token &) noexcept = default;
  Token & operator=(const Token &) noexcept = default;
private:
  std::weak_ptr<const detail::Identity> identity_;
  std::uint64_t control_ = 0, batch_ = 0, operation_ = 0;
  friend class Context;
  friend struct detail::Access;
};
using ControlToken = Token<ControlTag>;
using BatchToken = Token<BatchTag>;
using ResultToken = Token<ResultTag>;

enum class Code : std::uint16_t {
  Ok, NotCompiled, NoDevice, DriverUnavailable, CapacityExceeded,
  HostMemoryUnavailable, DeviceMemoryUnavailable, OperationFailedDrained,
  CpuForCycle, InvalidLimits, InvalidInput, UnsupportedShape,
  InvalidState, InvalidToken, OutstandingLease, ConcurrentUse,
  Closed, RestartRequired, DrainUnproven
};
enum class Stage : std::uint8_t {
  None, Configure, Validate, Snapshot, Setup, Reserve, H2D, Launch, D2H, Drain, Close
};
struct Status {
  Code code = Code::Ok;
  Stage stage = Stage::None;
  std::int32_t native_code = 0;
};
struct Limits {
  std::uint32_t max_rows = 100000;
  std::uint32_t max_steps = 1000;
  std::uint64_t max_map_cells = 16777216;
  std::uint64_t max_device_bytes = 268435456;
  std::uint64_t max_host_bytes = 268435456;
  std::int32_t device_ordinal = 0;
  bool measure_timings = true;
};
struct MapInput {
  const std::uint8_t * data = nullptr;
  std::size_t data_bytes = 0;
  std::uint32_t width = 0, height = 0;
  double origin_x = 0, origin_y = 0, resolution = 0;
};
struct BatchInput {
  const float * x = nullptr;
  const float * y = nullptr;
  std::size_t x_count = 0, y_count = 0;
  std::uint32_t rows = 0, steps = 0;
};
enum class ResultKind : std::uint8_t {
  CostScoresFloat, CostCentersFloat, ObstacleCentersDouble
};
struct ResultSpec {
  ResultKind kind = ResultKind::CostScoresFloat;
  std::uint32_t point_step = 1;
  bool track_collisions = false;
  double host_footprint_margin = 0;
};
struct Timings {
  float map_h2d_ms = 0, xy_h2d_ms = 0, kernel_ms = 0, result_d2h_ms = 0;
  bool valid = false;
};
struct HostResult {
  ResultKind kind = ResultKind::CostScoresFloat;
  const float * values = nullptr;
  std::size_t value_count = 0;
  const std::uint8_t * collisions = nullptr;
  std::size_t collision_count = 0;
  std::uint32_t rows = 0, columns = 0, all_collided = 0;
  Timings timings{};
};
struct Stats {
  std::uint64_t control_epochs = 0, batches = 0, operations = 0;
  std::uint64_t map_uploads = 0, xy_uploads = 0, map_bytes = 0, xy_bytes = 0;
  std::uint64_t result_copies = 0, result_bytes = 0;
  std::uint64_t queued_launches = 0, completed_launches = 0;
  std::uint64_t allocations = 0, host_allocations = 0, growths = 0;
  std::uint64_t host_snapshot_bytes = 0, sync_nanoseconds = 0;
  std::uint64_t device_payload_peak = 0, host_payload_peak = 0;
  std::uint64_t fallback_cycles = 0, opaque_barriers = 0, timing_failures = 0;
  Status last_status{};
};
class Context final
{
public:
  Context() noexcept;
  ~Context() noexcept;
  Context(const Context &) = delete;
  Context & operator=(const Context &) = delete;
  Context(Context &&) = delete;
  Context & operator=(Context &&) = delete;
  Status configure(const Limits &) noexcept;
  Status begin_control(const MapInput &, ControlToken &) noexcept;
  Status begin_batch(const ControlToken &, const BatchInput &, BatchToken &) noexcept;
  Status invalidate_inputs(const ControlToken &) noexcept;
  Status refresh_inputs(const ControlToken &, const MapInput &, const BatchInput &,
    BatchToken &) noexcept;
  Status read_result(const ResultToken &, HostResult &) const noexcept;
  Status release_result(const ResultToken &) noexcept;
  Status end_control(const ControlToken &) noexcept;
  Status reset() noexcept;
  Status close() noexcept;
  Status read_stats(Stats &) const noexcept;
private:
  struct Impl;
  Impl * impl_ = nullptr;
  mutable std::atomic_flag entry_ = ATOMIC_FLAG_INIT;
  friend struct detail::Access;
};
class ReadOnlyInputsV1
{
public:
  virtual ~ReadOnlyInputsV1() noexcept;
};
class CriticGpuV1 : public ReadOnlyInputsV1
{
public:
  ~CriticGpuV1() noexcept override;
  virtual std::uint32_t gpu_contract_version() const noexcept = 0;
  virtual bool gpu_requested() const noexcept = 0;
  virtual Status attach_gpu(Context &) noexcept = 0;
  virtual void detach_gpu() noexcept = 0;
};
static_assert(sizeof(float) == 4, "R2 device values require 32-bit float");
static_assert(std::is_trivially_copyable<MapInput>::value, "POD input");
static_assert(std::is_trivially_copyable<BatchInput>::value, "POD input");
static_assert(std::is_trivially_copyable<Status>::value, "POD status");
static_assert(!std::is_copy_constructible<Context>::value, "single owner");
static_assert(!std::is_same<ControlToken, BatchToken>::value, "typed epochs");
}}  // namespace mppi::gpu
#endif
