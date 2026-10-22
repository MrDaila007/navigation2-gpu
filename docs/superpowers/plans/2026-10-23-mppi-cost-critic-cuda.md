# MPPI CostCritic CUDA Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an optional CUDA path for `mppi::critics::CostCritic::score` that writes the same costs, failure flag, and collision flags as the classical CPU critic.

**Architecture:** `CostCritic::score` keeps the classical loop and the host weight/power update. When `use_cuda` is true and a device is available, a `CostCriticCuda` helper copies the costmap and the trajectory samples, runs one thread per trajectory for center-cell scoring, or gathers center costs for the host footprint scan. The CPU path stays the default. Parity tests score the same inputs both ways and require bitwise equality.

**Tech Stack:** C++17/20, Eigen, ROS 2 Humble, ament/colcon, CUDA Toolkit 11.5 on this machine (floor 11.4), GoogleTest.

**Spec:** `docs/superpowers/specs/2026-10-22-mppi-cost-critic-cuda-design.md`

## Global Constraints

- Parameter `use_cuda`, bool, default `false`, read in `CostCritic::initialize`.
- CMake option `NAV2_MPPI_CUDA`, default `OFF`. `OFF` does not find the CUDA toolkit and does not compile `.cu` files.
- CUDA toolkit floor is 11.4. This machine is CUDA 11.5, RTX 3060, compute capability 8.6.
- Empty `CMAKE_CUDA_ARCHITECTURES` is filled from `nvidia-smi --query-gpu=compute_cap` (`8.6` becomes `86`). Explicit `87` remains the Jetson Orin value. No cross-compile.
- Host keeps the weight and power update. `pow` is Eigen's host `pow`.
- The costmap `unsigned char` buffer is copied at every device `score()`.
- Match rule is bitwise equality of `data.costs`, `data.fail_flag`, and `data.trajectories_in_collision`.
- `FootprintCollisionChecker::footprintCostAtPose` stays on the host.
- Plugin class stays `mppi::critics::CostCritic`. No new `critics.xml` entry.
- No `package.xml` CUDA dependency.
- Timing record has no speed threshold.
- Development acceptance runs on this machine's RTX 3060.

## File Structure

- Modify `nav2_mppi_controller/CMakeLists.txt`: option, toolkit, which source compiles into `mppi_critics`.
- Modify `nav2_mppi_controller/include/nav2_mppi_controller/critics/cost_critic.hpp`: `use_cuda_`, out-of-line destructor, `std::unique_ptr<CostCriticCuda>`.
- Modify `nav2_mppi_controller/src/critics/cost_critic.cpp`: parameter, shared host accumulate and weight update, device branch.
- Create `nav2_mppi_controller/include/nav2_mppi_controller/critics/cost_critic_cuda.hpp`: host-only API. No CUDA headers.
- Create `nav2_mppi_controller/src/critics/cost_critic_cuda_stub.cpp`: API used when `NAV2_MPPI_CUDA` is `OFF`, and the failing scaffold while the kernel is still empty.
- Create `nav2_mppi_controller/src/critics/cost_critic_cuda.cu`: buffers, copies, kernels. Replaces the stub when the option is `ON`.
- Create `nav2_mppi_controller/test/cost_critic_parity_test.cpp`: classical formula tests and CUDA parity tests.
- Modify `nav2_mppi_controller/test/CMakeLists.txt`: register the parity test.
- Create `nav2_mppi_controller/benchmark/cost_critic_cuda_benchmark.cpp`: transfer and kernel times. Built only when the option is `ON`.
- Modify `nav2_mppi_controller/benchmark/CMakeLists.txt`: register that benchmark only with the option.

`cost_critic.hpp` may include `cost_critic_cuda.hpp`. It must not include `cuda_runtime.h`.

## Review Focus

- A lethal sample ends that trajectory: later samples add nothing, and the repulsive cost is `collision_cost` exactly. Test: `EarlyExitLethal` in Task 1, CUDA twin in Task 3.
- A cell of 255 collides when unknown space is not tracked, including points below the origin and past the map size. When unknown space is tracked, 255 does not collide and adds `critical_cost` because `255 >= near_collision_cost`. Tests: `UnknownUntracked`, `UnknownTracked`, `OutOfMapLow`, `OutOfMapHigh` in Task 1, CUDA twins in Task 3.
- Near the goal, ordinary cell costs add nothing, a cell at or above `near_collision_cost` still adds `critical_cost`, and a lethal cell still collides. Tests: `NearGoalDrop`, `NearGoalCritical`, `NearGoalLethal` in Task 1, CUDA twins in Task 3.
- With `consider_footprint` true, a free center (`cost < 1`) never asks the footprint checker; a center of 200 whose footprint hits 254 becomes `collision_cost`; a clear footprint keeps the center cost; a center of 253 with a clear footprint adds `critical_cost` and does not collide. Tests in Task 4.
- `trajectory_point_step == 2` reads time columns 0, 2, 4. A lethal cell on column 1 or 3 adds nothing. Test: `StrideSkipsLethal` in Task 1, CUDA twin in Task 3.

---

### Task 1: Classical formula tests

**Files:**
- Create: `nav2_mppi_controller/test/cost_critic_parity_test.cpp`
- Modify: `nav2_mppi_controller/test/CMakeLists.txt`

**Interfaces:**
- Consumes: `CostCritic::on_configure`, `CostCritic::score`, `CriticData`, `models::Trajectories::reset`.
- Produces: `expect_classical(const ClassicalCase &)` in the test file. Later tasks add `expect_cuda_matches_classical` beside it. `ClassicalCase` fields are exactly the ones in Step 1.

- [ ] **Step 1: Write the classical tests**

Copy the `RosLockGuard` from `nav2_mppi_controller/benchmark/optimizer_benchmark.cpp` into the new test file. Do not include `utils_test.cpp`.

World coordinates that must land in cell `(mx, my)` on the default map (origin `0`, resolution `0.1`):

```cpp
float world_from_cell(unsigned int cell)
{
  return (static_cast<float>(cell) + 0.5f) * 0.1f;
}
```

`ClassicalCase` holds: `track_unknown`, `consider_footprint` (false for every case in this task), `power`, `point_step`, `time_steps`, `batch`, `local_path_length`, `near_collision_cost`, `cost_weight` (`254.0f`), `collision_cost` (`1000000.0f`), `critical_cost` (`300.0f`), `near_goal_distance` (`0.5f`), `enabled` (`true`), `track_collisions` (`true`), per-trajectory per-time cell coordinates or an explicit out-of-map `(x, y)`, painted `(mx, my, cost)` cells, per-trajectory `repulsive`, `fail`, and per-trajectory `collided`.

`expect_classical` builds a fresh `Costmap2DROS`, declares `track_unknown_space` before `on_configure`, zeroes the master grid with `setCost`, paints the listed cells, and configures one `CostCritic` named `classical`. Before `on_configure`, declare the critic parameters through `ParametersHandler::getParamGetter("classical")` using the same C++ types as `CostCritic::initialize`: bool, int `cost_power`, float `cost_weight`, float `critical_cost`, int `near_collision_cost`, float `collision_cost`, float `near_goal_distance`, int `trajectory_point_step`, bool `enabled`, bool `use_cuda` false. Place each in-map sample at `world_from_cell`. Score once. Expected added cost for trajectory `i` is computed in the test, not read back from the critic:

```cpp
const int strided_cols =
  static_cast<int>(std::floor((time_steps - 1) / static_cast<double>(point_step))) + 1;
const float scale = (254.0f / 254.0f) / static_cast<float>(strided_cols);
Eigen::ArrayXf expected(batch);
expected = repulsive[i] * scale;
if (power > 1) {
  expected = expected.pow(power);
}
EXPECT_EQ(costs(i), expected(i));
EXPECT_EQ(data.fail_flag, fail);
```

When `track_collisions` is true, `trajectories_in_collision` starts as `batch` entries of `false`, and each entry equals `collided[i]` after `score`. When it starts empty, it is still empty after `score`.

Each row has a factory `ClassicalCase <TestName>_case()`. Tests call `expect_classical(<TestName>_case())`. `DisabledCritic` does not use the repulsive formula: after `score`, `costs` is still all zeros, `fail_flag` is false, and collision flags are still false. `EmptyCollisionVector_case` sets `track_collisions` false.

| Test | batch | steps | step | power | track_unknown | path length | near_collision | painted cells `(mx,my,cost)` | sample cells per time | repulsive | collided | fail |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `FreeSpace` | 1 | 3 | 1 | 1 | false | 10 | 253 | none, grid 0 | `(5,5)` three times | 0 | false | false |
| `SumInflated` | 1 | 3 | 1 | 1 | false | 10 | 253 | `(10,10,10) (11,10,20) (12,10,30)` | those cells | 60 | false | false |
| `EarlyExitLethal` | 1 | 3 | 1 | 1 | false | 10 | 253 | `(10,10,254) (11,10,50) (12,10,50)` | those cells | 1000000 | true | true |
| `InscribedCollides` | 1 | 1 | 1 | 1 | false | 10 | 253 | `(4,4,253)` | `(4,4)` | 1000000 | true | true |
| `Cost252AddsItself` | 1 | 1 | 1 | 1 | false | 10 | 253 | `(4,4,252)` | `(4,4)` | 252 | false | false |
| `NearGoalDrop` | 1 | 3 | 1 | 1 | false | 0.1 | 253 | `(10,10,10) (11,10,20) (12,10,30)` | those cells | 0 | false | false |
| `NearGoalCritical` | 1 | 3 | 1 | 1 | false | 0.1 | 100 | `(10,10,10) (11,10,150) (12,10,30)` | those cells | 300 | false | false |
| `NearGoalLethal` | 1 | 3 | 1 | 1 | false | 0.1 | 253 | `(10,10,254) (11,10,10) (12,10,10)` | those cells | 1000000 | true | true |
| `UnknownTracked` | 1 | 1 | 1 | 1 | true | 10 | 253 | `(6,6,255)` | `(6,6)` | 300 | false | false |
| `UnknownUntracked` | 1 | 1 | 1 | 1 | false | 10 | 253 | `(6,6,255)` | `(6,6)` | 1000000 | true | true |
| `OutOfMapLow` | 1 | 1 | 1 | 1 | false | 10 | 253 | none | world `(-1, 1)` | 1000000 | true | true |
| `OutOfMapHigh` | 1 | 1 | 1 | 1 | false | 10 | 253 | none | world `(100, 1)` | 1000000 | true | true |
| `StrideSkipsLethal` | 1 | 5 | 2 | 1 | false | 10 | 253 | cells `(10+t, 10, cost)` for costs `10,254,20,254,30` | those five | 60 | false | false |
| `PowerTwo` | 1 | 1 | 1 | 2 | false | 10 | 253 | `(7,7,10)` | `(7,7)` | 10 | false | false |
| `MixedFailFlag` | 2 | 1 | 1 | 1 | false | 10 | 253 | traj0 `(8,8,254)`, traj1 `(9,9,0)` | those | `1000000, 0` | `true, false` | false |
| `AllCollide` | 2 | 1 | 1 | 1 | false | 10 | 253 | traj0 `(8,8,254)`, traj1 `(9,9,254)` | those | `1000000, 1000000` | `true, true` | true |
| `EmptyCollisionVector` | 1 | 3 | 1 | 1 | false | 10 | 253 | same as `SumInflated` | same | 60 | vector stays empty | false |
| `DisabledCritic` | 1 | 1 | 1 | 1 | false | 10 | 253 | `(4,4,254)` | `(4,4)` | costs stay 0 | flags stay false | false |

`DisabledCritic` sets `enabled` false. `EmptyCollisionVector` passes an empty `trajectories_in_collision`. Yaw is `0` for every sample.

Register the binary in `test/CMakeLists.txt` the same way as `critics_tests`, linking `mppi_controller` and `mppi_critics`.

- [ ] **Step 2: Run the classical tests**

```bash
source /opt/ros/humble/setup.bash
cd /home/user/Documents/Robots/navigation2-gpu
colcon build --packages-up-to nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF
source install/setup.bash
./build/nav2_mppi_controller/cost_critic_parity_test
```

Expected: every `TEST` above passes. Later runs in this plan use `--packages-select nav2_mppi_controller` once `install/` exists.

- [ ] **Step 3: Commit**

```bash
git add nav2_mppi_controller/test/cost_critic_parity_test.cpp nav2_mppi_controller/test/CMakeLists.txt
git commit -m "$(cat <<'EOF'
test: lock CostCritic classical scoring

EOF
)"
```

---

### Task 2: `use_cuda` flag and CPU fallback

**Files:**
- Create: `nav2_mppi_controller/include/nav2_mppi_controller/critics/cost_critic_cuda.hpp`
- Create: `nav2_mppi_controller/src/critics/cost_critic_cuda_stub.cpp`
- Modify: `nav2_mppi_controller/include/nav2_mppi_controller/critics/cost_critic.hpp`
- Modify: `nav2_mppi_controller/src/critics/cost_critic.cpp`
- Modify: `nav2_mppi_controller/CMakeLists.txt`
- Modify: `nav2_mppi_controller/test/cost_critic_parity_test.cpp`

**Interfaces:**
- Consumes: `ClassicalCase` and `expect_classical` from Task 1.
- Produces:

```cpp
namespace mppi::critics {

struct CostCriticDeviceScore {
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

struct CostCriticCudaTimings {
  float costmap_h2d_ms;
  float trajectory_h2d_ms;
  float kernel_ms;
  float d2h_ms;
};

class CostCriticCuda {
public:
  CostCriticCuda();
  ~CostCriticCuda();
  CostCriticCuda(const CostCriticCuda &) = delete;
  CostCriticCuda & operator=(const CostCriticCuda &) = delete;

  bool score_center(
    const float * x, const float * y,
    const unsigned char * costmap,
    const CostCriticDeviceScore & params,
    float * repulsive, unsigned char * collided, bool * all_collided);

  bool gather_center(
    const float * x, const float * y,
    const unsigned char * costmap,
    const CostCriticDeviceScore & params,
    float * pose_costs);
};

bool cost_critic_cuda_compiled();
bool cost_critic_cuda_device_available();
int cost_critic_cuda_launch_count();
void reset_cost_critic_cuda_launch_count();
CostCriticCudaTimings cost_critic_cuda_last_timings();

}  // namespace mppi::critics
```

`score_center` and `gather_center` return false when the device did not produce a buffer the caller may read. `x` and `y` are column-major Eigen buffers: sample `(row, time)` is at `time * rows + row`.

- [ ] **Step 1: Write the failing fallback tests**

Add to `cost_critic_parity_test.cpp`:

```cpp
TEST(CostCriticParity, CudaFlagOffDoesNotLaunch)
{
  reset_cost_critic_cuda_launch_count();
  const int before = cost_critic_cuda_launch_count();
  // Score SumInflated through a critic whose use_cuda parameter is false.
  expect_classical(SumInflated_case());
  EXPECT_FALSE(cost_critic_cuda_compiled());
  EXPECT_EQ(cost_critic_cuda_launch_count(), before);
}

TEST(CostCriticParity, CudaFlagOnMatchesCpuWhenDeviceMissing)
{
  if (cost_critic_cuda_device_available()) {
    GTEST_SKIP();
  }
  reset_cost_critic_cuda_launch_count();
  const int before = cost_critic_cuda_launch_count();
  Eigen::ArrayXf cpu_costs;
  Eigen::ArrayXf cuda_costs;
  bool cpu_fail = false;
  bool cuda_fail = false;
  std::vector<bool> cpu_hits;
  std::vector<bool> cuda_hits;
  score_case(SumInflated_case(), "cpu_critic", false, cpu_costs, cpu_fail, cpu_hits);
  score_case(SumInflated_case(), "cuda_critic", true, cuda_costs, cuda_fail, cuda_hits);
  ASSERT_EQ(cpu_costs.size(), cuda_costs.size());
  for (int i = 0; i < cpu_costs.size(); ++i) {
    EXPECT_EQ(cpu_costs(i), cuda_costs(i));
  }
  EXPECT_EQ(cpu_fail, cuda_fail);
  EXPECT_EQ(cpu_hits, cuda_hits);
  EXPECT_EQ(cost_critic_cuda_launch_count(), before);
}
```

`score_case` is the body of `expect_classical` with an explicit critic name and `use_cuda` value, returning the outputs instead of asserting the formula. `expect_classical` calls it with `use_cuda` false and then checks the formula.

- [ ] **Step 2: Run the tests and confirm they fail to compile**

Run: `./build/nav2_mppi_controller/cost_critic_parity_test` after rebuilding `nav2_mppi_controller` with `-DNAV2_MPPI_CUDA=OFF`.

Expected: compile error, `cost_critic_cuda_compiled` is not declared.

- [ ] **Step 3: Implement the flag and the stub**

Add `option(NAV2_MPPI_CUDA "Build MPPI CostCritic CUDA scoring" OFF)` to `nav2_mppi_controller/CMakeLists.txt` before `add_library(mppi_critics ...)`. Leave the option off in this task. Add `src/critics/cost_critic_cuda_stub.cpp` to `mppi_critics`.

Stub, compiled without CUDA headers:

- `cost_critic_cuda_compiled()` returns false.
- `cost_critic_cuda_device_available()` returns false.
- launch count stays 0. `reset_cost_critic_cuda_launch_count()` sets it to 0.
- `score_center` and `gather_center` return false.
- `cost_critic_cuda_last_timings()` returns zeros.

In `CostCritic::initialize`, read `use_cuda_` with default `false`. Hold `std::unique_ptr<CostCriticCuda> cuda_scorer_` and declare `~CostCritic() override` out of line in the `.cpp` so the unique pointer can be destroyed there. Create the scorer on the first `score()` that sees `use_cuda_` true.

In `score()`, when `use_cuda_` is true and `cost_critic_cuda_device_available()` is false, log one warning per process with a function-local static flag and run the existing loop. When `use_cuda_` is false, run the existing loop and do not construct a CUDA call.

- [ ] **Step 4: Run the tests**

Rebuild with `-DNAV2_MPPI_CUDA=OFF` and run `./build/nav2_mppi_controller/cost_critic_parity_test`.

Expected: Task 1 tests still pass. `CudaFlagOffDoesNotLaunch` passes. `CudaFlagOnMatchesCpuWhenDeviceMissing` passes on this stub because no device is available through the stub.

- [ ] **Step 5: Commit**

```bash
git add nav2_mppi_controller/CMakeLists.txt \
  nav2_mppi_controller/include/nav2_mppi_controller/critics/cost_critic.hpp \
  nav2_mppi_controller/include/nav2_mppi_controller/critics/cost_critic_cuda.hpp \
  nav2_mppi_controller/src/critics/cost_critic.cpp \
  nav2_mppi_controller/src/critics/cost_critic_cuda_stub.cpp \
  nav2_mppi_controller/test/cost_critic_parity_test.cpp
git commit -m "$(cat <<'EOF'
feat: keep CostCritic on CPU unless CUDA is available

EOF
)"
```

---

### Task 3: Device center scoring matches the classical critic

**Files:**
- Modify: `nav2_mppi_controller/CMakeLists.txt`
- Modify: `nav2_mppi_controller/src/critics/cost_critic.cpp`
- Modify: `nav2_mppi_controller/src/critics/cost_critic_cuda_stub.cpp`
- Create: `nav2_mppi_controller/src/critics/cost_critic_cuda.cu`
- Modify: `nav2_mppi_controller/test/cost_critic_parity_test.cpp`

**Interfaces:**
- Consumes: `CostCriticCuda::score_center` and `CostCriticDeviceScore` from Task 2.
- Produces: `CostCritic::applyWeightAndPower(CriticData &, const Eigen::ArrayXf &, int, bool)` and `CostCritic::accumulateFromPoseCosts(CriticData &, const Eigen::ArrayXXf *, Eigen::ArrayXf &, bool &)` in `cost_critic.cpp`. The device path calls only `applyWeightAndPower`.

- [ ] **Step 1: Write the failing CUDA parity tests**

Add `expect_cuda_matches_classical(const ClassicalCase &)`. It scores with `use_cuda` false and with `use_cuda` true into separate critics (`classical` and `cuda_classical`). Require:

```cpp
ASSERT_TRUE(cost_critic_cuda_compiled());
ASSERT_TRUE(cost_critic_cuda_device_available());
const int before = cost_critic_cuda_launch_count();
// score both critics
for (int i = 0; i < cpu_costs.size(); ++i) {
  EXPECT_EQ(cpu_costs(i), cuda_costs(i)) << case_name << " trajectory " << i;
}
EXPECT_EQ(cpu_fail, cuda_fail);
EXPECT_EQ(cpu_hits, cuda_hits);
if (enabled) {
  EXPECT_GT(cost_critic_cuda_launch_count(), before);
} else {
  EXPECT_EQ(cost_critic_cuda_launch_count(), before);
}
```

Guard the tests with `#if NAV2_MPPI_HAS_CUDA`. If `cost_critic_cuda_device_available()` is false inside that build, `GTEST_SKIP()`.

Call it for every case in the Task 1 table: `CudaFreeSpace`, `CudaSumInflated`, `CudaEarlyExitLethal`, `CudaInscribedCollides`, `CudaCost252AddsItself`, `CudaNearGoalDrop`, `CudaNearGoalCritical`, `CudaNearGoalLethal`, `CudaUnknownTracked`, `CudaUnknownUntracked`, `CudaOutOfMapLow`, `CudaOutOfMapHigh`, `CudaStrideSkipsLethal`, `CudaPowerTwo`, `CudaMixedFailFlag`, `CudaAllCollide`, `CudaEmptyCollisionVector`, `CudaDisabledCritic`.

- [ ] **Step 2: Turn the option on and confirm the new tests fail**

In `CMakeLists.txt`, when `NAV2_MPPI_CUDA` is `ON`:

- If `CMAKE_CUDA_ARCHITECTURES` is empty, run `nvidia-smi --query-gpu=compute_cap --format=csv,noheader`, strip the dot, and set `CMAKE_CUDA_ARCHITECTURES` to that string before `enable_language(CUDA)`.
- `enable_language(CUDA)`, `find_package(CUDAToolkit REQUIRED)`, `set(CMAKE_CUDA_STANDARD 17)`.
- Define `NAV2_MPPI_HAS_CUDA=1` on `mppi_critics` and on `cost_critic_parity_test`.
- Link `CUDA::cudart`.
- Keep compiling the stub in this step. Teach the stub, only under `NAV2_MPPI_HAS_CUDA`, to return `compiled() == true` and `device_available()` from `cudaGetDeviceCount`. `score_center` still returns false and does not increment the launch count.

Rebuild with `-DNAV2_MPPI_CUDA=ON` and run `./build/nav2_mppi_controller/cost_critic_parity_test --gtest_filter=CostCriticParity.CudaSumInflated`.

Expected: FAIL because the launch count does not increase. `CudaFlagOffDoesNotLaunch` now sees `cost_critic_cuda_compiled() == true`, so change its `EXPECT_FALSE(cost_critic_cuda_compiled())` to `EXPECT_EQ(cost_critic_cuda_launch_count(), before)` only. The compiled-false assertion belongs to the `OFF` build; keep it under `#if !NAV2_MPPI_HAS_CUDA`.

- [ ] **Step 3: Implement center scoring**

Replace the stub with `cost_critic_cuda.cu` in the `ON` branch. The `OFF` branch keeps the stub.

Move the existing repulsive loop into `accumulateFromPoseCosts`. A null gathered pointer keeps today's `worldToMapFloat` / `getCost` / `pose_cost < 1` continue. Then call `applyWeightAndPower`, whose body is the current lines that scale `repulsive` by `weight_ / strided_cols`, apply Eigen `pow` when `power_ > 1`, add into `data.costs`, and assign `data.fail_flag`.

Device branch, only when `use_cuda_`, the device is available, and `consider_footprint_` is false:

- Fill `CostCriticDeviceScore` from the values `score()` already computes (`origin_*`, `resolution_`, `size_*`, collision parameters, `near_goal`, rows, full time-column count, `trajectory_point_step_`, whether the collision vector is non-empty).
- Call `score_center` with `trajectories.x.data()` and `trajectories.y.data()`.
- On false, log once and use `accumulateFromPoseCosts` with a null gather.
- On true, write collision flags from the returned `unsigned char` buffer when tracking is on, and call `applyWeightAndPower` with the device repulsive vector. Do not run the host cell loop after a true return.

`score_center` owns a stream and device buffers for the life of `CostCriticCuda`, grows them when the byte counts grow, and frees them in the destructor. Every call copies `size_x * size_y` costmap bytes and both trajectory buffers to the device. Synchronize the stream before writing the host repulsive pointer. Increment `cost_critic_cuda_launch_count` only after the kernel launch and the synchronize both succeed. Record the four `CostCriticCudaTimings` with CUDA events around the costmap copy, the trajectory copy, the kernel, and the copy back.

Kernel: one thread per trajectory, block size 128. For strided column `j`, time index is `j * point_step`, and the sample address is `time * rows + i`. Cell rules are the ones in `CostCritic::worldToMapFloat`, `getIndex`, and `inCollision` for `consider_footprint_ == false`: below the origin or outside `size_x`/`size_y` is pose cost 255; in-map pose cost below 1 adds nothing; 254 collides; 253 collides; 255 collides only when `is_tracking_unknown` is false; the first collision stores `collision_cost` and stops that trajectory; otherwise pose cost `>= near_collision_cost` adds `critical_cost`; otherwise a near-goal sample adds nothing and a far sample adds the pose cost. `all_collided` is the AND of every trajectory's collision bit.

- [ ] **Step 4: Run the parity tests**

Rebuild with `-DNAV2_MPPI_CUDA=ON` and run `./build/nav2_mppi_controller/cost_critic_parity_test`.

Expected: every classical test and every `Cuda*` test passes. `CudaDisabledCritic` keeps the launch count unchanged. The other enabled `Cuda*` tests increase it.

- [ ] **Step 5: Commit**

```bash
git add nav2_mppi_controller/CMakeLists.txt \
  nav2_mppi_controller/src/critics/cost_critic.cpp \
  nav2_mppi_controller/src/critics/cost_critic_cuda.cu \
  nav2_mppi_controller/src/critics/cost_critic_cuda_stub.cpp \
  nav2_mppi_controller/test/cost_critic_parity_test.cpp
git commit -m "$(cat <<'EOF'
feat: score CostCritic centers on CUDA with CPU parity

EOF
)"
```

---

### Task 4: Footprint mode stays on the classical result

**Files:**
- Modify: `nav2_mppi_controller/src/critics/cost_critic.cpp`
- Modify: `nav2_mppi_controller/src/critics/cost_critic_cuda.cu`
- Modify: `nav2_mppi_controller/test/cost_critic_parity_test.cpp`

**Interfaces:**
- Consumes: `CostCriticCuda::gather_center` and `accumulateFromPoseCosts` from Task 3.
- Produces: no new public signature. `gather_center` writes a column-major pose-cost matrix of shape `(rows, strided_cols)` at `pose_costs[j * rows + i]`.

- [ ] **Step 1: Write the failing footprint parity tests**

Footprint cases declare, before `Costmap2DROS::on_configure`:

```cpp
costmap_ros->declare_parameter("plugins", std::vector<std::string>{});
costmap_ros->declare_parameter(
  "footprint",
  std::string("[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]"));
```

`consider_footprint` is true. `cost_weight` is `254.0f`. One time step unless noted. Pose yaw is `0`. The center pose for cell `(10, 10)` is `(world_from_cell(10), world_from_cell(10))`.

Paint a lethal ring only for the hit case: every cell with `mx` in `{8, 12}` and `my` in `[8, 12]`, and every cell with `my` in `{8, 12}` and `mx` in `[8, 12]`, cost `254`. Set the center cell after the ring.

| Test | center cell | ring | second sample | repulsive | collided | fail |
|---|---|---|---|---|---|---|
| `FootprintHit` | `(10,10)=200` | yes | none | 1000000 | true | true |
| `FootprintClear` | `(10,10)=200` | no | none | 200 | false | false |
| `FreeCenterSkipsFootprint` | `(10,10)=0` | yes | none | 0 | false | false |
| `InscribedCenterFootprintClear` | `(10,10)=253` | no | none | 300 | false | false |
| `FootprintEarlyExit` | `(10,10)=200` with ring | yes | time 1 at cell `(20,10)` painted `40` | 1000000 | true | true |

`FootprintEarlyExit` uses 2 time steps and `trajectory_point_step` 1. Under `#if NAV2_MPPI_HAS_CUDA`, the same five cases run through `expect_cuda_matches_classical` as `CudaFootprintHit`, `CudaFootprintClear`, `CudaFreeCenterSkipsFootprint`, `CudaInscribedCenterFootprintClear`, and `CudaFootprintEarlyExit`. Each enabled case expects the launch count to increase.

Also keep a CPU-only `expect_classical` assertion for these five, so the footprint numbers are locked before the device path is trusted.

- [ ] **Step 2: Run the footprint tests and confirm the CUDA ones fail**

Rebuild with `-DNAV2_MPPI_CUDA=ON` and run `./build/nav2_mppi_controller/cost_critic_parity_test --gtest_filter=CostCriticParity.CudaFootprintHit`.

Expected: FAIL because `consider_footprint_` still takes the CPU branch and the launch count does not increase. The CPU `FootprintHit` test passes.

- [ ] **Step 3: Implement the gather**

When `use_cuda_` is true, the device is available, and `consider_footprint_` is true, call `gather_center`. On false, use the null-gather host loop. On true, copy the device pose costs into an `Eigen::ArrayXXf` of shape `(rows, strided_cols)` and call `accumulateFromPoseCosts` with that pointer.

Gathered pose cost `255` means outside the map. Gathered pose cost below `1` continues, same as an in-map free cell. `inCollision` and `footprintCostAtPose` stay on the host and still see `x`, `y`, and yaw from the strided trajectory maps.

`gather_center` uses the same copy, stream, timing, and launch-count rules as `score_center`. One thread per strided sample is enough. The pose-cost value follows `worldToMapFloat` and `getIndex` only. It does not apply collision or footprint rules.

- [ ] **Step 4: Run the parity tests**

Run `./build/nav2_mppi_controller/cost_critic_parity_test`.

Expected: the five footprint tests and their `Cuda*` twins pass, and the Task 3 center tests still pass.

- [ ] **Step 5: Commit**

```bash
git add nav2_mppi_controller/src/critics/cost_critic.cpp \
  nav2_mppi_controller/src/critics/cost_critic_cuda.cu \
  nav2_mppi_controller/test/cost_critic_parity_test.cpp
git commit -m "$(cat <<'EOF'
feat: gather CostCritic centers on CUDA and score footprints on CPU

EOF
)"
```

---

### Task 5: Map resize between scores

**Files:**
- Modify: `nav2_mppi_controller/test/cost_critic_parity_test.cpp`
- Modify: `nav2_mppi_controller/src/critics/cost_critic_cuda.cu` only if the Task 3 buffers ignore a later shape change.

**Interfaces:**
- Consumes: `score_center` buffer growth from Task 3.
- Produces: no new signature.

- [ ] **Step 1: Write the failing resize test**

```cpp
TEST(CostCriticParity, CudaResizeUsesNewGrid)
{
#if !NAV2_MPPI_HAS_CUDA
  GTEST_SKIP();
#endif
  // First score: default 0.1 m grid, one sample at cell (10, 10) painted 254.
  // expect_cuda_matches_classical for that single lethal sample.
  // Then costmap->resizeMap(15, 15, 0.2, -1.0, -1.0), zero the new grid,
  // paint cell (3, 3) as 254, and score one sample at world (-0.3, -0.3).
  // mx = (-0.3 - (-1.0)) / 0.2 = 3.5, which casts to 3.
  // CPU and CUDA costs are both collision_cost * (254/254) / 1,
  // fail_flag is true, the collision flag is true, and the launch count increases
  // on the second score as well as the first.
}
```

Use the same critic objects for both scores so the device buffers allocated for the 5 m map are still alive. `consider_footprint` is false. `cost_weight` is `254.0f`. `trajectory_point_step` is 1. One trajectory, one time step.

- [ ] **Step 2: Run the resize test**

Run `./build/nav2_mppi_controller/cost_critic_parity_test --gtest_filter=CostCriticParity.CudaResizeUsesNewGrid`.

Expected: FAIL if the second score still reads the old dimensions or the old origin. If Task 3 already re-reads `size_x`, `size_y`, origin, and resolution on every `score()` and reallocates when the byte count changes, this test passes immediately. Treat that pass as success and skip the production edit.

- [ ] **Step 3: Reallocate when the grid changes**

On each `score_center` and `gather_center`, copy `params.size_x * params.size_y` bytes from the pointer passed in. If the device costmap capacity is smaller, free and allocate the new size before the copy. Do the same for the trajectory and output buffers when `rows` or `time_cols` grow. A smaller later map still copies only the new byte count and passes the new `size_x`, `size_y`, origin, and resolution into the kernel.

- [ ] **Step 4: Run the full parity binary**

Run `./build/nav2_mppi_controller/cost_critic_parity_test`.

Expected: `CudaResizeUsesNewGrid` passes, and every earlier parity test passes.

- [ ] **Step 5: Commit**

```bash
git add nav2_mppi_controller/test/cost_critic_parity_test.cpp \
  nav2_mppi_controller/src/critics/cost_critic_cuda.cu
git commit -m "$(cat <<'EOF'
test: require CUDA CostCritic to follow a resized costmap

EOF
)"
```

---

### Task 6: Record transfer and kernel time

**Files:**
- Create: `nav2_mppi_controller/benchmark/cost_critic_cuda_benchmark.cpp`
- Modify: `nav2_mppi_controller/benchmark/CMakeLists.txt`
- Modify: `nav2_mppi_controller/test/cost_critic_parity_test.cpp`

**Interfaces:**
- Consumes: `cost_critic_cuda_last_timings()` and `cost_critic_cuda_launch_count()` from Task 2.
- Produces: benchmark executable `cost_critic_cuda_benchmark`, built only when `NAV2_MPPI_CUDA` is `ON`.

- [ ] **Step 1: Write the failing timing test**

```cpp
TEST(CostCriticParity, RecordsTransferAndKernelTime)
{
#if !NAV2_MPPI_HAS_CUDA
  GTEST_SKIP();
#endif
  if (!cost_critic_cuda_device_available()) {
    GTEST_SKIP();
  }
  // use_cuda true, consider_footprint false, cost_weight 254,
  // trajectory_point_step 2, batch 2000, time steps 56.
  // All samples at cell (5, 5), cell cost 0.
  // After score():
  const auto timings = cost_critic_cuda_last_timings();
  EXPECT_GE(timings.costmap_h2d_ms, 0.f);
  EXPECT_GE(timings.trajectory_h2d_ms, 0.f);
  EXPECT_GE(timings.kernel_ms, 0.f);
  EXPECT_GE(timings.d2h_ms, 0.f);
  EXPECT_GT(cost_critic_cuda_launch_count(), 0);
}
```

Print the four millisecond values to stdout. Do not compare them to a speed limit.

Add `cost_critic_cuda_benchmark.cpp` with the same batch, steps, and step. It configures `CostCritic` with `use_cuda` true and times a full `score()` via `benchmark::State`. Guard the `benchmark/CMakeLists.txt` entry with `if(NAV2_MPPI_CUDA)`.

- [ ] **Step 2: Run the timing test**

Rebuild with `-DNAV2_MPPI_CUDA=ON` and run `./build/nav2_mppi_controller/cost_critic_parity_test --gtest_filter=CostCriticParity.RecordsTransferAndKernelTime`.

Expected: PASS if Task 3 already fills `cost_critic_cuda_last_timings`. The printed line shows the four times. FAIL only if the timings stay at the stub zeros while a kernel launch was counted, or if the test does not compile because the benchmark target is missing. A missing benchmark target is fixed in Step 3.

- [ ] **Step 3: Build the benchmark target**

Register `cost_critic_cuda_benchmark` with the same libraries as `optimizer_benchmark`, plus `CUDA::cudart` when the option is on. The benchmark calls `score()`; it does not set a byte threshold or a time threshold.

- [ ] **Step 4: Run the parity binary and the benchmark once**

```bash
./build/nav2_mppi_controller/cost_critic_parity_test
./build/nav2_mppi_controller/cost_critic_cuda_benchmark --benchmark_min_time=0.5s
```

Expected: the parity binary passes, including `RecordsTransferAndKernelTime`. The benchmark exits 0 and prints a wall time. Neither result fails because of the absolute milliseconds.

- [ ] **Step 5: Commit**

```bash
git add nav2_mppi_controller/benchmark/cost_critic_cuda_benchmark.cpp \
  nav2_mppi_controller/benchmark/CMakeLists.txt \
  nav2_mppi_controller/test/cost_critic_parity_test.cpp
git commit -m "$(cat <<'EOF'
test: record CostCritic CUDA transfer and kernel time

EOF
)"
```

---

## Self-review

Spec coverage:

- `use_cuda` default false and the CPU loop: Task 2.
- Bitwise parity gate, including power, stride, unknown, out-of-map, near goal, early exit, fail flag, empty collision vector, and disabled critic: Tasks 1 and 3.
- Footprint checker stays on the host, center gather on the device: Task 4.
- Copy the costmap every score, persistent buffers, stream sync, CPU fallback: Task 3.
- Resize: Task 5.
- Launch proof and timing record without a speed gate: Tasks 3 and 6.
- CMake option default off, `nvidia-smi` architecture, toolkit floor, no plugin XML change, no package.xml change: Tasks 2 and 3.
- Jetson execution is not an acceptance run. The plan does not add a cross-compile task.

Review Focus tests live in Tasks 1, 3, and 4. Resize is Task 5. No uncovered spec input remains from the acceptance list.
