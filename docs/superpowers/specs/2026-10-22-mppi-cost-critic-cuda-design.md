# MPPI CostCritic CUDA scoring

## Intent

`mppi::critics::CostCritic::score` runs on every MPPI control cycle. It walks a batch of trajectories and reads one cell of the local costmap per sample. That read is a regular grid gather over `Costmap2D`'s flat `unsigned char` buffer, and the same buffer is reused for every trajectory in the cycle.

This project adds an optional CUDA path for that scoring step so the same source can run on a desktop NVIDIA GPU and on a Jetson. The reason to do this critic first is the control deadline: inflation and AMCL update more slowly and stay out of this project.

The person who asked for the work chose `CostCritic`, asked for the same command the CPU path would pick, and accepted this shape: the GPU scores the robot center and the cell-threshold collision penalty; `FootprintCollisionChecker` stays on the CPU; the CPU path stays the default.

## Goals

- A critic parameter `use_cuda`, default `false`, selects the device path at runtime.
- With `use_cuda` false, `score()` is the current CPU implementation, including its parameter defaults and plugin class `mppi::critics::CostCritic`.
- With `use_cuda` true and a CUDA device available, `score()` writes the same `data.costs` contribution, the same `data.fail_flag`, and the same `data.trajectories_in_collision` entries as the CPU implementation on the same inputs. This parity is a completion gate: the work is not done until those tests pass. The comparison is bitwise equality with the classical CPU `score()`, not a tolerance.
- With `use_cuda` true and no visible CUDA device, `score()` uses the CPU implementation and logs one warning.
- A build with the CUDA option off compiles and runs the existing critic tests on a machine without the CUDA toolkit.
- On a machine with a device, a test shows that the device path actually launches a kernel, and a benchmark records transfer plus kernel time for one `score()` call.
- The same sources build on the desktop used for development (CUDA 11.5, RTX 3060, compute capability 8.6) and, when `CMAKE_CUDA_ARCHITECTURES` is set, for a Jetson. Jetson Orin is architecture `87`. This workspace does not contain a Jetson, so Jetson execution is not an acceptance run for this project.

## Non-goals

- `InflationLayer`, `DistanceTransform`, `ObstaclesCritic`, AMCL, and the other MPPI critics.
- Moving `FootprintCollisionChecker::footprintCostAtPose` onto the device.
- Replacing Eigen trajectory storage or the critic plugin id.
- Changing the defaults of `cost_power`, `cost_weight`, `critical_cost`, `collision_cost`, `near_collision_cost`, `near_goal_distance`, `trajectory_point_step`, or `consider_footprint`.
- `cudaMallocManaged`, zero-copy, or a Jetson-only memory path. Explicit host-to-device copies are the memory contract.
- A guaranteed speedup at small batch sizes. If the copy dominates, the CPU default stays, and the measurement is the result.

## Anti-goals

- A device path that, for `consider_footprint == false`, changes which trajectory wins relative to the CPU on the same costmap and the same samples.
- Turning the device path on unless `use_cuda` is set.
- A CUDA dependency that breaks the default package build.

## Behavior the device path must match

Source of truth: `CostCritic::score`, `CostCritic::worldToMapFloat`, `CostCritic::getIndex`, and `CostCritic::inCollision` in `nav2_mppi_controller`.

Costmap cell values used by the collision test:

- `nav2_costmap_2d::LETHAL_OBSTACLE` is 254
- `nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE` is 253
- `nav2_costmap_2d::NO_INFORMATION` is 255

Per strided sample `(i, j)`, with trajectory row `i` and strided time column `j`:

1. Read `Tx`, `Ty`, and yaw with the same strides the CPU uses: `strided_traj_cols = floor((cols - 1) / trajectory_point_step_) + 1`, `outer_stride = rows * trajectory_point_step_`, Eigen stride `(outer_stride, 1)` over the column-major `trajectories.x`, `trajectories.y`, and `trajectories.yaws` buffers.
2. If `Tx < origin_x` or `Ty < origin_y`, the sample is outside the map and its pose cost is `255`.
3. Otherwise the cell is `mx = (unsigned)((Tx - origin_x) / resolution)`, `my = (unsigned)((Ty - origin_y) / resolution)`, index `my * size_x + mx`. If `mx >= size_x` or `my >= size_y`, the pose cost is `255`. Otherwise the pose cost is the unsigned char at that index, promoted to float.
4. If the sample is inside the map and the pose cost is `< 1`, it adds nothing and the scan continues.
5. Collision uses the center pose cost when `consider_footprint` is false:
   - 254 collides
   - 253 collides
   - 255 collides when `is_tracking_unknown` is false, and does not collide when it is true
   - any other value does not collide
6. On the first colliding sample of a trajectory, that trajectory's repulsive cost becomes `collision_cost` exactly, later samples of that trajectory add nothing, and `trajectories_in_collision[i]` is set when that vector is non-empty.
7. A non-colliding sample with pose cost `>= near_collision_cost` adds `critical_cost`.
8. Otherwise, when `state.local_path_length < near_goal_distance`, the sample adds nothing. When the robot is not near the goal, the sample adds the pose cost.
9. `fail_flag` is true only when every trajectory collided.
10. After the repulsive vector exists, the host applies the existing weight and power update: divide `weight` by the strided column count, multiply, and when `power > 1` raise that product to `power` with the host `pow`, then add it into `data.costs`.

`consider_footprint == true` keeps the footprint test on the host. The device gathers the center pose cost for every strided sample, including the out-of-map value 255. The host then runs the existing scan order. Where the CPU would call `footprintCostAtPose` (center cost `>= possible_collision_cost`, or `possible_collision_cost < 1`), the host still calls it. Collision for that sample uses the footprint cost with the existing rule: 254 collides, 253 does not, 255 collides only when unknown space is not tracked. Repulsive accumulation, early exit, `fail_flag`, and the collision vector stay on the host in this mode. The weight and power update stays on the host in both modes.

Disabled critic (`enabled_ == false`) returns without reading the device.

## Integration

- Runtime switch: `use_cuda`, bool, default `false`, read in `CostCritic::initialize` through the existing critic `parameters_handler_` getter. Dynamic updates are not required for this parameter.
- Build switch: CMake option `NAV2_MPPI_CUDA`, default `OFF`. `OFF` does not call `find_package(CUDAToolkit)` and does not compile `.cu` files. `ON` requires the CUDA toolkit, enables the CUDA language for the critic library, and compiles the device sources into `mppi_critics`.
- Toolkit floor: CUDA 11.4 or newer. This covers the development machine's CUDA 11.5 and JetPack 6's CUDA 12.
- When `CMAKE_CUDA_ARCHITECTURES` is empty, configure-time detection fills it from `nvidia-smi --query-gpu=compute_cap` (this machine reports `8.6`, stored as `86`). CMake on this machine is 3.22.1, which has no `native` CUDA architecture value. An explicit `CMAKE_CUDA_ARCHITECTURES` still wins, including `87` for Jetson Orin. Cross-compiling from this desktop for a Jetson is out of scope.
- One persistent device context owned by the critic: a stream, a costmap buffer, and trajectory buffers, allocated on first device `score()` and grown when the map size or the batch shape grows. Freed when the critic is destroyed.
- Copy the costmap's `unsigned char` buffer to the device at the start of every device `score()`. The master grid is updated in place, so a stable pointer is not a sign that the bytes are unchanged. Copy trajectory samples the kernel reads. Copy the repulsive costs and collision flags back before the host weight and power update.
- The critic does not take a new costmap mutex. It keeps the CPU critic's threading assumption: `score()` runs on the controller thread while that thread is allowed to read the costmap.
- No new plugin XML entry.

## Acceptance tests

Device tests are built only when `NAV2_MPPI_CUDA=ON`. If no device is visible at runtime, they `GTEST_SKIP`. On this workspace a device is visible, so the acceptance run executes them.

Compare the device path with the CPU path on one synthetic costmap that contains free cells (`0`), inflated cells (`1..252`), inscribed (`253`), lethal (`254`), and unknown (`255`):

- inside free space adds nothing
- a lethal cell sets `collision_cost`, skips later samples, and sets the collision flag
- an inscribed cell collides when `consider_footprint` is false
- unknown collides only when tracking unknown is false
- a point with `x < origin_x` or `y < origin_y`, and a point past `size_x` or `size_y`, scores as 255
- near the goal, a non-critical inflated cost adds nothing, and a near-collision cost still adds `critical_cost`
- `trajectory_point_step > 1` reads the strided samples only
- `power == 1` and `power > 1` both match, because `pow` stays on the host
- `consider_footprint == true` matches the CPU path, including a footprint hit that the center cell alone would not call a collision, and a center cell inside the circumscribed band that the footprint clears
- every trajectory colliding sets `fail_flag`
- an empty `trajectories_in_collision` leaves collision tracking off
- `use_cuda` false does not launch a kernel
- `use_cuda` true with a device launches a kernel
- a map resize between two `score()` calls uses the new dimensions

Match means bitwise equality of the `data.costs` array, `fail_flag`, and any collision flags the CPU path writes. The test builds those expectations by running the CPU `score()` on a copy of the inputs.

A separate timing record, test or benchmark, reports milliseconds for the costmap copy, the trajectory copy, the kernel, and the copy back, for a batch of 2000 trajectories and 56 time steps with `trajectory_point_step == 2`. It does not fail on a threshold.

## Constraints

- C++ and CUDA live in `nav2_mppi_controller`. The public critic API stays `initialize()` and `score(CriticData &)`.
- Apache-2.0 headers, matching the surrounding critic sources.
- `gpu-dsp` checks from the embedded accelerator apply: own the device buffers for the critic lifetime, synchronize before the host reads results, keep a CPU fallback, and measure the transfer plus the kernel.
- Development acceptance is the RTX 3060 on this machine. Jetson is a source-level target, not a run target here.

## Decisions left to the builder

- File split inside `nav2_mppi_controller`, as long as CPU-only builds do not include CUDA headers.
- Kernel launch geometry, occupancy, and whether the footprint-off scan is its own kernel or part of the gather.
- How the test detects that a kernel launched, as long as a successful configuration without a launch fails the test.
- Names of private helpers.

## Calls made in this document

These were not spelled out by the requester. They are fixed here so a plan does not reopen them.

- Parameter name `use_cuda`, default false. CMake option `NAV2_MPPI_CUDA`, default `OFF`.
- The host keeps the weight and power update so `pow` cannot drift between libm and the device.
- The costmap is copied on every device `score()`.
- Empty `CMAKE_CUDA_ARCHITECTURES` is filled from `nvidia-smi` compute capability (`86` on this machine). Jetson Orin is documented as `87`. No cross-compile in this project.
- CUDA toolkit floor is 11.4.
- Bitwise equality is the match rule for both footprint modes.
- The timing record has no pass/fail speed threshold.
