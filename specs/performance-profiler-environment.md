# Desktop GPU baseline environment

**Source revision:** 9557655b38a839b7c14e0aff862f63332f7ec7e4 + benchmark/mppi_gpu_baseline.cpp and benchmark/CMakeLists.txt measurement changes.
**Date:** 2026-10-23.
**Scope:** проверенная среда desktop RTX 3060 и exploratory measurement contract; не полная системная приёмка.

## Identity and constraints

- [VERIFIED] Repository: ROS 2 Navigation2, ament/colcon/CMake; core C++20, CUDA units C++17. package manifest and nav2_common/cmake/nav2_package.cmake.
- [VERIFIED] Actual GPU: NVIDIA GeForce RTX 3060, 12288 MiB, driver 550.163.01. Read-only nvidia-smi inventory outside sandbox.
- [VERIFIED — human clarification, 2026-10-24] Текущий desktop target и baseline: RTX 3060. Jetson отложен; модель desktop соответствует фактическим измерениям. Ограничения workload и фоновой нагрузки сохраняются.
- [VERIFIED] CPU Intel Core i7-10700F, 8 cores/16 logical CPUs; scaling governor powersave, clocks not pinned. Physical host RAM MemTotal 65710804 kB. lscpu/sysfs/proc from container.
- [VERIFIED] At discovery GPU reported P5, graphics 495 MHz, memory 810 MHz, 46 C, 21.88 W, 7625 MiB used, utilization 29%. These are one-time observations, not fixed measurement conditions. Existing desktop/other GPU workload not stopped.
- [VERIFIED] Existing container nav2-rolling, image sha256:31c33f4b0373eafdd68fae9b07dcc4e340f185fe51682d5495c201d17d837f1a; Ubuntu 26.04.1, ROS Rolling, GCC 15.2.0, CUDA Toolkit 12.4.131, CMake 4.2.3, Nsight Systems 2023.4.4.54, Compute Sanitizer 2024.1.1.0.
- [VERIFIED] Host has different toolchain (GCC 11.4, CUDA 11.5, ROS Humble); these are not the measurement build tools.
- [VERIFIED] Existing /ws maps to feature worktree. New package staged separately at /tmp/nav2-gpu-baseline-9557655b/src/nav2_mppi_controller; dependencies from /ws/install. CPU/GPU build/install/log directories separate, Release, CUDA architecture 86. No project source modifications in the mounted feature worktree.
- [UNKNOWN] Exact revision/artifact hashes of existing dependency overlay. Current dependency identity is image + existing /ws/install overlay, recorded as a reproducibility limitation.
- [UNKNOWN] Product robot/plugin configuration, latency/resource budgets and acceptable numerical changes not approved. Fixed workload below is an exploratory measurement contract.

## Measurement contract

[VERIFIED — measurement harness contract; completion counts in performance report] Fixed robot pose (2,2), zero reported current velocity, straight 1 m path on a 40x40 costmap at 0.1 m resolution. Footprint ON: ordered square half-width 0.15 m, zero padding; OFF: radius 0.15 m. Costmap global frame odom matches pose/path. One of CostCritic/ObstaclesCritic + Constraint/Goal/GoalAngle/PathAlign/PathFollow/PathAngle/PreferForward; DiffDrive. 2000 trajectories x 56 steps, 2 optimizer iterations per controller call; 50 warmup calls; 1000 measured calls per backend x 5 repetitions per scenario. Footprint OFF/ON, free map or lateral lethal/inflated bands. CPU/GPU run order alternates between repetitions.

Numerical comparison: default std::default_random_engine instances, same standard library, regenerate_noises=false; independently configured controllers replay same sequence. No changes to noise implementation. Compare all finite linear.x/linear.y/angular.z command double representations including warmup; exact launch count must equal calls x 2 on GPU and zero on CPU. Existing critic parity suite additionally checks costs/flags, but full trajectory/cost representations are not compared by this harness.

Wall metric: steady_clock around complete computeVelocityCommands (including parameter/costmap locks, optimizer, validators), microseconds per call. Raw samples retained; nearest-rank p50/p95/p99/max and arithmetic mean. Process CPU metric: std::clock around measured block, microseconds per call; includes process threads and harness bookkeeping outside wall intervals. Clocks/observer overhead not independently calibrated; use identical harness for both backends. Setup/teardown/output excluded from measured wall samples. No visualization, live sensors or physics; this is repeated fixed-input controller evaluation, not closed-loop navigation or deadline acceptance.

## Operations and recovery

No GPU clock/power settings, unrelated containers or hardware state changed. New benchmark target builds in CPU-only and CUDA configurations; production defaults/scoring APIs unchanged. Use CPU mode or previous binary to roll back measurement harness. Temporary container build workspaces and task artifacts are intentionally retained for reproducibility.
