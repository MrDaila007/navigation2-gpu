# P0 validator loader — commands and reproduction

[VERIFIED] 2026-10-25. Baseline 57c8c327; branch fix/mppi-validator-reconfigure-lifetime from feat/gpu-adaptation; initial tree clean. All builds run in existing local nav2-rolling container, never in mounted /ws source. Host stages only nav2_mppi_controller to isolated `/tmp/nav2-mppi-r2-p0-57c8c327/src/nav2_mppi_controller`. Exact final source hashes accompany the report.

## Environment/staging (host)

```bash
git switch -c fix/mppi-validator-reconfigure-lifetime feat/gpu-adaptation
docker ps --filter name=nav2-rolling --format '{{.Names}} {{.Status}}'
docker exec nav2-rolling bash -c 'g++ --version; nvcc --version; cmake --version; nvidia-smi --query-gpu=name,driver_version --format=csv,noheader'
docker exec nav2-rolling bash -c 'test ! -e /tmp/nav2-mppi-r2-p0-57c8c327 && mkdir -p /tmp/nav2-mppi-r2-p0-57c8c327/src'
docker cp nav2_mppi_controller nav2-rolling:/tmp/nav2-mppi-r2-p0-57c8c327/src/nav2_mppi_controller
```

Add regressions first, leave production optimizer.cpp unchanged. In each fresh container shell:

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-r2-p0-57c8c327
```

## RED: coherent CPU build/install before fix

```bash
MAKEFLAGS='-j4 -l4' colcon --log-base log-red build --build-base build-red --install-base install-red --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=Release > build-red.log 2>&1
source install-red/setup.bash
build-red/nav2_mppi_controller/test/optimizer_unit_tests --gtest_filter='OptimizerTests.ValidatorLoader*' > red-optimizer.log 2>&1
build-red/nav2_mppi_controller/test/controller_state_transition_test --gtest_filter='ControllerLifecycle.ReconfigureGpuPrerequisite' > red-controller.log 2>&1
```

Build exit 0, each test executable exit 1. Actual 3+1 selected tests all fail on captured real class_loader unsafe-unload diagnostics; no zero-test PASS. Red-source hashes retained separately. Final controller regression adds a computeVelocityCommands call in each lifecycle cycle after the red reproduction; optimizer regressions are unchanged.

Apply only the validator_loader_ reuse guard; copy changed optimizer.cpp and final controller test to staged package. Final builds use new complete build/install directories, never target-only compile with old install DSOs.

## GREEN CPU Release

```bash
MAKEFLAGS='-j4' colcon --log-base log-cpu-final build --build-base build-cpu --install-base install-cpu --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=Release > build-cpu-final.log 2>&1
source install-cpu/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-cpu/nav2_mppi_controller --output-on-failure --no-tests=error > ctest-cpu.log 2>&1
```

## GREEN CUDA Release (fresh base shell)

```bash
MAKEFLAGS='-j4' colcon --log-base log-gpu-final build --build-base build-gpu --install-base install-gpu --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86 -DCMAKE_BUILD_TYPE=Release > build-gpu-final.log 2>&1
source install-gpu/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-gpu/nav2_mppi_controller --output-on-failure --no-tests=error > ctest-gpu.log 2>&1
```

After coherent install, focused regressions may be rerun serially with `--gtest_filter='OptimizerTests.ValidatorLoader*:OptimizerTests.MotionModelLoaderOutlives*'` and `--gtest_filter='ControllerLifecycle.ReconfigureGpuPrerequisite'`; assert 5+1 executed tests.

CUDA controller smoke (stdout/stderr separated, eight scenarios):

```bash
mkdir -p smoke
for critic in cost obstacles; do
  for footprint in 0 1; do
    for scene in free bands; do
      RCUTILS_LOGGING_USE_STDOUT=0 build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline pair "$critic" "$footprint" "$scene" 10 1 > "smoke/$critic-fp$footprint-$scene.log" 2> "smoke/$critic-fp$footprint-$scene.stderr.log"
    done
  done
done
```

Require exit 0 and PARITY,command_bits,60,PASS for every scenario, absence of unsafe unload/unfinished lifecycle/BASELINE_ERROR diagnostics. This is 480 command pairs including warmup, not a speedup measurement.

## ASan/UBSan/leak (fresh base shell)

```bash
MAKEFLAGS='-j2 -l2' colcon --log-base log-sanitize build --build-base build-sanitize --install-base install-sanitize --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined' '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined' > build-sanitize.log 2>&1
source install-sanitize/setup.bash
export ASAN_OPTIONS=detect_leaks=1:halt_on_error=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
build-sanitize/nav2_mppi_controller/test/optimizer_unit_tests --gtest_filter='OptimizerTests.ValidatorLoader*:OptimizerTests.MotionModelLoaderOutlives*' > sanitize-optimizer.log 2>&1
build-sanitize/nav2_mppi_controller/test/controller_state_transition_test --gtest_filter='ControllerLifecycle.ReconfigureGpuPrerequisite' > sanitize-controller.log 2>&1
RCUTILS_LOGGING_USE_STDOUT=0 build-sanitize/nav2_mppi_controller/benchmark/mppi_gpu_baseline cpu cost 1 bands 10 1 > sanitize-benchmark.log 2> sanitize-benchmark.stderr.log
```

## Identity/static checks

```bash
git diff --check
```

For cpu/gpu/sanitize, compare readelf -n build/install libmppi_controller.so, libmppi_critics.so and libmppi_trajectory_validators.so IDs after install. Full test/lint XML retained locally; inspect cppcheck test cases/skipped markers, build warnings and selected GTest counts. Copy only compact source/ELF identity and report to Git; raw logs/XML stay local.

Tool portability: rg is absent inside this container (first log-inspection attempt exit 127); used grep there, without installing tools. No runtime/code outcome depends on that failed inspection command.

## Intermediate attempt disposition

[VERIFIED] The first final-source CPU/CUDA build attempts stopped with exit 2 because the new controller smoke passed a bare {} Twist argument, incompatible with the generated explicit ROS message constructor under -Werror. Corrected to a named geometry_msgs::msg::Twist variable, then reran full coherent colcon build/install (log-cpu-final/log-gpu-final). Those failed compile attempts are not test evidence. Sanitizer compilation of the controller test uses the corrected file.
