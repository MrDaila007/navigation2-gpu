# MPPI baseline correctness commands

## Reproduction

Host staging:

```bash
git switch -c fix/mppi-baseline-checks feat/gpu-adaptation
docker exec nav2-rolling mkdir -p /tmp/nav2-mppi-baseline-fixes-9557655b/src
docker cp nav2_mppi_controller nav2-rolling:/tmp/nav2-mppi-baseline-fixes-9557655b/src/nav2_mppi_controller
```

Inside container, base environment and working directory:

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-baseline-fixes-9557655b
```

Before production edits, after adding the regression test:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ament_cppcheck src/nav2_mppi_controller/test/critic_manager_test.cpp src/nav2_mppi_controller/test/utils_test.cpp src/nav2_mppi_controller/test/critics_tests.cpp --include_dirs src/nav2_mppi_controller/include
build-cpu/nav2_mppi_controller/test/optimizer_unit_tests --gtest_filter=OptimizerTests.MotionModelLoaderOutlivesInstances
```

Both exit 1: ten diagnostics and unsafe plugin unload respectively. Declaration-order-only coherent build/install passes destruction but fails replacement regression (lifetime-core-installed.log); final reuse guard passes both.

## CPU Release, fresh shell

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-baseline-fixes-9557655b
colcon --log-base log-cpu build --build-base build-cpu --install-base install-cpu --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
source install-cpu/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-cpu/nav2_mppi_controller --output-on-failure
```

## CUDA Release, fresh shell

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-baseline-fixes-9557655b
colcon --log-base log-gpu build --build-base build-gpu --install-base install-gpu --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86 -DCMAKE_BUILD_TYPE=Release
source install-gpu/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-gpu/nav2_mppi_controller --output-on-failure
```

Smoke executable invoked serially for each combination of cost/obstacles, 0/1 footprint, free/bands:

```bash
RCUTILS_LOGGING_USE_STDOUT=0 build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline pair cost 0 free 10 1
```

Eight logs retained. Subprocess assertions required exit 0, PARITY,command_bits,60,PASS and absence of Attempting to unload library / LifecycleNode is not shut down / BASELINE_ERROR in stderr. Numerical stdout and ROS stderr retained separately.

## ASan/UBSan and leak checks

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-baseline-fixes-9557655b
MAKEFLAGS="-j2 -l2" colcon --log-base log-sanitize build --build-base build-sanitize --install-base install-sanitize --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer" "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined" "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined"
source install-sanitize/setup.bash
export ASAN_OPTIONS=detect_leaks=1:halt_on_error=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
build-sanitize/nav2_mppi_controller/test/optimizer_unit_tests --gtest_filter="OptimizerTests.MotionModelLoaderOutlives*"
build-sanitize/nav2_mppi_controller/benchmark/mppi_gpu_baseline cpu cost 1 bands 10 1
```

Full build/install mandatory after header layout changes. Final coherence checked with readelf -n on build/install libmppi_controller.so and libmppi_critics.so for all three variants; IDs saved in artifacts/elf-build-ids.txt. Existing tools/run_sanitizers was read but not run because it cleans workspace build/install/log and assumes mixins/dependency setup. Explicit isolated flags preserve existing work.
