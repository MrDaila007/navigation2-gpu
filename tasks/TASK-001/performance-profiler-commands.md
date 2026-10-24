# Baseline commands and evidence locations

All Docker commands operate on the existing local nav2-rolling container. Source staged independently from its mounted feature worktree. No other container modified.

## Environment

```bash
docker inspect nav2-rolling --format '{{.Image}} {{json .Mounts}}'
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
docker exec nav2-rolling bash -lc 'cat /etc/os-release; g++ --version; nvcc --version; cmake --version; lscpu; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'
```

## Staging and isolated builds

```bash
docker exec nav2-rolling mkdir -p /tmp/nav2-gpu-baseline-9557655b/src
docker cp nav2_mppi_controller nav2-rolling:/tmp/nav2-gpu-baseline-9557655b/src/nav2_mppi_controller
```

Inside container, each in a fresh shell:

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-gpu-baseline-9557655b
colcon --log-base log-cpu build --build-base build-cpu --install-base install-cpu --packages-select nav2_mppi_controller --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
```

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-gpu-baseline-9557655b
colcon --log-base log-gpu build --build-base build-gpu --install-base install-gpu --packages-select nav2_mppi_controller --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86 -DCMAKE_BUILD_TYPE=Release
```

After source-only fixture corrections, both executables rebuilt with:

```bash
cmake --build build-cpu/nav2_mppi_controller --target mppi_gpu_baseline --parallel 2
cmake --build build-gpu/nav2_mppi_controller --target mppi_gpu_baseline --parallel 2
```

## Tests

Source matching install-cpu or install-gpu after base environment; inside workspace:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-cpu/nav2_mppi_controller --output-on-failure
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-gpu/nav2_mppi_controller --output-on-failure
```

Do not present cppcheck failure as PASS. A recheck of a fixed test only supersedes that test result, not failures elsewhere.

## Measurements

```bash
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
source /tmp/nav2-gpu-baseline-9557655b/install-gpu/setup.bash
python3 /tmp/nav2-gpu-baseline-9557655b/performance-profiler-run.py --workspace /tmp/nav2-gpu-baseline-9557655b --results /tmp/nav2-gpu-baseline-9557655b/results-separated --samples 1000 --repetitions 5
```

Runner is `tasks/TASK-001/performance-profiler-run.py`. It launches each executable with `pair <cost|obstacles> <0|1> <free|bands> 1000 5` serially. First exploratory dataset results/ stopped on fixture configuration error and is excluded. The earlier results-final/ dataset has mixed stdout/stderr corruption and is excluded as well. Final evidence lives in results-separated/ and measurements-separated.log; filenames are preserved when copied locally.

Analyze after copying final results:

```bash
python3 tasks/TASK-001/performance-profiler-analyze.py tasks/TASK-001/performance-profiler-artifacts/results-separated
```

Profiler and failure-mode commands/results are listed in the performance report after execution; profiler-instrumented runs are never pooled with timings above.

## Final checks / diagnostics (inside isolated workspace, matching overlay sourced)

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ament_cppcheck src/nav2_mppi_controller/benchmark/mppi_gpu_baseline.cpp --include_dirs src/nav2_mppi_controller/include
ament_uncrustify src/nav2_mppi_controller/benchmark/mppi_gpu_baseline.cpp
ctest --test-dir build-cpu/nav2_mppi_controller -R "uncrustify|cpplint|lint_cmake" --output-on-failure
CUDA_VISIBLE_DEVICES=-1 build-gpu/nav2_mppi_controller/test/cost_critic_parity_test --gtest_filter=CostCriticParity.CudaFlagOnMatchesCpuWhenDeviceMissing
CUDA_VISIBLE_DEVICES=-1 build-gpu/nav2_mppi_controller/test/obstacles_critic_parity_test --gtest_filter=ObstaclesCriticParity.CudaFlagOnMatchesCpuWhenDeviceMissing
CUDA_VISIBLE_DEVICES=-1 build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline pair cost 0 free 5 1
build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline cpu cost 0 free 0 1
```

Last two commands deliberately return 1 with BASELINE_ERROR (missing-device rejection / invalid-count rejection), verified by subprocess assertions. CPU-only executable pair mode also deliberately returns 1 even with visible GPU; ldd verified install-cpu library paths.

```bash
compute-sanitizer --tool memcheck --error-exitcode 99 build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline pair cost 1 bands 5 1
nsys profile --trace=cuda,osrt --sample=none --cpuctxsw=none --force-overwrite=true --output=/tmp/nav2-gpu-baseline-9557655b/nsys-cost build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline pair cost 0 bands 20 1
```

Memcheck attach timed out; diagnostic processes terminated (exit 143), no successful memory analysis. Nsight CLI exit 0 produced only qdstrm and reported missing importer; it is NOT a usable trace PASS. Host fallback command:

```bash
/opt/nvidia/nsight-systems/2024.5.1/host-linux-x64/QdstrmImporter --input-file tasks/TASK-001/performance-profiler-artifacts/nsys-cost.qdstrm --output-file tasks/TASK-001/performance-profiler-artifacts/nsys-cost.nsys-rep
```

Exited 3, TargetProfilingFailed, TimestampType not initialized. Partial output excluded from attribution.
