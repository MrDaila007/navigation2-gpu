# R2 implementation — executable command reference

[PROPOSED] Run during implementation, not performed by the planning task. Baseline procedures are from TASK-002; new target names correspond to files required by the plan. All snippets assume bash and stop on error (`set -e`). Do not accept zero selected tests. No source is copied to an external service; the existing local container is used.

## A — branch, environment, isolated staging

Host, before each focused branch (choose the planned name, shown for P0):

```bash
set -e
git status --short
git switch feat/gpu-adaptation
git switch -c fix/mppi-validator-reconfigure-lifetime
git merge-base --is-ancestor feat/gpu-adaptation HEAD
git rev-parse HEAD
docker ps --filter name=nav2-rolling --format '{{.Names}} {{.Status}}'
docker exec nav2-rolling nvidia-smi --query-gpu=name,driver_version,memory.total,utilization.gpu --format=csv
docker exec nav2-rolling bash -c 'source /opt/ros/rolling/setup.bash; g++ --version; nvcc --version; cmake --version'
```

Require clean state before switching, actual RTX 3060 and expected toolchain; differences are recorded/reconciled before claiming matched evidence. Existing second worktree is not edited. Use a fresh directory for each immutable source snapshot; before a new attempt change the suffix rather than overwriting installed artifacts.

```bash
R2_WORK=/tmp/nav2-mppi-r2-attempt-001
docker exec nav2-rolling test ! -e "$R2_WORK"
docker exec nav2-rolling mkdir -p "$R2_WORK/src"
docker cp nav2_mppi_controller "nav2-rolling:$R2_WORK/src/nav2_mppi_controller"
git rev-parse HEAD
git diff -- nav2_mppi_controller
```

Record source revision plus diff and SHA-256 list for the staged package (revision alone is insufficient for dirty implementation). Store commands/results in TASK-005; never collect .env, secrets or unrelated home files. Every following container snippet uses this exact attempt path or an explicitly recorded replacement.

## B-CPU — complete OFF configure/build/install

Run in a fresh container shell, to avoid inheriting GPU/sanitizer overlays:

```bash
set -e
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-r2-attempt-001
colcon --log-base log-cpu build --build-base build-cpu --install-base install-cpu --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install-cpu/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-cpu/nav2_mppi_controller --output-on-failure --no-tests=error
```

Inspect actual cppcheck output/XML for analysis rather than skipped-version PASS. Current baseline had 18 CTest entries; R2 adds tests, so do not hardcode 18 as expected total. OFF success on a machine with CUDA installed is not proof of a toolkit-free image. Also check installed/header dependencies (L). A genuinely toolkit-free CI/container build is required to close that platform claim; if unavailable record NOT RUN separately.

## B-GPU — complete ON configure/build/install

New shell, ROS + existing dependency overlay only:

```bash
set -e
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-r2-attempt-001
colcon --log-base log-gpu build --build-base build-gpu --install-base install-gpu --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install-gpu/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build-gpu/nav2_mppi_controller --output-on-failure --no-tests=error
```

Never use target-only build followed by loading an old install library after class layout changes. Readelf build IDs below must match build/install; skipped no-device tests on a present GPU are expected, not GPU fallback evidence.

## T — focused tests after the corresponding full build/install

Container, attempt directory and chosen overlay already sourced; use CPU first, then GPU where applicable. These future test targets must be registered via existing nav2_add_gtest patterns.

T-P0 (new GTest names specified in plan; keep existing motion-model tests too):

```bash
set -e
build-cpu/nav2_mppi_controller/test/optimizer_unit_tests --gtest_filter='OptimizerTests.ValidatorLoaderOutlivesReinitialization:OptimizerTests.MotionModelLoaderOutlives*'
build-cpu/nav2_mppi_controller/test/controller_state_transition_test --gtest_filter='ControllerLifecycle.ReconfigureGpuPrerequisite'
```

Assert that the requested tests actually execute (GoogleTest can return success on zero matching tests). Capture stderr and reject unsafe unload/unfinished lifecycle warnings in new fixtures; do not suppress them.

T-HOST:

```bash
ctest --test-dir build-cpu/nav2_mppi_controller -R '^gpu_context_test$' --output-on-failure --no-tests=error
```

T-DEVICE (RTX 3060 required):

```bash
ctest --test-dir build-gpu/nav2_mppi_controller -R '^gpu_context_cuda_test$' --output-on-failure --no-tests=error
```

T-PARITY:

```bash
ctest --test-dir build-cpu/nav2_mppi_controller -R '^(cost_critic_parity_test|obstacles_critic_parity_test)$' --output-on-failure --no-tests=error
ctest --test-dir build-gpu/nav2_mppi_controller -R '^(cost_critic_parity_test|obstacles_critic_parity_test)$' --output-on-failure --no-tests=error
```

Use separate shells for the two overlays above; never source both in one process. T-INTEGRATION in both variants, substituting the variant directory:

```bash
ctest --test-dir build-gpu/nav2_mppi_controller -R '^(gpu_context_integration_test|critic_manager_test|optimizer_unit_tests|controller_state_transition_test|parameter_handler_test)$' --output-on-failure --no-tests=error
```

Mandatory test assertions beyond exit code: healthy two-critic/two-iteration counters 1 map / 2 xy / 4 completed launches, warmed zero new payload allocations; unchanged command bits/costs/flags; explicit opaque mutation refresh; no runtime calls for all-disabled; no partial output after injected failure. Missing-device acceptance uses a test-only unavailable backend or an isolated subprocess with CUDA_VISIBLE_DEVICES empty before first runtime call; never disable/reset the actual shared device.

## S — host sanitizers

Fresh shell; only new isolated build/install roots. Existing project sanitizer helper cleans workspace directories, so do not invoke it here.

```bash
set -e
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-r2-attempt-001
MAKEFLAGS='-j2 -l2' colcon --log-base log-sanitize build --build-base build-sanitize --install-base install-sanitize --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined' '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined'
source install-sanitize/setup.bash
export ASAN_OPTIONS=detect_leaks=1:halt_on_error=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
ctest --test-dir build-sanitize/nav2_mppi_controller -R '^(gpu_context_test|gpu_context_integration_test|critic_manager_test|optimizer_unit_tests|controller_state_transition_test)$' --output-on-failure --no-tests=error
RCUTILS_LOGGING_USE_STDOUT=0 build-sanitize/nav2_mppi_controller/benchmark/mppi_gpu_baseline cpu cost 1 bands 10 1
```

P0/P1 run only test targets available at that checkpoint. Intentional fake fatal death tests must use subprocesses/EXPECT_DEATH with explicit expected termination; ordinary sanitizer reports remain failures. TSan, if needed for independent-owner/try-entry concurrency, requires its own OFF build/install with `-fsanitize=thread` instead of address/undefined, then focused gpu_context_test concurrent cases; never combine ASan and TSan. Unsupported runtime is NOT RUN, not PASS.

## L — identity, OFF dependencies, downstream plugin/relocation

For each built variant, inside its environment:

```bash
set -e
cd /tmp/nav2-mppi-r2-attempt-001
for variant in cpu gpu sanitize; do
  for lib in mppi_controller mppi_critics mppi_gpu_context; do
    readelf -n "build-$variant/nav2_mppi_controller/lib$lib.so"
    readelf -n "install-$variant/nav2_mppi_controller/lib/lib$lib.so"
  done
done
readelf -d install-cpu/nav2_mppi_controller/lib/libmppi_gpu_context.so
readelf -d install-cpu/nav2_mppi_controller/lib/libmppi_critics.so
ldd -r install-gpu/nav2_mppi_controller/lib/libmppi_gpu_context.so
ldd -r install-gpu/nav2_mppi_controller/lib/libmppi_critics.so
```

Run only completed variants; compare matching build IDs, not byte hashes after install RPATH edits. No cudart in OFF DT_NEEDED or public header transitive includes; private backend/native headers are not installed. New public header syntax must compile with C++17/C++20, using only the installed include path; link test in fixture supplies out-of-line symbols.

Old plugin test needs an immutable old overlay first. Host commands (baseline source only, no checkout/reset of working files):

```bash
set -e
docker exec nav2-rolling test ! -e /tmp/nav2-mppi-r2-legacy-b23f498a
docker exec nav2-rolling mkdir -p /tmp/nav2-mppi-r2-legacy-b23f498a/src
git archive b23f498a nav2_mppi_controller | docker exec -i nav2-rolling tar -x -C /tmp/nav2-mppi-r2-legacy-b23f498a/src
```

Build that baseline with B-CPU procedure using `/tmp/nav2-mppi-r2-legacy-b23f498a` paths. P5's fixture CMake must support `MPPI_BUILD_LEGACY_ONLY=ON` to compile only the old-interface plugin against that baseline and register its XML/ament resource. In a baseline-only shell:

```bash
set -e
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
source /tmp/nav2-mppi-r2-legacy-b23f498a/install-cpu/setup.bash
cmake -S /tmp/nav2-mppi-r2-attempt-001/src/nav2_mppi_controller/test/gpu_context_downstream -B /tmp/nav2-mppi-r2-attempt-001/build-legacy-fixture -DMPPI_BUILD_LEGACY_ONLY=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/nav2-mppi-r2-attempt-001/install-legacy-fixture
cmake --build /tmp/nav2-mppi-r2-attempt-001/build-legacy-fixture --parallel 2
cmake --install /tmp/nav2-mppi-r2-attempt-001/build-legacy-fixture
```

New shell: source ROS, dependencies, **new GPU overlay**, then legacy fixture prefix (without reintroducing old MPPI overlay). Fixture must not carry old MPPI absolute RPATH. `gpu_context_integration_test` P5 downstream case loads `mppi_gpu_downstream/LegacyCpuCritic` via registered XML, checks expected score/barrier, and verifies loaded core/critics are the new install via ldd/readelf evidence. Its test name is `GpuContextIntegration.LegacyBinaryCpuPlugin`.

```bash
set -e
export AMENT_PREFIX_PATH="/tmp/nav2-mppi-r2-attempt-001/install-legacy-fixture:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="/tmp/nav2-mppi-r2-attempt-001/install-legacy-fixture/lib:$LD_LIBRARY_PATH"
cd /tmp/nav2-mppi-r2-attempt-001
build-gpu/nav2_mppi_controller/test/gpu_context_integration_test --gtest_filter=GpuContextIntegration.LegacyBinaryCpuPlugin
cmake -S src/nav2_mppi_controller/test/gpu_context_downstream -B build-downstream -DMPPI_BUILD_LEGACY_ONLY=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/nav2-mppi-r2-attempt-001/install-downstream
cmake --build build-downstream --parallel 2
cmake --install build-downstream
```

The fixture CMake uses ament_package/pluginlib resource registration, but direct cmake --install does not supply colcon's workspace setup.bash. The explicit fixture-prefix exports above avoid that assumption. Legacy-only install must not add an old MPPI prefix/RPATH.

Relocation, fresh shell, preserve dependency overlay but never source the old/unmoved R2 overlay. Proposed fixture executable name is `mppi_gpu_downstream_consumer`, installed in lib/mppi_gpu_downstream. Fixture has no absolute build-tree include/library paths and new consumer links the exported context/controller targets.

```bash
set -e
source /opt/ros/rolling/setup.bash
source /ws/install/setup.bash
cd /tmp/nav2-mppi-r2-attempt-001
test ! -e relocated-gpu-prefix
mkdir relocated-gpu-prefix
cp -a install-gpu/nav2_mppi_controller/. relocated-gpu-prefix/
export CMAKE_PREFIX_PATH="$PWD/relocated-gpu-prefix:$CMAKE_PREFIX_PATH"
export AMENT_PREFIX_PATH="$PWD/relocated-gpu-prefix:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/relocated-gpu-prefix/lib:$LD_LIBRARY_PATH"
cmake -S src/nav2_mppi_controller/test/gpu_context_downstream -B build-downstream-relocated -DMPPI_BUILD_LEGACY_ONLY=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/install-downstream-relocated"
cmake --build build-downstream-relocated --parallel 2
cmake --install build-downstream-relocated
ldd -r install-downstream-relocated/lib/mppi_gpu_downstream/mppi_gpu_downstream_consumer
install-downstream-relocated/lib/mppi_gpu_downstream/mppi_gpu_downstream_consumer
```

Require ldd/loaded-symbol evidence points to relocated R2 libraries, not /ws/install MPPI or the old install-gpu prefix; merely launching without checking resolution does not pass relocation. A toolkit-free run and distribution CI matrix remain separate gates.

## D — optional device tooling, mandatory honest status

Only valid kernels on the RTX 3060:

```bash
compute-sanitizer --tool memcheck --error-exitcode 99 build-gpu/nav2_mppi_controller/test/gpu_context_cuda_test
```

Previous memcheck attach/Nsight limitations are unresolved. Tool attach failure is NOT RUN with error text; no container privilege/driver changes or GPU resets in this plan. Fake faults cover fatal cleanup logic independently, not real GPU fault recovery.

## M — measurements and final evidence

Fresh GPU overlay, stable declared timing mode, unchanged clock/governor/settings and unrelated workloads. For each source snapshot retain inventory, source hashes, ELF IDs, command, stdout/stderr separately:

```bash
set -e
cd /tmp/nav2-mppi-r2-attempt-001
mkdir -p measurements
for critic in cost obstacles; do
  for footprint in 0 1; do
    for scene in free bands; do
      RCUTILS_LOGGING_USE_STDOUT=0 build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline pair "$critic" "$footprint" "$scene" 1000 5 > "measurements/$critic-fp$footprint-$scene.log" 2> "measurements/$critic-fp$footprint-$scene.stderr.log"
    done
  done
done
```

Host: copy numeric results to the task-specific artifact directory; use existing analyzer only if original schema preserved:

```bash
python3 tasks/TASK-001/performance-profiler-analyze.py tasks/TASK-005/performance-profiler-artifacts/results-r2
```

For stats/both extension P6 must create exact repo-relative files `tasks/TASK-005/performance-profiler-run.py` and `tasks/TASK-005/performance-profiler-analyze.py`, documenting versioned records and both-case expectations before collection. Apply identical measurement-only harness to baseline and R2 snapshots; record patch hashes. Existing source comparison alone does not implement a baseline `both` scenario. Do not accept launches without command parity/counter freshness. Full controlled performance/deadline acceptance remains unresolved if desktop conditions cannot be controlled within authorized scope.

Final implementation report: requirement matrix P0–P6, exact commands/exit codes, PASS/FAIL/NOT RUN including skipped tests/tools, source+artifact identity, raw numeric provenance, resource peaks, unresolved constraints and rollback. Integrate focused branch after applicable DOD checks; push only named authorized branches and verify origin SHAs. No merge into main, force push, branch deletion, reset/rebase or hardware configuration changes.
