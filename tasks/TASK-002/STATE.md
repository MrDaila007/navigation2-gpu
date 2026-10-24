# TASK-002 State

**Goal:** fix the baseline cppcheck findings and confirmed MPPI/benchmark lifetime warnings, with CPU/CUDA verification.
**Status:** ready-for-integration (focused fix and verification complete; no commits/merge)
**Baseline revision:** 9557655b38a839b7c14e0aff862f63332f7ec7e4
**Branch:** fix/mppi-baseline-checks from feat/gpu-adaptation.
**Pre-existing dirty state:** benchmark/CMakeLists.txt modified, benchmark/mppi_gpu_baseline.cpp and specs/ untracked, ignored tasks/ from TASK-001; preserved without commits/stash/reset.
**Environment:** specs/performance-profiler-environment.md; local ROS Rolling container, CUDA 12.4/GCC 15.2, actual/target RTX 3060.

## Scope and acceptance

- Ten cppcheck uninitvar diagnostics in unchanged test fixtures: reproduce, explicitly initialize fixture State.local_path_length without changing production State semantics or test expectations.
- Optimizer motion-model loader destruction order: demonstrate failing teardown regression, fix ownership order, prove test passes.
- Baseline helper node/costmap callbacks bypass real lifecycle: configure/cleanup/shutdown through transitions, preserve numerical workload.
- CPU/CUDA full package suite with actual cppcheck enabled; no skipped static-analysis PASS.
- No scoring optimization, GPU clock/capability changes, merge/push or unrelated teardown refactor.

## Next action

Review/commit focused fixes separately from inherited baseline work, then design R2 residency contract. No automatic skill chaining or Git integration performed in this debugging step.

## Confirmed results so far

- CPU/CUDA Release configure/build/install PASS. Full CTest 18/18 in both with AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1.
- Destruction regression observed FAIL before fix. Declaration-order-only fix made destruction PASS but replacement FAIL; persistent loader fixed both.
- Sixteen fixture declarations now explicitly initialize local_path_length without changing production State API/defaults. State{} rejected by GCC explicit generated-message constructor diagnostics; that attempt discarded.
- Incoherent target-only rebuild invalid run (old install library ABI) excluded; all final builds use full colcon install with build/install ELF build-ID verification.
- PASS: ASan/UBSan focused regressions and footprint benchmark with leak detection; eight CUDA paired smokes have clean teardown and 480 matching command pairs.
- PASS: final controller/critics ELF build IDs match build/install for CPU, GPU and sanitizer variants.
- Report/commands/evidence completed; unrelated lifecycle reinitialization constraints in DEFERRED.md.
