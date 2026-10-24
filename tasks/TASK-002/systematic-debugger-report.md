# MPPI baseline correctness and lifetime diagnosis

**Hardware clarification (2026-10-24):** desktop target и baseline — RTX 3060; численные результаты и сохранённые raw artifacts не изменялись.
**Baseline:** 9557655b + pre-existing measurement harness; branch fix/mppi-baseline-checks.
**Environment:** [profiling environment](../../specs/performance-profiler-environment.md); desktop target RTX 3060.
**Scope:** fix and verify pre-existing fixture cppcheck diagnostics and the confirmed optimizer/benchmark destruction warnings; no scoring/numerical algorithm changes.

## Hypotheses before production edits

1. [VERIFIED source] State::local_path_length is a scalar without an initializer; three test files construct State without value initialization. Several fixtures assign the field later, but cppcheck reports ten aggregate data construction sites before those assignments. Hypothesis: value-initializing fixture State removes diagnostics without changing production State/reset semantics or scenario-specific assignments. Existing forced cppcheck is the regression check.
2. [VERIFIED source] Optimizer declares motion_model_ before motion_model_loader_; C++ member destruction order reverses declarations. Shared reference critics_data_.motion_model is destroyed earlier but motion_model_ still exists when loader dies. Regression added: destroy actual initialized optimizer while stderr is captured, require weak model expiry and absence of unsafe unload warning. Production edit awaits observed failure.
3. [VERIFIED source] Baseline fixture calls costmap on_configure directly, so lifecycle state stays unconfigured despite allocated layers; no costmap cleanup/node shutdown occurs. Hypothesis: proper configure/cleanup/shutdown transitions remove remaining helper teardown warnings; production costmap semantics must not be changed for a benchmark fixture problem.

## Reproduction / verification

Pending commands/results appended after execution. Artifacts intentionally retained in tasks/TASK-002/systematic-debugger-artifacts/ and isolated container /tmp/nav2-mppi-baseline-fixes-9557655b.

## Confirmed before fixes

- Forced cppcheck on the three files: exit 1, ten uninitvar diagnostics (cppcheck-before.log). Binding CriticData references itself does not read the float; not all diagnostics are independently proven runtime UB. The fixtures nevertheless supplied indeterminate scalar state; value initialization makes their inputs explicit and removes this diagnostic without weakening analysis.
- Regression OptimizerTests.MotionModelLoaderOutlivesInstances: exit 1, unsafe unload diagnostic found at captured stderr position 53 (lifetime-before.log). Weak instance expired after full destruction; the defect is loader lifetime order, not a surviving external owner.
- Baseline CPU smoke: two unsafe-unload warnings and two lifecycle-not-finalized warnings (benchmark-before.stderr.log). Remaining benchmark-only warnings will be checked independently after the optimizer fix.
- Chosen production fix moves the loader declaration before its instance, leaving destructor RAII and shutdown behavior unchanged. Protected Optimizer object layout changes: rebuild all consumers; no binary compatibility promise for mixing old/new headers or libraries.

## Discriminating follow-up

- Generic aggregate State{} initialization removed cppcheck findings but failed GCC -Werror because generated ROS messages have explicit constructors. Chosen compatible fixture fix: keep ordinary construction and explicitly initialize local_path_length=0.0f; scenario-specific assignments retained. No compiler options weakened.
- Inspection found the same loader-lifetime boundary during setMotionModel replacement: loader was recreated while CriticData can still own previous model. Added replacement regression to distinguish destruction-order-only fix from full loader ownership fix. Reusing the same optimizer-owned loader is the proposed narrow correction, pending reproduced failure.

## Verification build identity correction

A target-only rebuild after protected Optimizer layout change initially executed against stale install-cpu libmppi_controller.so via LD_LIBRARY_PATH. Runtime crashed before the replacement test; this is not valid evidence about the source fix. Confirmed build/install library hashes differed (c18302ff... vs 5c53a423...). Switched to full colcon build/install and require matching build/install ELF build IDs before running tests. The invalid run is preserved as lifetime-core-only.log and excluded from regression conclusions. All consumers must be rebuilt/reinstalled after header layout changes.

## Narrowed ownership result

After coherent full build/install with declaration-order fix only: destruction regression PASS, replacement regression FAIL (unsafe unload captured). This confirms that loader recreation is a separate trigger of the same ownership defect. Chosen fix: one loader per optimizer, reused across motion-model replacements. A prior model retained by CriticData can then safely outlive the replacement assignment.

Benchmark fix uses real configure/cleanup/shutdown transitions. A guard outlives controller and handles successful or partial map initialization; no costmap production code changed. Numerical fixture inputs and timed call boundaries unchanged.

## Final changes and verification — DOD Tier 1, scoped result

| Requirement | Fix / evidence | Outcome |
|---|---|---|
| DBG-01 deterministic fixture scalar | 16 explicit local_path_length initializations in critic_manager_test.cpp, utils_test.cpp, critics_tests.cpp; scenario-specific assignments preserved | PASS: forced cppcheck 76 checks, zero failures/errors/skips in CPU and CUDA |
| DBG-02 loader outlives all owned motion models | Loader declared before instances and created once per optimizer, following existing CriticManager::loadCritics ownership pattern | PASS: both lifetime regressions; optimizer target now 22 tests |
| DBG-03 baseline helper cleanup | Configure real costmap state, destroy controller first, cleanup/shutdown costmap and node; guard handles partial initialization | PASS: 8 CPU/CUDA smoke pairs, 480 bitwise command comparisons incl. warmup, no unload/lifecycle warnings in those fixtures |
| DBG-04 supported build/test modes | Fresh isolated package build/install with existing /ws/install dependencies | PASS: CPU Release 18/18 CTest, CUDA Release 18/18 CTest; actual cppcheck enabled |
| DBG-05 memory/lifetime checks | CPU RelWithDebInfo + address/undefined sanitizers, leak detection enabled; selected real plugin regressions + footprint benchmark | PASS: two regressions and benchmark, exits 0, no ASan/UBSan/LSan errors |
| DBG-06 artifact coherence | Build/install ELF build IDs checked for controller/critics in CPU, CUDA and sanitizer variants | PASS: IDs match; final staged/local source hashes recorded |
| DBG-07 style/repository consistency | Full suite lint + git diff --check and document/source-hash checks | PASS; no compiler-check bypasses |

Exact commands: [systematic-debugger-commands.md](systematic-debugger-commands.md). Logs, test XML, source hashes, CMake caches and ELF IDs: [artifacts](systematic-debugger-artifacts/). Both valid before-fix failures and the excluded mixed-ABI crash are retained; final result never relies on that invalid run. SHA hashes can differ solely because of RPATH editing during installation, so ELF build IDs—not raw byte equality—are the final build/install coherence check.

## Scope and limits

- New assertions use real plugin lifetime/diagnostics and weak ownership; no tests were added merely to mirror explicit scalar assignments. Existing cppcheck is the durable regression check for those fixtures.
- Production numerical scoring/defaults/state-reset behavior unchanged; short command parity verifies the fixed workload but is not a new performance benchmark. Previous TASK-001 timings remain historical, source-hashed measurements.
- Initialization/model replacement/destruction occur in the existing controller/lifecycle context; no new shared mutable state or ISR work. No new locks or allocations in the steady control loop.
- Optimizer protected member layout changed. Rebuild/reinstall all direct consumers; do not mix previous headers/binaries. Public plugin/controller methods and configuration defaults retained.
- Sanitizers instrument the MPPI package, not the complete prebuilt ROS/dependency overlay. CUDA memcheck/Nsight, TSan, all legacy fixtures under sanitizer, Jetson, SIL/robot/HIL and resource/performance budgets: NOT RUN in this task; no corresponding PASS claim.
- Full cleanup/reconfigure of trajectory-validator and all third-party plugin callback lifetimes is not covered; source-level follow-up recorded in [DEFERRED.md](DEFERRED.md). Legacy helper-node warnings outside the new benchmark remain outside this focused fix.
- Actual hardware for CUDA verification: RTX 3060 12 GiB, driver 550.163.01; desktop target RTX 3060 verified for the scoped tests. Toolchain as in profiling environment; CUDA 12.4, GCC 15.2, C++20 / CUDA C++17.

## Rollback / integration

Keep fixes on fix/mppi-baseline-checks until source changes are committed/reviewed independently from inherited uncommitted baseline work. No commit, merge, push or history rewrite performed. Rollback is reverting this focused fix and rebuilding/reinstalling all package consumers; avoid mixing ABI versions. No firmware/device configuration changed.

Next workflow step: design R2 GPU residency contract, informed by baseline data; final performance acceptance still needs controlled target workload. This debugging step stops after recording the verified fix and remaining constraints.
