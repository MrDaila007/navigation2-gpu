# R2 shared GPU context — implementation plan

**Task:** TASK-005. **Date:** 2026-10-24.
**Host:** Linux x86_64; existing ROS Rolling container, GCC 15.2/CUDA 12.4, CMake/ament/colcon, GoogleTest. Host GCC 11.4/ROS Humble is a different environment.
**Target:** desktop NVIDIA RTX 3060, architecture 86. Jetson/robot/HIL outside this stage.
**Profiles:** [environment](../../specs/performance-profiler-environment.md), [architecture](../../specs/architect-architecture.md), [v1 contract](../../specs/interface-designer-contracts.md).
**Goal:** one optimizer-owned input/workspace engine used by both CUDA critics, without duplicate map/x-y uploads for the same valid generations; numerical/controller compatibility and safe failure paths.
**Constraints:** core C++20, CUDA boundary C++17; CPU default/OFF preserved; unknown plugins remain functional; no device reset, destructive fault kernels, new GPU scoring math or rollout.
**Evidence required:** host protocol/fake faults, OFF/ON builds and full package tests, sanitizer/lifetime, downstream plugin/relocation, RTX 3060 parity/upload/resource measurements; deadline acceptance separately unresolved.
**Baseline revision/state:** b23f498a9bcb40e4097a280ae376f30917578a46; clean; TASK-002 recorded 18/18 CPU/CUDA and focused sanitizers for its source hashes. R2 has no implementation/test PASS yet.

[VERIFIED] Plan facts are traced to paths/symbols below, package CMake and package.xml, nav2_common/cmake/nav2_package.cmake, and .github/workflows/build_main_against_distros.yml (Jazzy/Lyrical build matrix, no GPU verification). No package-specific preset/toolchain/linker script was found in the inspected package/common/CI paths. Commands adapt the verified isolated TASK-002 procedure; their R2 results are future evidence.

[PROPOSED] All P0–P6 implementation steps/commit names are plans. Contract choices, especially fatal fail-stop policy and default caps, remain subject to implementation review; they are not approved product resource/deadline budgets.

## Close existing patterns

1. `nav2_mppi_controller/src/critics/cost_critic_cuda.cu`: `CostCriticCuda::DeviceState`, `grow_n`, score/gather kernels; obstacles equivalent in `obstacles_critic_cuda.cu`. Reuse numerical kernels and column-major inputs. Deliberately replace duplicated allocation/copy/drain handling; do not copy `fail_after_d2h`'s unchecked synchronize into the new engine.
2. `nav2_mppi_controller/src/critic_manager.cpp`: `loadCritics`, sequential `evalTrajectoriesScores`; `test/critic_manager_test.cpp`: `DummyCritic`, `CriticManagerWrapper`. Extend secondary-interface discovery and fake plugin sequence tests while preserving ordering, fail early exit and visualization statistics.
3. `nav2_mppi_controller/test/optimizer_unit_tests.cpp`: `OptimizerTester` and motion-model loader lifetime regressions; `src/optimizer.cpp`: persistent `motion_model_loader_`. Reuse regression style for validator and repeat-configure tests; rebuild/install the whole package after header layout changes.

All paths in work items below are relative to **nav2_mppi_controller/** unless explicitly marked repo-relative. New test names/files are proposed, not existing commands that already pass.

## Scope and wiring decisions for implementation

- Install public `tools/gpu_context.hpp` and `critic_gpu_interface.hpp`, adapted from declaration-only TASK-004 prototypes, not by including task headers. Define interface destructors/key functions once in `src/gpu_context.cpp` so RTTI identity is shared.
- `mppi_gpu_context` is one exported shared library. Host backend interface is package-private `src/gpu_context_backend.hpp`; native lease/view is `src/gpu_context_device.hpp`. Both controller and critics link the same library; no controller → critics dependency.
- Keep inline Optimizer destructor and complete pure host facade include. Declare `unique_ptr<gpu::Context>` before `critic_manager_`; manager observers detach before context close and on partial configure failure.
- Bridge the exact v1 attach-only plugin seam without adding virtuals: private `detail::Access::current_batch(Context &, BatchToken &)` declared in backend header/defined in context.cpp exposes the current validated batch only to package helpers. Managed helpers acquire that token before native::acquire. Never infer freshness from incoming pointer equality. This is an internal implementation detail, not a new installed API.
- Manager holds borrowed Context + current ControlToken/BatchToken, populated by explicit nonvirtual integration methods. Before an opaque score invalidate; only the next requesting GPU score calls refresh_inputs using current CriticData and costmap metadata under the existing lock. A stale map persists across batch boundaries until refresh; no unconditional refresh after CPU plugins.
- begin_control snapshots the map, begin_batch snapshots x/y as v1 requires. H2D remains lazy. If no enabled critic requests GPU, skip the entire GPU scope and all explicit CUDA calls. Do not implement the architecture's earlier wording as repeated host snapshots in every helper: v1 is authoritative for exact method semantics.
- On optimizer retry/reset, end/invalidate the prior scope before resetting state; restart a fresh control scope under the same controller lock. Each generation gets a new batch token even if iteration becomes zero. Scope guard finishes normal/exception exits before lock release. Fatal/unproven drain prevents command return and cannot be swallowed by a noexcept guard.
- Do not widen the runtime plugin ABI: CriticData and CriticFunction files remain unchanged. Derived built-ins/Optimizer layout changes require coherent full rebuild; old unmarked binary CPU plugin is a separately tested compatibility boundary.
- New immutable ROS caps must use read-only parameter descriptors. `ParameterType::Static` alone is not proof that runtime changes are rejected: test rejection through ROS set_parameters as well as configure validation. Existing `use_cuda=false` stays unchanged.

## Requirements to evidence

| Architecture ID | Work | Required signal |
|---|---|---|
| R2-01 owner/isolation | P1–P4 | two critics share buffers; two optimizers have separate counters/workspaces |
| R2-02 freshness | P1/P4 | changed content at same pointer, resize/origin/resolution, retry, stale/foreign tokens |
| R2-03 plugins | P4/P5 | unchanged base headers, marker audit, mutating opaque plugin order, old binary plugin load |
| R2-04 numerical behavior | P3/P5 | existing score/flags tests; float/double boundaries, stride/footprint, command bit comparison |
| R2-05 atomic fallback | P2/P3/P5 | sentinel host outputs, fake recoverable failures, current-critic recompute exactly once |
| R2-06 lifecycle | P0–P5 | repeated configure/shutdown, partial failure, leases, callbacks, sanitizer and fake fatal death tests |
| R2-07 OFF/opt-in | P1/P4/P5 | OFF dependency isolation, ON with use_cuda=false zero runtime operations, no-device fallback |
| R2-08 observability | P2/P6 | instance counters, matched full wall/CPU comparisons and raw sample identities |
| R2-09 limits | P1/P2/P5 | checked arithmetic, exact caps including replacement peak, warmed no-allocation |
| R2-10 reproducible identity | P5/P6 | source/diff hashes, flags, image/overlay identity, raw samples and ELF build IDs; target evidence distinguished |

## P0 — close validator lifecycle prerequisite

**Purpose/IDs:** R2-06; prerequisite inherited from [TASK-002 deferred](../TASK-002/DEFERRED.md).
**Modify:** `src/optimizer.cpp`, `include/nav2_mppi_controller/optimizer.hpp` only if needed for safe declaration/teardown order; `test/optimizer_unit_tests.cpp`, `test/controller_state_transition_test.cpp`.
**Symbols:** Optimizer::initialize/shutdown/reset, validator_loader_, trajectory_validator_, parameter reset callback, MPPIController::cleanup.

1. Add `OptimizerTests.ValidatorLoaderOutlivesReinitialization` and `ControllerLifecycle.ReconfigureGpuPrerequisite` using real lifecycle transitions and a live old validator through replacement. Confirm the unsafe-unload signal or failing lifetime assertion against baseline first; if not reproduced, record that and determine ownership from source rather than fabricating a red test.
2. Retain the validator loader across replacement as appropriate, destroy old plugin before any loader replacement, and cover partial initialization failure. Never reorder unrelated motion-model logic. Verify callback targets remain live until ParametersHandler is destroyed; no reset callback after shutdown/owner destruction.
3. Use fresh handler per controller configure lifecycle, and separately test same optimizer initialize replacement. If safe same-handler reinitialization needs a new registration/removal API outside this file set, stop P0 and revise the plan before adding it.

**Ownership/thread:** lifecycle thread, serialized against control; loader outlives instances; no GPU resources yet. **Compatibility/blast radius:** MPPI configure/reconfigure only; header changes require all consumers rebuilt. **Commands:** commands B-CPU, T-P0, S below. **Expected success:** no plugin unload/lifecycle warning in new tests, no callback UAF, normal existing lifetime tests pass. **Checkpoint/commit:** `fix/mppi-validator-reconfigure-lifetime`; record regression, hashes and full install IDs in STATE. Integrate this focused fix only after applicable DOD; rollback is that isolated commit with a coherent rebuild.

## P1 — host protocol, OFF backend and test seam

**Purpose/IDs:** R2-01/02/06/07/09.
**Create:** `include/nav2_mppi_controller/tools/gpu_context.hpp`, `include/nav2_mppi_controller/critic_gpu_interface.hpp`, `src/gpu_context.cpp`, `src/gpu_context_backend.hpp`, `src/gpu_context_stub.cpp`, `test/gpu_context_test.cpp`, `test/utils/gpu_context_fake_backend.hpp`, `test/utils/gpu_context_fake_backend.cpp`.
**Modify:** package `CMakeLists.txt`, `test/CMakeLists.txt`.

1. Implement v1 descriptors/status/tokens, weak identity, checked counts/products/sums, finite domain validation and one-slot state machine. Constructor/noexcept allocation and partial configure failure must return valid status, not throw across facade. Preserve last stats on close and invalidate identity on reconfigure/overflow.
2. Define private backend operations for setup, reserve/copies/completion/drain/close and test factory injection. Build fake-backed context in a separate test library using the same host source, never link duplicate Context/RTTI symbols into a production DSO/process. No global fake switch or mutable singleton.
3. OFF stub returns NotCompiled, no CUDA includes/runtime. Test snapshots outlive input scope, wrong/expired/same-address owners, stale result/lease, zero-size/mismatched/nonfinite input, cap boundary and sequence overflow via private test access, ConcurrentUse and saturation. Assert status out parameters are invalidated correctly.
4. Add/export/install `mppi_gpu_context`, controller PUBLIC link (installed inline destructor), critics link to the same target. Move CUDA option/language/flag scoping before the new target definition; select one backend source only. C++17 boundary headers use no Eigen/ROS/CUDA. ON backend arrives in P2; P1 builds with OFF without enabling CUDA and keeps existing OFF tests functional.

**Ownership/thread:** one optimizer owner later; entry guard rejects concurrent access; no background workers/ISR. **ABI:** new headers/library; no CriticData/CriticFunction edits. **Commands:** B-CPU, T-HOST, S; missing new target/undefined methods are expected before implementation. **Evidence:** protocol tests exercise fake allocations/errors without a device, OFF DSOs have no cudart dependency. **Checkpoint/commit:** `feat/mppi-gpu-context-core`; resume from recorded OFF suite and exported-target smoke, not from header syntax alone. **Rollback/blast radius:** remove new target/links with its commit before consumer integration; do not leave installed stale exports.

## P2 — CUDA resource engine and failure proof

**Purpose/IDs:** R2-01/05/06/08/09.
**Create:** `src/gpu_context_cuda.cu`, `src/gpu_context_device.hpp`, `test/gpu_context_cuda_test.cpp`.
**Modify:** `src/gpu_context.cpp`, `src/gpu_context_backend.hpp`, package/test CMake and fake backend/tests from P1.

1. Lazy device-0 setup, stream/events/buffers and owned staging. Select configured device on each CUDA entry independent of lifecycle/control OS thread. No driver context/device reset. Reserve exact capacities, account peak replacement allocation and retain warmed buffers; include staging/result/collision/scalar payload in caps.
2. Implement acquire/complete/abandon and explicit synchronization success before host publication/free. Only owner buffers may be asynchronous copy targets. Handle H2D/D2H/launch/partial setup failures, and distinguish capability/OOM/drained failure from fatal/unproven completion per v1. Complete consumes lease on every path.
3. Review fatal/destructor policy BEFORE wiring consumers; record review in task STATE. Fake tests model illegal address, bad drain and live-lease destruction, proving retained storage/no early free and expected death behavior. Never trigger actual illegal access/timeout/assert on the shared desktop GPU. Event-timing failure after data completion only invalidates timings.
4. Add saturating per-instance counters, enqueued vs completed counts, allocation/capacity peaks and sync wait. Measure timing mode separately. Test same shape warmup then zero new allocations, one-map/two-batch protocol, growth/shrink/cap/OOM behavior and reconfigure device re-probe.

**Ownership/thread:** all DMA storage owner-held until quiescence; no concurrent destruction allowed. **ABI/blast radius:** new context backend only, existing helper buffers unchanged until P3. **Commands:** B-CPU/B-GPU, T-HOST/T-DEVICE, S, optional D. **Expected red:** no ON backend/native symbols before P2; fake post-launch failure must not publish sentinel output. **Checkpoint/commit:** `feat/mppi-gpu-context-cuda-backend`; fake and real device evidence are separate. **Rollback:** OFF build/previous artifacts; no GPU reset or driver changes.

## P3 — migrate both helpers while preserving standalone behavior

**Purpose/IDs:** R2-01/04/05/07/08.
**Modify:** `include/nav2_mppi_controller/critics/cost_critic_cuda.hpp`, `include/nav2_mppi_controller/critics/obstacles_critic_cuda.hpp`; `src/critics/cost_critic_cuda.cu`, `src/critics/obstacles_critic_cuda.cu`, `src/critics/cost_critic_cuda_stub.cpp`, `src/critics/obstacles_critic_cuda_stub.cpp`; both existing `test/*critic_parity_test.cpp`.

1. Delegate buffers/stream to the engine; keep score/gather signatures, kernel arithmetic/ordering, float Cost lookup vs double Obstacle lookup, Cost step>T and legacy zero-row score path. Use own context for never-attached standalone helpers, explicit managed/closed state after detach.
2. Add attach/detach, last_status/read_stats and internal current-batch access; managed calls consume existing snapshots/tokens rather than recopy caller x/y. No hidden standalone recreation after shutdown.
3. Read owned completed result, copy/consume host values and release lease. Failed GPU operation leaves user outputs/partial costs untouched. Existing critic CPU fallback must inspect fatal status before using legacy boolean false; adapters are completed together with P4 before integration of managed execution.
4. Migrate legacy counters/timings to documented thread_local diagnostics, preserve symbols/reset behavior/sequential tests, add two-owner/two-thread independence tests. All tests/bench acceptance will use per-instance stats (P6).

**Ownership/thread:** standalone helper owns engine; managed helper borrows, no cached device pointers. **ABI/blast radius:** experimental helper implementation/derived layout rebuild; legacy sequential diagnostics explicitly change threading semantics per v1. **Commands:** B-CPU/B-GPU, T-PARITY and T-HOST; existing tests first on baseline then after refactor. **Checkpoint/commit:** `refactor/mppi-critic-gpu-resources` commit on a `feat/` branch; compare old/new costs, flags and output shapes. **Rollback:** revert helper migration with full build/install; never mix old installed DSOs with new headers.

## P4 — optimizer/manager scopes and audited marker adoption

**Purpose/IDs:** R2-01/02/03/05/06/07.
**Modify:** `include/nav2_mppi_controller/optimizer.hpp`, `src/optimizer.cpp`, `include/nav2_mppi_controller/critic_manager.hpp`, `src/critic_manager.cpp`; `include/nav2_mppi_controller/critics/{cost_critic,obstacles_critic}.hpp`, matching cpp; `test/critic_manager_test.cpp`, `test/optimizer_unit_tests.cpp`, `test/controller_state_transition_test.cpp`, `test/parameter_handler_test.cpp`.
**Additional exact marker candidates:** headers under `include/nav2_mppi_controller/critics/`: `constraint_critic.hpp`, `goal_critic.hpp`, `goal_angle_critic.hpp`, `path_align_critic.hpp`, `path_follow_critic.hpp`, `path_angle_critic.hpp`, `prefer_forward_critic.hpp`, `mecanum_critic.hpp`, `omni_constraint_critic.hpp`, `twirling_critic.hpp`, `velocity_deadband_critic.hpp`. Their cpp files are read for audit; modify only when declaration include/inheritance requires it, no score math changes.

1. Audit every candidate score and called utility for writes to costmap bytes/geometry or trajectories x/y/shape; document symbols/results in task evidence. Add ReadOnlyInputsV1 only after that audit. Any uncertain plugin remains opaque; markers are not a class-name allowlist.
2. GPU critics implement CriticGpuV1; manager discovers interfaces once after plugin configure, version checks, binds observer, detaches before reload/destruction/partial failure. Default-disabled consumers still execute the existing CPU score. No user plugin calls outlive owner.
3. Wire begin/end cycle, fresh batch after every generateNoisedTrajectories, private current token, opaque invalidate/lazy refresh and reset/retry exceptions as specified above. Preserve visualization effects, fail_flag early break and critic ordering.
4. Add immutable bounded parameters with integer validation before narrowing and read-only descriptors. Test runtime set rejection, negative/zero/huge values, existing use_cuda dynamics if currently supported, all-disabled zero runtime queries.
5. Tests: GPU → mutating unmarked CPU → GPU sees changed map/x-y; consecutive opaque CPU plugins cause one lazy refresh, opaque last causes no refresh; disabled/unknown/version-mismatch plugin still scored. Two iterations/two GPU consumers: map_uploads=1, xy_uploads=2, completed_launches=4 only for healthy fully marked no-failure sequence. Test reset/retry new epochs, no double-counting with preseeded costs/collisions, thrown plugin cleans leases before rethrow.

**Ownership/thread:** existing parameter-then-costmap lock order; serialized lifecycle/control only, context lifetime before manager; no extra global mutex/background work. **ABI/blast radius:** full controller/critics/native consumers rebuilt, downstream old CPU plugin unchanged and tested P5. **Commands:** B-CPU/B-GPU, T-INTEGRATION/S; new mutation/counter tests should fail with duplicate/stale-input baseline or missing interfaces. **Checkpoint/commit:** `feat/mppi-shared-gpu-inputs`; do not claim R2 residency from standalone parity tests alone. **Rollback:** use_cuda=false or coherent previous build, then revert integrated consumer commits together.

## P5 — acceptance, downstream compatibility and packaging

**Purpose/IDs:** all correctness/lifecycle IDs, R2-10 target evidence.
**Create:** `test/gpu_context_integration_test.cpp`, `test/gpu_context_downstream/CMakeLists.txt`, `test/gpu_context_downstream/package.xml`, `test/gpu_context_downstream/consumer.cpp`, `test/gpu_context_downstream/legacy_cpu_critic.cpp`, `test/gpu_context_downstream/legacy_critics.xml`; wire test CMake only where appropriate.
**Modify:** tests from P0–P4, package CMake export/install if checks expose missing dependencies. Do not change production package.xml unless an actually required dependency is identified; CUDA stays optional CMake dependency.

1. Build old CPU plugin fixture against baseline b23f498a installed headers/library before replacing overlay, retain that binary; plugin implements only unchanged CriticFunction/score, modifies a known test value. Load it against new installed library in separate subprocess, assert score/order/opaque barrier and no unload warning. A rebuilt new plugin alone is not this ABI check.
2. Fresh downstream executable includes installed context/interface/Optimizer headers, links exported targets via find_package, no source-tree include paths. Relocate a staged install prefix and run with proper dependency prefix; inspect readelf dynamic dependencies and ldd -r for missing symbols/single context library. Never copy private native headers into public include install.
3. Complete precision/stride/near-goal/unknown/off-map/footprint/collision/command tests, missing-device path, two controller instances, reset/cleanup/reconfigure and partial-init failures. Full CTest CPU/CUDA with actual cppcheck; inspect XML skipped counts rather than treating a skipped linter/device test as PASS.
4. ASan/UBSan/leak focused suite includes context/fake state machine, manager/lifetime and controller smoke; TSan host/fake concurrency may run as a separate build if supported. Device memcheck/tool attach failure is NOT RUN; do not change container privileges or poison GPU to force evidence. CI Jazzy/Lyrical and toolkit-free build remain distinct from local Rolling target results.

**Commands:** B/T/S/L/D in command companion. **Expected evidence:** reproducible install identity/source hashes, no unresolved relocations, old plugin binary runs, exact skipped/unavailable checks reported. **Checkpoint/commit:** `test/mppi-gpu-context-acceptance` commit on a `feat/` branch. **Rollback/blast radius:** installed MPPI libraries/plugins and exports; retain b23f498a source and known-good CPU overlay. Do not integrate on broken ownership, parity, stale-input or fallback behavior.

## P6 — matched measurements and R2 closeout

**Purpose/IDs:** R2-08/09/10; no promised speedup.
**Modify:** `benchmark/mppi_gpu_baseline.cpp`, `benchmark/cost_critic_cuda_benchmark.cpp`, `benchmark/obstacles_critic_cuda_benchmark.cpp`; repo-relative TASK-005 evidence/report and relevant specs/MANIFEST entries.

1. Expose read-only optimizer instance stats via existing benchmark/test wrapper patterns, not new ROS services. Replace global launch assertions with per-instance stats deltas; migrate microbench timing reads. Preserve stdout/stderr separation and strict numeric collector validation.
2. Keep existing single-critic CLI/workload, add explicit synthetic `both` scenario only for shared upload evidence. Old analyzer understands 8 scenarios only: keep their schema compatible or add a producer-prefixed TASK-005 collector/analyzer with explicit version and both-case checks. Never silently parse `both` as one of the old critics; do not recommend both as product scoring configuration.
3. Compare baseline b23f498a and R2 in isolated coherent overlays, identical toolchain, caps/timing modes, scenes and alternating CPU/GPU order, 1000 samples × 5 repeats × 8 scenarios. Baseline has independent contexts; synthetic both-case needs the same added harness on both revisions without production changes. Report fixture patch/source hashes. Record host/device allocation peaks, upload bytes, total wall p50/p95/p99/max and process CPU. Do not substitute event sums for wall latency.
4. Require healthy fully marked sharing counts and zero warmed new allocations; investigate every latency regression rather than declaring speedup from fewer copies. Performance budgets/background GPU load are unknown, so report exploratory changes and explicitly withhold deadline/system acceptance. Preserve raw numeric samples/source/ELF identity; bulky traces remain local.

**Commands:** M in companion; full checks once after final changes. **Checkpoint/commit:** `perf/mppi-shared-input-baseline`; update architecture status only for implemented/tested requirements. **Rollback/blast radius:** benchmark/report only; runtime revert remains coherent previous overlay or CPU setting.

## Branches, checkpoints and stop conditions

[VERIFIED human decision] Every new implementation branch starts from current `feat/gpu-adaptation`; preserve existing branches/main. Suggested sequence: P0 `fix/mppi-validator-reconfigure-lifetime`, P1 `feat/mppi-gpu-context-core`, P2 `feat/mppi-gpu-context-cuda`, P3–P4 one coherent `feat/mppi-shared-gpu-inputs` branch with separate commits, P5 `feat/mppi-gpu-context-acceptance`, P6 `perf/mppi-shared-input-baseline`. Integrate applicable DOD-passing checkpoints into the shared branch before creating the next. No partial managed path is ready merely because it compiles.

A fresh agent reads STATE, this plan, v1 and the last checkpoint report; verifies branch ancestry/clean state; reruns focused acceptance for changed behavior, full suite when required. Retain source hashes and actual commands/results; never substitute TASK-002 historical PASS. Scope changes/falsified assumptions pause the affected item for plan revision; independent planned work may continue. Invalid commands/tool failures are fixed or recorded NOT RUN, not skipped silently.

[UNKNOWN] Full Nav2 profile, approved numerical budgets for future R3–R5, deadline/resource product acceptance, complete dependency-overlay provenance, GPU CI, Jetson/robot/HIL and supervisor restart orchestration remain outside R2 completion. See [DEFERRED](DEFERRED.md).

## Completion boundary for this planning step

DOD Tier 0 only: plan/commands cover R2-01…10, exact paths/symbols/patterns, proposed vs verified facts, rollback and remaining constraints; MANIFEST points to this plan. No source implementation/build/device tests run here. Runtime readiness requires later P0–P6 evidence; execution entry is `coder` P0 (or `using-git-worktrees` for isolation), not automatic skill chaining.
