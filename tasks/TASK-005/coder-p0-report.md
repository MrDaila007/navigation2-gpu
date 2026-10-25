# P0 — validator loader lifetime

**Date:** 2026-10-25. **Baseline:** 57c8c32775e5b887f806e0c37af95b1a8f8c8f74, initially clean.
**Branch:** fix/mppi-validator-reconfigure-lifetime, from feat/gpu-adaptation.
**Scope:** TASK-005 P0 / R2-06 lifecycle prerequisite; R2 GPU context is not implemented by this fix.
**Environment:** existing local nav2-rolling container, GCC 15.2.0, CUDA 12.4.131, CMake 4.2.3, Linux x86_64; actual desktop RTX 3060, driver 550.163.01. CPU/CUDA native Release and CPU ASan/UBSan variants are separate coherent installs. Jetson/robot/HIL not exercised.

## Problem and resulting behavior

[VERIFIED] Optimizer::initialize previously replaced validator_loader_ while trajectory_validator_ still owned a plugin created by that loader. The real class_loader emitted its unsafe-unload warning on repeated initialization, including recovery from plugin creation failure. Existing declaration order already destroys validator instances before the loader at final optimizer destruction; no header/layout change was needed.

[VERIFIED] The loader is now created once and retained across reinitializations, as already done for motion_model_loader_. Successful create replaces the old validator while its loader remains alive. A throwing create leaves the old instance owned and permits later retry after shutdown/handler cleanup. No score math, plugin IDs/base ABI, parameters, CUDA kernels or production hot-path allocations changed. Loader/library lifetime extends until optimizer destruction; this is intentional ownership, not a leaked loader.

[VERIFIED] New optimizer regressions cover successful reinitialization, first creation failure and replacement creation failure, with three attempts per case; old weak references expire on successful replacement/destruction and remain valid after failed replacement. Noise regeneration is enabled, shutdown is repeated, and fresh ParametersHandler scopes model controller cleanup/configure. Parameter changes after handler destruction must not reset/update the optimizer. Controller regression runs three configure/activate/reset/compute/deactivate/cleanup cycles and checks callback removal both after cleanup and after controller destruction. Costmap uses real lifecycle configure/cleanup/shutdown transitions; the parent node is finalized through shutdown.

## Verification — DOD Tier 1 plus available native CUDA checks

Exact invocations: [coder-p0-commands.md](coder-p0-commands.md). Compact [source identity](coder-artifacts/source-hashes.txt), [red source identity](coder-artifacts/red-source-hashes.txt) and [build/install ELF IDs](coder-artifacts/elf-build-ids.txt). Raw logs/XML remain local; see [evidence scope](coder-artifacts/coder-evidence-scope.md).

| Criterion / P0 requirement | Outcome / evidence |
|---|---|
| Reproduce original ownership problem | PASS: coherent red OFF configure/build/install, three optimizer regressions + one controller regression fail with real unsafe-unload diagnostic (both executables exit 1) |
| Safe success/replacement/create failure | PASS: final CPU/GPU optimizer_unit_tests: 25 cases each, no skipped/failing cases, including all three new regressions and existing motion-model lifetime regressions |
| Full cleanup/reconfigure and callback lifetime | PASS: final CPU/GPU controller_state_transition_test: 2 cases each, including 3-cycle new lifecycle smoke and post-destruction parameter update |
| Configure/build/install | PASS: complete CPU/CUDA Release colcon builds after final typed-Twist correction; no compiler warnings/errors in final build logs |
| Full package/static checks | PASS: CPU 18/18 and CUDA 18/18 CTest targets; cppcheck 76 analyzed cases each, zero skipped/failing cases; cpplint/uncrustify/lint_cmake/xmllint included |
| RTX 3060 existing-controller smoke | PASS: cost/obstacles × footprint off/on × free/bands, 8 paired runs, 480 command pairs including warmup, clean teardown; no speedup claim |
| ASan/UBSan/leak focused lifetime checks | PASS: complete sanitizer build/install; 5 loader regressions + 1 three-cycle controller regression; CPU footprint/bands benchmark; detect_leaks=1, no sanitizer report |
| Build/install artifact identity | PASS: controller/critics/trajectory-validator ELF IDs match in all 3 variants; 7 staged source/build-file hashes match host |
| Diff/scope/document integrity | PASS: git diff --check; exact P0 source blast radius (3 files), Markdown/local links/shell grammar; main unchanged |
| New GPU context/concurrency/fatal policy/resource budgets | NOT RUN: P1–P6 not implemented; no claim from this P0 evidence |
| TSan/device sanitizer/Jetson/robot/distribution CI | NOT RUN: host loader ownership fix; existing tooling/system gates separate, no GPU kernel changes |

Intermediate CPU/CUDA compile failure in the new smoke used a bare {} where the generated ROS Twist constructor is explicit. Corrected to a named message without changing warnings/compiler flags; final full builds/install/tests supersede it. Container lacks rg; grep was used for diagnostic inspection. Neither failed attempt is counted as PASS.

## Boundaries, recovery and remaining work

- [VERIFIED] Controller cleanup destroys ParametersHandler after optimizer shutdown. Supported tested reconfigure uses a fresh handler per cycle and stops the previous noise thread before initialize. Callback removal remains the handler owner's responsibility, not a new shutdown registration API.
- [VERIFIED source / UNKNOWN runtime scope] Reusing the same live handler across plugin replacement can leave registered references to old plugin fields; this remains deferred. Custom validator initialize throwing after registering callbacks, arbitrary concurrent lifecycle/control use and externally retained plugin references beyond optimizer lifetime are not guaranteed by these tests.
- [VERIFIED] Revert this isolated source/test change or use the known-good CPU overlay to roll back. No clocks/power/capabilities, existing mounted source, main or prior branches changed. No header/base ABI migration needed; still rebuild/install the package coherently before testing native plugins.
- Next implementation work is P1 host context protocol/OFF backend, after the recommended self-review of P0. This coder step stops at P0; no auto-chaining into P1 or another skill.
