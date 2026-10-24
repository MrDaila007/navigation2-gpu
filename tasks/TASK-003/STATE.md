# TASK-003 State

**Goal:** R2 shared GPU context architecture and corrected desktop GPU descriptions.
**Status:** ready-for-interface-design (DOD Tier 0 passed; implementation not started)
**Date:** 2026-10-24
**Baseline revision:** 9557655b38a839b7c14e0aff862f63332f7ec7e4 + inherited uncommitted TASK-001 benchmark and TASK-002 verified fixes.
**Branch:** feat/mppi-gpu-context-design from feat/gpu-adaptation.
**Pre-existing state:** seven tracked MPPI files modified, untracked benchmark/mppi_gpu_baseline.cpp and specs/, ignored tasks and local AGENTS.md. No commits/stash/reset/merge; all retained.
**Profile equivalent:** specs/performance-profiler-environment.md (previous verified environment; no fresh inventory claimed).

## Human decisions

- 2026-10-24: current desktop/baseline card is RTX 3060. Correct prior model descriptions; Jetson remains future scope.
- Continue solution with next proposed stage: shared GPU context/residency architecture.

## Scope

- Update current descriptions/policy and clarify historical reports without changing raw measurements, XML, logs or source hashes.
- Create specs/architect-architecture.md: owner/dependencies, data validity, lifecycle, mixed CPU plugins, failure handling, compatibility, resource accounting and acceptance.
- No production code edits, new CUDA kernels or performance claims in this stage.

## Next action

Use interface-designer to fix exact descriptors/lease/status/limits, plugin visibility/compatibility and legacy diagnostic migration; then plan implementation. Do not auto-chain skills in this task.

## Verification — DOD Tier 0

- PASS: git diff --check.
- PASS: local Python check of 18 policy/spec/task Markdown documents, 35 local links, whitespace/code fences and absence of obsolete hardware model mentions.
- PASS: 10 R2 requirements, 14 architecture sections, 9 current source anchors verified.
- PASS: all eight current MPPI source/CMake hashes equal TASK-002 verified source manifest; no code changed during this design step.
- PASS: performance-profiler-analyze.py on results-separated: 80,000 raw samples and 42,000 command pairs still validate after description correction. Raw measurements and XML/log artifacts were not edited.
- NOT RUN: new build/test/sanitizer/device performance checks; no R2 implementation exists. Prior PASS is historical evidence only.

## Design decisions / remaining contract work

- Optimizer-owned resource context, small shared context target, secondary opt-in consumer interface; leave CriticData and CriticFunction ABI unchanged.
- Owner staging + generations; one map upload per stable control epoch, x/y per new batch; opaque CPU plugin barriers. Different float/double lookup semantics preserved.
- Resource caps, statuses/quiescence proof, interface visibility and old diagnostic migration must be fixed before implementation; product budgets still unknown.
- Architecture lives in specs/architect-architecture.md and is indexed in MANIFEST. Previous uncommitted baseline/fixes retained, no Git integration/commits performed.
