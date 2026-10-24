# TASK-004 State

**Goal:** exact R2 native C++/CUDA interface contract, following architect-architecture.md.
**Status:** complete (interface design only; production implementation not started)
**Date:** 2026-10-24
**Baseline revision:** 9557655b38a839b7c14e0aff862f63332f7ec7e4 + inherited uncommitted TASK-001 benchmark and verified TASK-002 fixes.
**Branch:** feat/mppi-gpu-context-contracts from feat/gpu-adaptation.
**Pre-existing state:** seven modified tracked MPPI files, untracked baseline .cpp/specs, ignored task/policy artifacts; preserved without stash/commit/merge.
**Profile:** specs/performance-profiler-environment.md, desktop RTX 3060, Linux/ROS Rolling, C++20 and CUDA C++17 bridge.

## Scope

- Exact descriptors, owner/generation tokens, synchronous result lease and native CUDA bridge.
- Explicit limits/defaults, status/fallback/fatal cleanup behavior, opaque plugin barriers and diagnostic migration.
- Documentation/prototype syntax only: no production implementation, CUDA launch or fresh runtime PASS claim.
- CUDA 12.4.1 archived primary documentation checked for async error/completion rules; no repository material uploaded.

## Next action

Prepare an implementation plan with writing-plans for R2 v1; no automatic implementation or skill chaining in this step.

## Contract decisions

- Context-owned staging, opaque weak-identity tokens (no ABA / resource retention), capacity-one native/result lease.
- Default payload caps 256 MiB host/device, 16M map cells; configurable immutable controller namespace parameters, not product resource budgets.
- Secondary GPU and read-only-input interfaces keep CriticData/base score ABI; opaque CPU barriers refresh lazily.
- Per-instance stats; deprecated legacy counters/timings retain symbols but become calling-thread diagnostics with explicit migration.
- Fatal/unproven DMA cannot be safe CPU fallback; proposed close/destructor fail-stop policy documented, supervisor integration outside context.
- Declaration-only syntax checks: host C++17/C++20, native bridge C++17 against installed CUDA headers, all with warnings as errors. No link/runtime test claim.

## User steering: publish completed work

- User explicitly requested pushing completed project branches after this step.
- origin verified: git@github.com-mrdaila007:MrDaila007/navigation2-gpu.git; main unchanged.
- Will split accumulated work into scoped commits/checkpoints and publish named branches without force/history rewrite. Large raw tool traces, stderr, caches, binaries and core dumps stay local; share reproducible docs, code, sanitized numeric samples/source identity only.

## Verification — DOD Tier 0

- PASS: C++17/C++20 host declarations and C++17 native bridge syntax with g++ -Wall -Wextra -Werror -pedantic -x c++ -fsyntax-only; exact paths below in interface-designer-verification.md.
- PASS: document links, naming, whitespace and code fences; all eight source hashes match TASK-002 verified source manifest.
- PASS: architecture and MANIFEST link the proposed v1 contract; requirements/test vectors distinguish future implementation acceptance from current syntax evidence.
- NOT RUN: linking, CUDA context implementation, runtime/fault injection, new performance/robot/Jetson checks; declarations only.
- Publication is separately authorized by the user; scoped commits preserve earlier source evidence and main/history.
