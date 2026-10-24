# TASK-001 State

**Goal:** глобальный roadmap максимального переноса численных вычислений Nav2 на GPU.
**Status:** active (R0/R1 baseline)
**Baseline revision:** 9557655b38a839b7c14e0aff862f63332f7ec7e4
**Branch:** perf/gpu-baseline, от feat/gpu-adaptation
**Pre-existing state:** untracked specs/; local ignored AGENTS.md содержит согласованные target/branching rules. Свежие build/test failures не исследовались; предыдущие historical reports отдельно в context.
**Current phase:** exploratory controller baseline complete; R0/R1 partial, full readiness blocked by documented existing failures/target gaps.
**Last verified revision:** baseline выше для source-level inspection; новые kernels/builds не проверялись.

## Decisions / approvals

- Пользователь: максимум вычислений на GPU; desktop RTX 3060 сейчас, Jetson позже.
- Пользователь: общий integration branch feat/gpu-adaptation, отдельные focused branches.
- Предложенные stage order/architecture/budgets в roadmap не являются утверждёнными численными допусками или обещаниями speedup.

## Changed scope

- specs/writing-plans-gpu-roadmap.md — единый living roadmap.
- tasks/TASK-001/writing-plans-plan.md — requirements mapping, первая итерация и exact commands.
- specs/MANIFEST.md и repository context — ссылки на roadmap вместо дублирующего краткого плана.
- Production sources, builds, adapters/hooks и devices не менялись.

## Verification

- PASS — DOD Tier 0: git diff --check; локальный Python check: 5 документов, 10 relative links, 12 stage IDs в roadmap/plan, 13 существующих source anchors, balanced code fences; отдельная проверка trailing whitespace/final newline для untracked/ignored документов.
- PASS — git branch --show-current: feat/gpu-roadmap. Документы сохранены без commit/push; tasks/ скрыт существующим Git exclude (git check-ignore).
- Build/tests/sanitizers/target: NOT RUN; документационный scope.
- Graphify query использован как карта; package/skill mismatch и усечённый вывод ограничивают evidence, факты сверены по source.

## Blockers / unknowns

- Hardware model clarified by user on 2026-10-24: desktop/baseline RTX 3060; no model mismatch remains.
- ROS/workload/frequency/budgets и точный scope численного контракта.
- Нет подтверждённого полного CPU/GPU performance baseline.

## Next action

Начать R0/R1 на отдельной ветке perf/gpu-baseline от feat/gpu-adaptation. Сначала прочитать roadmap/plan и зафиксировать environment; пользователь авторизовал продолжение R0/R1 сообщением «ок продолжим».

## R0/R1 execution baseline

- Before source changes: HEAD 9557655b; untracked specs/ and ignored task/policy artifacts from planning.
- Container nav2-rolling mounts the existing feature worktree at /ws. Do not edit its source/build state for new work; copy only nav2_mppi_controller into isolated container /tmp workspace and reuse /ws/install dependencies.
- Verified actual GPU RTX 3060 12288 MiB, driver 550.163.01; exploratory measurements on the desktop target RTX 3060.
- Container Ubuntu 26.04.1, GCC 15.2, CUDA 12.4.131, CMake 4.2.3; image sha256:31c33f4b0373eafdd68fae9b07dcc4e340f185fe51682d5495c201d17d837f1a.
- Planned source scope: new benchmark/mppi_gpu_baseline.cpp, benchmark/CMakeLists.txt. No production scoring changes; measurement-only. Fixed-input full controller calls, backend launch verification, exact command replay, warmup + repeated per-cycle samples.

## R0/R1 results

- PASS: CPU OFF/CUDA ON Release builds; all functional GTest targets; actual missing-device fallback; benchmark CLI rejection guards; new-source cppcheck and final lint.
- PASS: final results-separated dataset: 80,000 raw measured calls, 42,000 paired command-bit comparisons incl. warmup, all expected kernel launch counts. Source/commands/raw data retained in performance-profiler-report.md and performance-profiler-artifacts/.
- FAIL: package cppcheck has ten pre-existing uninitialized local_path_length diagnostics; original test sources hash-identical. No full DoD/merge readiness claim.
- NOT RUN: memcheck attach failed; usable Nsight trace unavailable; system/Jetson verification and host sanitizers not performed.
- First footprint fixture and mixed-stream output mistakes corrected; initial datasets explicitly excluded, collector now validates numeric output per scenario.
- Next action: systematic-debugger handoff for existing test/lifetime findings; controlled target baseline and R2 contract afterward. No production scoring change, commit, merge, push or device settings changes.
