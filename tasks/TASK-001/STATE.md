# TASK-001 State

**Goal:** глобальный roadmap максимального переноса численных вычислений Nav2 на GPU.
**Status:** roadmap complete; R0/R1 planned
**Baseline revision:** 9557655b38a839b7c14e0aff862f63332f7ec7e4
**Branch:** perf/gpu-baseline, от feat/gpu-adaptation
**Pre-existing state:** untracked specs/; local ignored AGENTS.md содержит согласованные target/branching rules. Свежие build/test failures не исследовались; предыдущие historical reports отдельно в context.
**Current phase:** roadmap/documentation complete; baseline next.
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
