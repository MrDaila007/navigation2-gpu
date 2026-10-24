# Project: Navigation2 — optional NVIDIA GPU support

[VERIFIED] ROS 2 Navigation2. CUDA-работа касается CostCritic/ObstaclesCritic внутри MPPI и находится в feature branches.

[VERIFIED — human clarification, 2026-10-24] Текущий target — desktop NVIDIA RTX 3060. Jetson — будущая адаптация; конкретная модель пока не выбрана.

## Specs Index

| File | Purpose | Depends On | Last Updated | Verified Revision |
|---|---|---|---|---|
| [repository-doc-context.md](repository-doc-context.md) | GPU status, topology, build/test evidence, unknowns, roadmap input | source/branches/worktree logs; отдельный profile отсутствует | 2026-10-23 | main 94bace6f; GPU 9557655b |
| [writing-plans-gpu-roadmap.md](writing-plans-gpu-roadmap.md) | Глобальные этапы максимального переноса вычислений на GPU, зависимости и exit gates | repository context, source inspection, human decisions | 2026-10-23 | 9557655b |
| [performance-profiler-environment.md](performance-profiler-environment.md) | Проверенная среда и measurement contract текущего desktop baseline | source, container inventory, human decisions | 2026-10-23 | 9557655b + measurement harness |
| [architect-architecture.md](architect-architecture.md) | R2: shared GPU context, ownership/epochs, plugin seam, errors, resource accounting and acceptance | baseline, TASK-002 fixes, source inspection | 2026-10-24 | 9557655b + unchanged working source |
| [interface-designer-contracts.md](interface-designer-contracts.md) | R2 v1: descriptors, leases, status/caps, plugin seam, failure policy and compatibility | architect-architecture.md + source/CUDA 12.4.1 reference | 2026-10-24 | 9557655b + unchanged working source |

## Existing GPU implementation sources

- [CostCritic CUDA implementation](../nav2_mppi_controller/src/critics/cost_critic_cuda.cu)
- [ObstaclesCritic CUDA implementation](../nav2_mppi_controller/src/critics/obstacles_critic_cuda.cu)

Исторические notes в локальном docs/superpowers/ не входят в опубликованное дерево. Источник текущего поведения — код; единый план — writing-plans-gpu-roadmap.md.

## Known Unknowns

- ROS/workload и latency/resource budgets; модель desktop GPU уточнена: RTX 3060.
- Speedup полного control loop и system verification на выбранном target.
- Отдельный project profile пока не создан.

## Current measurement evidence

- [Desktop MPPI baseline report](../tasks/TASK-001/performance-profiler-report.md): source 9557655b + measurement-only changes, branch perf/gpu-baseline, 2026-10-23. Exploratory RTX 3060 evidence; controlled performance acceptance and full package readiness not confirmed.

## Correctness follow-up

- [Baseline correctness fixes](../tasks/TASK-002/systematic-debugger-report.md), 2026-10-23, branch fix/mppi-baseline-checks, source 9557655b + source-hashed fixes: CPU/CUDA 18/18 with actual cppcheck; focused ASan/UBSan/leak checks PASS. Controlled target performance and full lifecycle scope remain separate gates.

## R2 implementation planning

- [TASK-005 implementation plan](../tasks/TASK-005/writing-plans-plan.md) and [command reference](../tasks/TASK-005/writing-plans-commands.md): b23f498a baseline, desktop RTX 3060; P0–P6 checkpoints and R2-01…10 acceptance. [PROPOSED] Implementation not started; historical TASK-002 PASS is not new R2 evidence.
