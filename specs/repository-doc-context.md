# Repository Context

**Verified revision:** main `94bace6f7bcf60ecfc17cd3a97155657c3c249aa`; GPU worktree `9557655b38a839b7c14e0aff862f63332f7ec7e4`
**Verified date:** 2026-10-23
**Project profile:** отдельный `specs/project-profiler-profile.md` отсутствует; проверенные сведения ниже.
**Confidence:** verified facts + listed assumptions/unknowns

## System shape

[VERIFIED] Это ROS 2 Navigation2: C++ библиотеки/плагины, ROS nodes, Python tools, ament/colcon/CMake. GPU-работа ограничена `nav2_mppi_controller`. Источники: package manifests и diff веток.

| Ветка | Ревизия | Состояние |
|---|---|---|
| main | 94bace6f | CPU MPPI; новых CUDA sources нет |
| feat/mppi-cost-critic-cuda | f8e49f20 | CUDA CostCritic; есть remote-tracking ветка |
| feat/mppi-obstacles-critic-cuda | 9557655b | CostCritic + ObstaclesCritic; локальная ветка |

[VERIFIED] Последняя ветка открыта в `.worktrees/mppi-cost-critic-cuda`, несмотря на имя каталога. Diff относительно общего предка с main: 17 файлов, +4060/-49 строк. Источники: `git worktree list`, `git branch -avv`, `git diff --stat main...feat/mppi-obstacles-critic-cuda`. Remote-tracking ветка не подтверждает текущий статус PR/сервера.

[VERIFIED] История обоих этапов: CPU scoring tests → opt-in/fallback → CUDA scoring/gather → footprint parity → resize → timings. Источник: `git log --oneline main..feat/mppi-obstacles-critic-cuda`.

## Build and verification quick reference

[VERIFIED] Host Linux x86_64, GCC 11.4, CUDA Toolkit 11.5.119, установлен ROS Humble. Вне sandbox доступна GeForce RTX 3060, driver 550.163.01. Команды: `uname -sm`, `g++ --version`, `nvcc --version`, `ls /opt/ros`, `nvidia-smi --query-gpu=name,driver_version --format=csv,noheader`. Внутри sandbox NVIDIA driver недоступен.

[VERIFIED] Сохранённые сборки/тесты выполнялись в ROS Rolling, пути `/ws` и `/opt/ros/rolling`. C++20 задаёт `nav2_common/cmake/nav2_package.cmake`; CUDA C++17 — GPU `nav2_mppi_controller/CMakeLists.txt`; CMake minimum 3.20. Тесты GoogleTest/Google Benchmark/ament lint. CI содержит Jazzy/Lyrical, CUDA-изменений CI в diff нет. Источники: worktree logs, build files, `.github/workflows/build_main_against_distros.yml`.

Команды следующего воспроизведения внутри подготовленного Rolling-контейнера с GPU worktree в `/ws` (в этом обзоре не запускались):

```bash
source /opt/ros/rolling/setup.bash
cd /ws
colcon build --packages-select nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86
source /ws/install/setup.bash
colcon test --packages-select nav2_mppi_controller --event-handlers console_direct+
colcon test-result --test-result-base /ws/build/nav2_mppi_controller --verbose
/ws/build/nav2_mppi_controller/benchmark/cost_critic_cuda_benchmark --benchmark_min_time=0.5s
/ws/build/nav2_mppi_controller/benchmark/obstacles_critic_cuda_benchmark --benchmark_min_time=0.5s
```

CPU build: отдельные build/install/log, `-DNAV2_MPPI_CUDA=OFF`. Предполагаются уже собранные зависимости overlay. Имя контейнера `nav2-rolling` и команды есть в `docs/superpowers/plans/2026-10-23-mppi-obstacles-critic-cuda.md`; текущее состояние контейнера не проверено.

## Entry points and primary flows

[VERIFIED] `MPPIController::computeVelocityCommands()` → `Optimizer::evalControl()` → генерация траекторий → последовательные `CriticManager` scoring calls → `Optimizer::updateControlSequence()`. Основной optimizer pipeline, motion models и остальные critics не перенесены на GPU. Источники: `src/{controller,optimizer,critic_manager}.cpp`, GPU diff.

| Компонент | На GPU | На CPU |
|---|---|---|
| CostCritic без footprint | Чтение карты, collision checks, накопление стоимости траектории | Финальные weight/power и обновление результатов |
| CostCritic с footprint | Gather стоимости центров с trajectory_point_step | Footprint checker и последующая оценка |
| ObstaclesCritic | Gather центров для всех временных шагов | Footprint, collision rules, log/distance, min/reductions, weight/power |

[VERIFIED] Источники: GPU worktree `src/critics/{cost_critic,obstacles_critic}.cpp` и соответствующие `_cuda.cu`. Gather не означает перенос полного scoring.

## Components and ownership

[VERIFIED] Каждый critic владеет отдельным helper/stream/events/device buffers. Буферы сохраняются и растут по необходимости, деструкторы освобождают ресурсы. Costmap и x/y копируются при каждом вызове, результаты возвращаются на CPU. Общего GPU-контекста/costmap между critics нет. Источники: CUDA headers, `DeviceState`, `grow_n`, `score_center`, `gather_center`.

## Concurrency / real-time context

[VERIFIED] Controller удерживает mutex параметров и costmap во время evalControl; critics последовательны. CUDA synchronizes stream перед чтением результата на host. Источники: `src/controller.cpp:104`, `src/critic_manager.cpp`, CUDA `cudaStreamSynchronize`.

[UNKNOWN] Бюджеты p95/p99, jitter, RAM/VRAM и power не определены в найденных GPU designs. Конкурентная работа нескольких controller instances требует проверки: timing/launch counters в CUDA helpers глобальные и не атомарные.

## Interfaces and compatibility boundaries

[VERIFIED] Build switch `NAV2_MPPI_CUDA=OFF` по умолчанию: OFF выбирает stubs без CUDA headers, ON — CUDA sources и cudart. У каждого critic независимый `use_cuda=false`. Есть CPU fallback при отсутствии устройства/неуспешной CUDA-операции. Plugin ids сохранены. Источники: GPU CMake, critic initialize/score, stubs; XML/package manifests не изменялись.

[UNKNOWN] Наличие fallback в коде не подтверждает все ошибки driver/OOM/stream. Нужны fault-injection и lifecycle проверки.

## Target / hardware boundaries

[VERIFIED] Исторические логи содержат kernel timings и launch-counter tests. RTX 3060 сейчас подтверждена inventory-командой. Это не evidence выполнения на Jetson или роботе.

[VERIFIED — human clarification, 2026-10-24] Текущий target разработки и оптимизации — desktop с NVIDIA GeForce RTX 3060. Адаптация под Jetson отложена на будущее и не является критерием готовности desktop-этапа. Конкретная модель Jetson не выбрана; упоминание Orin/architecture 87 в старых designs не фиксирует выбор платформы.

[VERIFIED — human clarification, 2026-10-24] Desktop target и baseline используют RTX 3060. [UNKNOWN] Продуктовые ROS/workload и latency/resource budgets ещё не утверждены; synthetic baseline не является полной системной приёмкой.

## Tests as documentation

[VERIFIED, historical evidence] GPU worktree `log/test_2026-10-23_18-06-18/nav2_mppi_controller/stdout.log`: CTest 18/18 без failures; CostCritic 52 passed + 1 skipped; ObstaclesCritic 59 passed + 1 skipped. Пропущены случаи отсутствия GPU, поскольку устройство доступно. XML: `build/nav2_mppi_controller/test_results/nav2_mppi_controller/`. Это сохранённые результаты, не новый прогон и не доказательство соответствия бинарника текущему HEAD.

[VERIFIED] `test/{cost,obstacles}_critic_parity_test.cpp` покрывают free/lethal/unknown/off-map, flags, footprint, near goal, power, disabled mode, CostCritic stride, все колонки ObstaclesCritic, resize/origin/resolution и kernel launches.

[VERIFIED] Designs требуют bitwise equality; helpers используют `EXPECT_EQ` для float, без сравнения представления битов. Подтверждается точное числовое равенство проверенных случаев; полная побитовая эквивалентность шире evidence.

[VERIFIED] CTest показывает cppcheck Passed, но ament пропускает анализ версии 2.19.0 из-за известных performance issues; XML содержит 75 skipped. Реальный cppcheck — NOT RUN.

Один сохранённый timing call: 2000 траекторий × 56 шагов, мс:

| Critic | Map H2D | Trajectory H2D | Kernel | D2H | Сумма |
|---|---:|---:|---:|---:|---:|
| CostCritic | 0.003072 | 0.167200 | 0.016096 | 0.023232 | 0.209600 |
| ObstaclesCritic | 0.010240 | 0.184992 | 0.041312 | 0.129856 | 0.366400 |

[VERIFIED] Источник: stdout.log выше, строки 4021/6348. Копирования — около 92%/89% суммы. Это CUDA event intervals, не полное wall time score и не CPU speedup. Оба новых benchmark включают CUDA, используют свободную карту/одинаковые координаты; CPU baseline и сцены с препятствиями отсутствуют (`benchmark/{cost,obstacles}_critic_cuda_benchmark.cpp`).

## Operations and recovery

[VERIFIED] Возврат к CPU предусмотрен через use_cuda=false либо OFF build. Default остаётся CPU. В ходе обзора ветки не переключались/не сливались, устройства не изменялись.

## Gotchas

- [VERIFIED] Просмотр main скрывает реализованную GPU-работу; имя worktree не совпадает с веткой.
- [VERIFIED] MANIFEST был шаблоном; profile отсутствует. Реальные designs/plans находятся в `docs/superpowers/`. Unchecked пункты планов не отражают готовность: соответствующий код/commits уже есть.
- [VERIFIED] Graphify query описывает преимущественно CPU/main, не дал CUDA helpers; CLI сообщил version mismatch 0.9.41/0.9.94 и усечение выдачи. GPU-факты проверены по source/branches/logs.
- [VERIFIED] Предыдущие designs исключали гарантированный speedup, общий buffer, полный controller replay и другие модули. Это границы прошлых этапов, не автоматически дефекты.

## Unknowns / stale evidence

[UNKNOWN] Свежие OFF/ON build/tests, benchmarks, sanitizers/compute-sanitizer, GPU CI, Jetson, SIL/HIL и робот в этом обзоре не запускались. Исторические результаты отделены от текущих проверок. Нужен воспроизводимый performance report с source/artifact identity.

## Development roadmap

[VERIFIED — human decision] Общая integration branch — `feat/gpu-adaptation` (создана на `9557655b`); правило веток записано в AGENTS.md. Документы roadmap подготовлены на `feat/gpu-roadmap`.

Единый глобальный план, зависимости, numerical gates и направления за пределами MPPI: [writing-plans-gpu-roadmap.md](writing-plans-gpu-roadmap.md). Стартовые команды и checkpoints — [TASK-001](../tasks/TASK-001/writing-plans-plan.md). Этот раздел заменяет предварительный краткий список этапов; факты реализации выше остаются привязаны к baseline.

## Review evidence — DOD Tier 0

| Проверка | Результат / evidence |
|---|---|
| Ветки и реализованная область | PASS: git branch -avv; git worktree list; git log --oneline main..feat/mppi-obstacles-critic-cuda; git diff --stat main...feat/mppi-obstacles-critic-cuda |
| Host/toolchain/GPU | PASS: uname -sm; g++ --version; nvcc --version; nvidia-smi --query-gpu=name,driver_version --format=csv,noheader вне sandbox |
| Сделанное/оставшееся | PASS: implementation matrix, historical logs, unknowns и proposed roadmap выше |
| Новые build/test/target checks | NOT RUN: анализ документации/source/logs; implementation не менялась |


## Subsequent profiling evidence

2026-10-23: добавлен measurement-only harness на ветке perf/gpu-baseline. Свежие результаты, точные commands и открытые проверки — [performance report](../tasks/TASK-001/performance-profiler-report.md); среда — [performance-profiler-environment.md](performance-profiler-environment.md). Исторические результаты выше не подменяют этот отчёт.
