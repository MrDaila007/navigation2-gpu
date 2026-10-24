# GPU Migration Development Plan

**Task:** TASK-001
**Host:** Linux x86_64; host GCC 11.4/CUDA 11.5 по предыдущему inventory; историческая рабочая сборка в ROS Rolling container.
**Target:** desktop RTX 3060, модель уточнена пользователем 2026-10-24. Jetson позже.
**Profiles:** GPU/host-device constraints; отдельный project profile отсутствует, не считать tooling/target полностью зафиксированными.
**Goal:** создать глобальный roadmap и исполнимую первую итерацию переноса вычислений; не реализовывать kernels в рамках этой задачи.
**Constraints:** CPU-only compatibility, source numerical contracts, ROS plugin semantics; desktop latency/resource budgets ещё не утверждены.
**Evidence required:** документация Tier 0 сейчас; будущая реализация Tier 1–3 по scope с измерениями на целевом GPU.
**Baseline revision/state:** `9557655b38a839b7c14e0aff862f63332f7ec7e4`; исходно untracked `specs/`; локально изменённый ignored `AGENTS.md` с согласованным branching rule; известных свежих build/test результатов нет.
**Branch:** `feat/gpu-roadmap`, от `feat/gpu-adaptation`. Никаких merge/push этот план не выполняет.
**Closest existing patterns:** `CostCriticCuda::score_center` / `ObstaclesCriticCuda::gather_center` в `nav2_mppi_controller/src/critics/*_cuda.cu`; parity tests в `test/*_critic_parity_test.cpp`; full-model benchmarks `benchmark/{optimizer,controller}_benchmark.cpp`.

## Требования и evidence

| ID | Требование | Evidence в плане |
|---|---|---|
| GPU-01 | Максимально перенести активные численные workloads Nav2 на GPU | R2–R10 и residual compute ledger в roadmap |
| GPU-02 | Текущий desktop RTX 3060, Jetson позже | R0, J1 и platform gate |
| GPU-03 | Общая GPU-ветка и отдельные оптимизации | AGENTS.md, ветки ниже |
| GPU-04 | Сохранить корректность и CPU fallback | numerical contract и общие gates roadmap |
| GPU-05 | Измерять full-cycle benefit и CPU reduction | R1 measurement protocol, команды A–D ниже |
| GPU-06 | Исполнимая последовательность без обещаний непроверенного speedup | Work items ниже, PLANNED статусы и DoD |

Источник приоритетов и критериев: [глобальный roadmap](../../specs/writing-plans-gpu-roadmap.md). Здесь — стартовая последовательность и команды; архитектурные решения будущих этапов принимаются перед их реализацией.

## Work item 1 — Зафиксировать среду и CPU/CUDA baseline (R0)

**Purpose:** GPU-02/04/05. Ветка реализации `perf/gpu-baseline` от общей GPU-ветки.
**Files:** проверить `nav2_mppi_controller/CMakeLists.txt`, `nav2_common/cmake/nav2_package.cmake`, `nav2_mppi_controller/test/CMakeLists.txt`, `Dockerfile`, `.github/workflows/build_main_against_distros.yml`. Создать `specs/project-profiler-profile.md` и индексировать в `specs/MANIFEST.md` при фиксации среды. Решение о воспроизводимом GPU container file принять после проверки существующего образа; не заменять общий Dockerfile вслепую.
**Symbols/targets:** `mppi_controller`, `mppi_critics`, parity test targets, NAV2_MPPI_CUDA.
**Steps:**

1. Проверить target inventory, container/image digest, driver/toolkit/compiler/ROS, RAM/VRAM, монтирование /ws и overlay dependencies. Ни host Humble, ни имя image не подтверждают совместимость Rolling API.
2. Записать active plugin lists, control frequency, sensor rates, map sizes; утвердить budgets и numerical policy. Выбор конкретного Jetson сейчас не требуется.
3. Запустить отдельные OFF/ON сборки и tests командами A/B; собрать skips/errors. Убедиться, что no-device fallback проверен отдельным запуском.
4. Реально выполнить cppcheck: текущий historical run его пропускал. Проверить AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS в доступном ament, затем запустить CTest с этим env, если это поддержанный override.
5. Зафиксировать revision + build args + image + test reports. Сравнение CPU/GPU до workloads использовать только как smoke, не как performance conclusion.

**Ownership/concurrency:** код не меняется; отдельные build/install/log для конфигураций, не переиспользовать worktree artifacts другого ABI/toolchain.
**Compatibility/blast radius:** сборка MPPI и его overlay dependencies; defaults/API без изменений.
**Before/after evidence:** TDD не требуется для фиксации среды. До шага toolchain воспроизводимость UNKNOWN; после — OFF/ON builds и tests с реальным PASS/FAIL/SKIP, no-device evidence, target identity.
**Rollback:** восстановить прежнюю конфигурацию container/runtime; device reset/flash не требуется.
**Checkpoint:** заполненный profile и commands/output paths, parity tests не skipped на GPU lane; blockers явно перечислены.

## Work item 2 — Сопоставимые measurements и глобальный профиль (R1)

**Purpose:** GPU-01/05/06.
**Modify:** `nav2_mppi_controller/benchmark/{cost_critic_cuda_benchmark,obstacles_critic_cuda_benchmark,optimizer_benchmark,controller_benchmark}.cpp`, `nav2_mppi_controller/benchmark/CMakeLists.txt`.
**Read/optionally instrument:** `nav2_mppi_controller/src/{optimizer,critic_manager,controller}.cpp` — `optimize`, `evalTrajectoriesScores`, `computeVelocityCommands`; instrumentation должна быть выключаемой и не добавлять per-sample logging.
**Extend tests where needed:** `nav2_mppi_controller/test/{cost_critic_parity_test,obstacles_critic_parity_test,optimizer_smoke_test,controller_state_transition_test}.cpp`; зарегистрировать новые meaningful test targets только при новой harness logic.
**Create evidence:** `tasks/TASK-001/writing-plans-baseline-results.md` для принятых baseline measurements. Сырые confidential traces хранить локально в согласованном месте с ссылкой/identity, не публиковать внешним сервисам.

**Steps:**

1. Добавить benchmark arguments/fixtures для одинаковых CPU/CUDA inputs; не менять plugin defaults. Разделить warmup, allocation, steady state и rendering contention.
2. Исправить разрыв bitwise requirement/test assertion для текущих critics без ослабления контракта; численные исключения согласовать отдельно. Проверить collision flags и command replay, а не только стоимость одного critic.
3. Запустить commands C для smoke и повторяемых средних. Для p95/p99 собирать отдельные per-cycle wall-clock samples по protocol R1: как минимум 1000 × 5 после warmup; Google Benchmark агрегаты не подменяют эти samples.
4. Получить CPU/system trace costmap, AMCL, planner и controller на одном workload. Разделить process CPU time, threads, GPU kernels, transfers, waits и deadlines. Доступность profiler обнаружить в container/host; точную invocation записать в results после проверки инструмента, не изобретать неподтверждённые flags.
5. Составить ranked ledger по symbol/component, baseline share, parallel candidate, transfers, expected dependency, evidence. Определить исполнение R2/R3/R4 и порядок R7/R8/R9.

**Ownership/thread:** samples buffers имеют bounded capacity и одного владельца; измерять текущий controller/costmap lock, не менять порядок блокировок ради профиля. Trace overhead оценить отдельным запуском.
**Interfaces/blast radius:** benchmark CLI может расширяться; production API и numerical behavior неизменны; build targets benchmarks + затронутые tests.
**TDD:** новый fixture сначала должен различать CPU и CUDA по launch counter, сохраняя одинаковые inputs; тесты должны ловить disabled/no-device/stale map cases. Не писать тесты на формат отчёта.
**Success:** сопоставимый performance report с confidence/разбросом и explicit regression cases, не обещание speedup. GPU execution — только evidence на actual target; profiling неизвестных сред — NOT RUN.
**Rollback:** instrumentation выключить, вернуть CPU flag; изменения benchmark отдельно от production fixes.
**Commit boundaries:** fixtures; comparison/replay tests; tracing; baseline evidence.
**Checkpoint:** заполненный baseline report, утверждённые budgets и ранжирование, известны full-loop CPU/GPU latency и transfers.

## Work item 3 — Подготовить contract residency, затем R2

**Purpose:** GPU-01/04/05; зависит от items 1/2.
**Exact existing entry files:** `nav2_mppi_controller/include/nav2_mppi_controller/{optimizer,critic_manager,critic_data}.hpp`; `src/{optimizer,critic_manager,controller}.cpp`; `include/nav2_mppi_controller/critics/{cost_critic_cuda,obstacles_critic_cuda}.hpp`; `src/critics/{cost_critic_cuda,obstacles_critic_cuda}.cu`; CPU stubs; package CMake/test/benchmark files.
**Design artifact before code:** `specs/architect-architecture.md` задаёт R2 ownership/residency direction. Следующий `specs/interface-designer-contracts.md` (ещё не создан) должен зафиксировать exact descriptors/limits/status/ABI contract; затем implementation plan. Это не уже реализованный API.
**Steps:** выбрать минимальный backend boundary по baseline; перечислить точные новые source/header names в плане R2; добавить tests shared-map lifetime/resize/failure; подключить оба critics, устранить повторную загрузку одинаковых inputs; выполнить A/B/C и изменённые package tests.
**Ownership:** один context на optimizer instance; cleanup после завершения device work; никакого process-global ownership; CPU recompute не использует partial device result. Межпроцессный costmap sharing вне первого R2.
**Compatibility/blast radius:** MPPI controller/critics, plugins, lifecycle/params; `nav2_core` API не менять. Сторонний CPU critic нельзя пропустить — explicit sync/fallback.
**TDD/success:** тест обнаруживает лишние uploads/launches или stale map до изменений; после — consistency flags, no duplicated upload между встроенными critics в той же iteration, сравнение full-loop time и allocation/transfer counts.
**Rollback:** runtime CPU mode + OFF build, revert focused integration commit при regression, без rewrite истории.
**Checkpoint:** contract accepted, file list fixed, focused and package suites + performance results recorded. До этого нельзя объявлять R2 реализованным.

## Последующие work items R3–J1

Roadmap задаёт для каждого цель и exit gate. Перед реализацией декомпозировать в отдельную TASK с exact files/commands, учитывая обновлённый baseline:

| Work item | Начальные существующие files / tests | Command families |
|---|---|---|
| R3 scoring | `nav2_mppi_controller/src/critics/*.cpp`, corresponding headers; `test/critics_tests.cpp`, parity tests; footprint checker source + `nav2_costmap_2d/test/unit/footprint_collision_checker_test.cpp` | A/B/C; D если меняется costmap |
| R4 rollout/noise | `src/noise_generator.cpp`, `src/optimizer.cpp`, `include/nav2_mppi_controller/motion_models.hpp`; `test/{noise_generator_test,motion_model_tests,optimizer_unit_tests}.cpp` | A/B/C |
| R5 reductions/update | `src/optimizer.cpp`, `include/nav2_mppi_controller/tools/utils.hpp`; `test/{optimizer_unit_tests,controller_state_transition_test}.cpp` | A/B/C + command replay |
| R6/R10 system | `nav2_system_tests/src/{system,localization,planning,system_failure}/`, their CMakeLists; `nav2_bringup/params/nav2_params.yaml`; CI workflows | D/E; availability of simulator required |
| R7 grids | `nav2_costmap_2d/plugins/{inflation_layer,obstacle_layer,voxel_layer}.cpp`, `include/nav2_costmap_2d/distance_transform.hpp`; unit/integration tests and `test/benchmark/inflation_layer_updatecosts_benchmark.cpp`; `nav2_voxel_grid/test/voxel_grid_tests.cpp` | D; CPU/GPU feature flags require design before creation |
| R8 AMCL | `nav2_amcl/src/sensors/laser/{likelihood_field_model,likelihood_field_model_prob,beam_model}.cpp`, `src/pf/pf.c`, motion models, `CMakeLists.txt`; `nav2_amcl/test/CMakeLists.txt`, localization system test | D/E; add pure sensor scoring tests before kernels |
| R9 planning | `nav2_smac_planner/src/{obstacle_heuristic,costmap_downsampler,collision_checker,smoother}.cpp`, `include/nav2_smac_planner/a_star_impl.hpp`; existing `test/test_{a_star,collision_checker,smoother,costmap_downsampler}.cpp` | D/E; full-search spike separate |
| J1 | Existing package CMake, container/toolchain configuration chosen after board selection | NOT DEFINED until model/JetPack/ROS fixed; do not flash without scope/recovery |

Таблица — entry points, не массовое разрешение изменить все файлы. Для каждого этапа применяются ownership/compatibility/failure/verification/rollback gates roadmap. Новые `.cu` filenames и target flags фиксируются до реализации соответствующего stage.

## Exact verification commands

**Prerequisites:** prepared Rolling environment inside container, repository at `/ws`, dependencies built and sourced from compatible overlay. Commands below execute there. Existing container state and tools must be checked first. Use dedicated paths below, not existing worktree build artifacts.

### A — CPU-only

```bash
source /opt/ros/rolling/setup.bash
cd /ws
colcon --log-base log-cpu build --build-base build-cpu --install-base install-cpu --packages-up-to nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=OFF
source /ws/install-cpu/setup.bash
colcon --log-base log-cpu test --build-base build-cpu --install-base install-cpu --packages-select nav2_mppi_controller --event-handlers console_direct+
colcon test-result --test-result-base /ws/build-cpu/nav2_mppi_controller --verbose
```

### B — CUDA (в отдельной shell, без CPU overlay)

```bash
source /opt/ros/rolling/setup.bash
cd /ws
colcon --log-base log-gpu build --build-base build-gpu --install-base install-gpu --packages-up-to nav2_mppi_controller --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86
source /ws/install-gpu/setup.bash
colcon --log-base log-gpu test --build-base build-gpu --install-base install-gpu --packages-select nav2_mppi_controller --event-handlers console_direct+
colcon test-result --test-result-base /ws/build-gpu/nav2_mppi_controller --verbose
CUDA_VISIBLE_DEVICES=-1 /ws/build-gpu/nav2_mppi_controller/test/cost_critic_parity_test --gtest_filter=CostCriticParity.CudaFlagOnMatchesCpuWhenDeviceMissing
CUDA_VISIBLE_DEVICES=-1 /ws/build-gpu/nav2_mppi_controller/test/obstacles_critic_parity_test --gtest_filter=ObstaclesCriticParity.CudaFlagOnMatchesCpuWhenDeviceMissing
```

### C — Existing benchmark executables (после B)

```bash
/ws/build-gpu/nav2_mppi_controller/benchmark/cost_critic_cuda_benchmark --benchmark_min_time=0.5s --benchmark_repetitions=5
/ws/build-gpu/nav2_mppi_controller/benchmark/obstacles_critic_cuda_benchmark --benchmark_min_time=0.5s --benchmark_repetitions=5
/ws/build-gpu/nav2_mppi_controller/benchmark/optimizer_benchmark --benchmark_min_time=0.5s --benchmark_repetitions=5
/ws/build-gpu/nav2_mppi_controller/benchmark/controller_benchmark --benchmark_min_time=0.5s --benchmark_repetitions=5
```

До item 2 новые critic benchmarks не имеют CPU workload mode; эти команды лишь существующий smoke/timing. CPU baseline full-model executables находятся в build-cpu после A. Уточнить fixtures перед сравнением: controller_benchmark name сам по себе не доказывает измерение реального ROS control period.

### D — Дополнительные пакеты, только после включения в scope

```bash
source /opt/ros/rolling/setup.bash
cd /ws
colcon --log-base log-stack build --build-base build-stack --install-base install-stack --packages-up-to nav2_costmap_2d nav2_voxel_grid nav2_amcl nav2_smac_planner nav2_collision_monitor --cmake-args -DBUILD_TESTING=ON
source /ws/install-stack/setup.bash
colcon --log-base log-stack test --build-base build-stack --install-base install-stack --packages-select nav2_costmap_2d nav2_voxel_grid nav2_amcl nav2_smac_planner nav2_collision_monitor --event-handlers console_direct+
colcon test-result --test-result-base /ws/build-stack --verbose
```

D проверяет текущие CPU interfaces этих пакетов; новых GPU flags в них пока нет. Дополнить команды отдельными OFF/ON опциями после design, не выдавать CPU run за GPU verification.

### E — Existing system tests, если simulator prerequisites доступны

```bash
source /opt/ros/rolling/setup.bash
cd /ws
colcon --log-base log-system build --build-base build-system --install-base install-system --packages-up-to nav2_system_tests nav2_mppi_controller nav2_bringup --cmake-args -DBUILD_TESTING=ON -DNAV2_MPPI_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=86
source /ws/install-system/setup.bash
colcon --log-base log-system test --build-base build-system --install-base install-system --packages-select nav2_system_tests --event-handlers console_direct+
colcon test-result --test-result-base /ws/build-system --verbose
```

Сначала проверить simulator requirements в `nav2_system_tests/README.md` и launch fixtures; существующие system tests не гарантируют выбор MPPI/use_cuda. Для GPU acceptance fixture должен явно выбирать нужный backend и подтверждать launches/version, затем replay сравнивается с CPU конфигурацией. Actual robot/HIL — отдельная согласованная задача с recovery.

## Проверка этого плана — DOD Tier 0

- Scope/target/численные и resource unknowns указаны; requirements matrix связывает намерение и этапы.
- План и living roadmap соответствуют naming policy; specs index обновлён; будущая реализация не объявлена выполненной.
- Проверки: `git diff --check`; Python link/path/section checks, описанные в STATE.
- Build/test/sanitizer/target runs: NOT RUN, потому что текущий deliverable — roadmap, исходники не менялись. Исторические результаты не выдаются за свежие.
- Blockers реализации: не зафиксированы runtime/workload/budgets, inventory mismatch и расширенный numerical contract. Это входы R0, не препятствие завершить планирование.
