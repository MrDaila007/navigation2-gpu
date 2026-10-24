# Desktop MPPI GPU baseline report

**Hardware clarification (2026-10-24):** desktop target и baseline — RTX 3060; численные результаты и сохранённые raw artifacts не изменялись.
**Correctness follow-up:** прежние cppcheck/teardown findings закрыты в [TASK-002](../TASK-002/systematic-debugger-report.md); историческая таблица проверок ниже сохранена для исходного baseline.
**Date:** 2026-10-23. **Branch:** perf/gpu-baseline, от feat/gpu-adaptation.
**Source:** 9557655b38a839b7c14e0aff862f63332f7ec7e4 + measurement-only benchmark source/CMake changes; source hashes in [evidence](performance-profiler-artifacts/source-hashes.txt).
**Status:** exploratory controller baseline completed; R0/R1 remain partial, DoD readiness blocked by existing cppcheck failures and missing target/tool evidence.
**Environment and measurement contract:** [performance-profiler-environment.md](../../specs/performance-profiler-environment.md).
**Exact commands:** [performance-profiler-commands.md](performance-profiler-commands.md).

## Вывод

[VERIFIED] Новый стенд вызывает настоящий `MPPIController::computeVelocityCommands()`, включая locks, optimizer, critics и trajectory validation. Получены 80 000 measured calls: 8 scenarios × 2 backends × 1000 samples × 5 repetitions, плюс 50 warmup calls на каждый backend/repetition. Все 42 000 пар команд (включая warmup) совпали побитово по linear.x/linear.y/angular.z. На CUDA проверены ровно 2100 launches на repetition, на CPU — ноль. Источник: чистые [выборки](performance-profiler-artifacts/results-separated/), collector/analyzer assertions.

[VERIFIED] Для этого synthetic workload CUDA не даёт универсального выигрыша полного цикла. CostCritic с footprint увеличивает среднее wall time приблизительно на 11–15%; варианты ObstaclesCritic уменьшают его приблизительно на 3–9%. Изменение p99 неоднородно. Значения CPU/CUDA ниже — отношения средних, не обещание speedup на любом workload.

[VERIFIED] Текущая desktop-платформа и GPU baseline — RTX 3060 12 GiB. Governor powersave, clocks не фиксировались; GPU load snapshots до/после — 29%/93%, used memory 7625/7442 MiB; после серии P2, graphics 1837 MHz, memory 7300 MHz, 52 C. Это snapshots вне измеряемых интервалов, непрерывной telemetry нет. Фоновая нагрузка и динамические частоты не контролировались. Приоритет следующего измерения — повтор на согласованной целевой машине при контролируемой нагрузке.

## Результаты полного цикла

Конфигурация: DiffDrive, batch=2000, horizon=56, iterations=2, одна из двух obstacle critics + семь CPU critics. Fixed pose/path, а не движение робота в замкнутом контуре. Footprint OFF — radius 0.15 m; ON — ordered square half-width 0.15 m. Free — свободная карта; bands — lethal/inflated полосы сбоку коридора. CPU и CUDA используют один CUDA build с runtime switch: это изолирует backend от различий сборки. Отдельная OFF build проверяет независимость от toolkit.

| Scenario | CPU mean ms | CUDA mean ms | CPU/CUDA | CPU p99 ms | CUDA p99 ms | CPU process time change | Per-repeat speedup range |
|---|---:|---:|---:|---:|---:|---:|---|
| cost-fp0-free | 2.674 | 2.713 | 0.985x | 4.462 | 4.670 | +1.5% | 0.937..1.074x |
| cost-fp0-bands | 2.719 | 2.695 | 1.009x | 3.850 | 4.266 | -0.8% | 0.931..1.203x |
| cost-fp1-free | 2.762 | 3.074 | 0.898x | 4.977 | 5.409 | +11.4% | 0.828..1.129x |
| cost-fp1-bands | 2.744 | 3.168 | 0.866x | 3.879 | 5.670 | +15.5% | 0.798..1.079x |
| obstacles-fp0-free | 4.072 | 3.969 | 1.026x | 6.328 | 6.258 | -2.5% | 0.982..1.097x |
| obstacles-fp0-bands | 4.168 | 3.824 | 1.090x | 6.274 | 6.124 | -8.2% | 1.022..1.272x |
| obstacles-fp1-free | 4.226 | 3.847 | 1.099x | 5.446 | 5.787 | -8.9% | 1.044..1.249x |
| obstacles-fp1-bands | 4.188 | 3.917 | 1.069x | 6.335 | 6.428 | -6.4% | 1.031..1.112x |

PASS: 80,000 raw measured calls; 42,000 paired command comparisons including warmup; expected launches verified.


p99 рассчитан nearest-rank по 5000 raw wall samples каждого backend/scenario; range — минимальное/максимальное отношение средних по пяти парам repetitions. Positive process time change означает больше CPU time на вызов. Диапазоны пересекают 1x для всех CostCritic cases и free/no-footprint ObstaclesCritic: эффект не устойчив между повторами. Здесь нет confidence interval или доказательства statistical significance; большие хвосты не удалялись как outliers.

CPU process time — std::clock, включая process threads и bookkeeping; wall metric — steady_clock только вокруг computeVelocityCommands. Observer overhead отдельно не откалиброван. Setup, teardown, output, allocations перед циклом и warmup исключены из wall samples. Новые binary/fixture не изменялись между measured repetitions; collector поправлен только для раздельной записи stdout/stderr.

## Обоснование следующей оптимизации

[VERIFIED — source-derived counts, not profiler attribution] Helpers каждый score копируют полные x/y и costmap и синхронизируют stream. При batch/horizon/iterations этого стенда x/y H2D = 2 arrays × 2000 × 56 × 4 bytes × 2 iterations = 1 792 000 bytes за обычный controller call. При footprint CostCritic D2H center costs = 448 000 bytes за call (stride=2); ObstaclesCritic = 896 000 bytes. Дополнительные costmap/flag/reduction transfers сюда не включены. Источники: `cost_critic_cuda.cu` score_center/gather_center и `obstacles_critic_cuda.cu` gather_center; calls/iterations подтверждены launches.

[ASSUMED — optimization hypothesis] GPU-resident trajectories/costs и общий optimizer-owned context уменьшат эти transfers и synchronization boundaries. Для большого общего speedup дальше требуется rollout/reductions/control update на device. Перенос ещё одного изолированного critic без контроля границ данных может добавить overhead. Польза R2 должна подтверждаться тем же workload/полным wall time после изменений; текущие measurements сами по себе не доказывают причинную долю каждого kernel/transfer.

[UNKNOWN] Точное распределение CPU time между rollout, critics и updateControlSequence не получено. CUDA trace attribution не подтверждена: инструменты дали infrastructure errors, описанные ниже. Профиль живой costmap/localization/planning также не выполнен — нет зафиксированного live navigation workload. Это ещё не полный R1 всего Nav2.

## Проверки — DOD Tier 0/1, desktop baseline

| Проверка | Результат | Evidence / предел |
|---|---|---|
| Release CPU-only configure/build | PASS | cpu-build.log + cpu-rebuild.log, CMakeCache-cpu.txt; CUDA OFF stubs |
| Release CUDA configure/build | PASS | gpu-build.log + gpu-rebuild.log, CUDA ON/86; CMakeCache-gpu.txt |
| Functional suites CPU/CUDA | PASS для всех 13 GTest targets | полные test-results-cpu/gpu; это не PASS всей CTest suite |
| CostCritic parity | CPU: 27 passed + 1 skipped; GPU: 52 passed + 1 skipped | OFF timing skip; ON missing-device skip |
| ObstaclesCritic parity | CPU: 30 passed; GPU: 59 passed + 1 skipped | missing-device case отдельно выполнен ниже |
| Missing-device fallback | PASS: 1 test каждого critic | no-device-cost.log / no-device-obstacles.log, CUDA_VISIBLE_DEVICES=-1 |
| CPU-only harness, pair/no-device rejection, invalid-count rejection | PASS | cpu-smoke-final.log; cli-missing-device.log / cli-invalid-count.log |
| Новый source static check | PASS | harness-cppcheck-final.log, AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 |
| Final source/CMake lint | PASS | cpu-lint-final.log: cpplint, lint_cmake, uncrustify; harness-format-final.log |
| Package cppcheck | FAIL, 10 существующих diagnostics | cppcheck-cpu/gpu.txt; tests unchanged, source-hashes.txt confirms original /ws versions |
| Полная suite | FAIL: GPU 17/18 из-за cppcheck; CPU исходно 16/18, format исправлен и перепроверен, cppcheck остаётся FAIL | cpu-tests.log/gpu-tests.log плюс cpu-lint-final.log; failures не скрыты |
| Controller command replay / launches / raw samples | PASS на synthetic fixtures | 80 000 samples, 42 000 paired commands; analyzer validates lengths, finite numbers and summaries |
| CUDA memcheck | NOT RUN: attach failed | memcheck-cost.log: No attachable process found, timeout; diagnostic terminated, second critic not attempted |
| Nsight usable trace summary | NOT RUN | nsys-profile.log: missing importer; host importer exited 3 / TargetProfilingFailed / TimestampType not initialized |
| Host ASan/UBSan/TSan, device racecheck | NOT RUN | алгоритмы/production memory ownership не менялись; harness reviewed, но sanitizer coverage не заявляется |
| Peak per-process RAM/VRAM, power budget | NOT RUN | global inventory snapshots не являются component/resource acceptance |
| Jetson, robot/SIL/HIL | NOT RUN | desktop baseline — fixed workload без simulator/live sensors |

Весь перечисленный evidence хранится в [performance-profiler-artifacts](performance-profiler-artifacts/). CUDA driver/runtime операции и kernel behavior проверены на целевой desktop RTX 3060. Existing `cppcheck` раньше был skip; новый forced run не является regression от benchmark source, но блокирует полную readiness.

## Ошибки стенда и исключённые данные

- Первый footprint smoke обнаружил неверную настройку: polygon после on_configure не переключает getUseRadius. Исправлено declaration до configure, проверен configure return и согласован global_frame=odom. Все восемь smoke fixtures после этого проходят.
- Первые results/ остановились на этой ошибке и исключены. results-final/ содержал stdout/stderr interleaving и также исключён целиком.
- Final accepted dataset — только results-separated/. Collector отдельно пишет ROS diagnostics и принудительно устанавливает RCUTILS_LOGGING_USE_STDOUT=0; counts/float parsing проверяются после каждого scenario. Analyzer дополнительно сверяет 1000 samples per repetition, точные launches, parity records и raw arithmetic means.
- Compute Sanitizer процессы остановлены по точным PID после attach timeout. CUDA clocks/permissions/container capabilities не менялись. Nsight intermediate qdstrm и частично импортированный nsys-rep не использовались для percentages/attribution.

## Ограничения и handoff

[VERIFIED] При teardown есть ClassLoader unload-while-live warnings и lifecycle warnings helper nodes; они вне measured intervals. Ownership/lifetime требует отдельной диагностики; ничего не подавлено. Детали и 10 cppcheck source locations — [DEFERRED.md](DEFERRED.md). Старые тесты с uninitialized state не исправлялись под видом профилирования.

[UNKNOWN] Не закреплены source/artifact revisions dependency overlay /ws/install; image digest и MPPI source hashes зафиксированы, но полный reproducible image ещё не подготовлен. Нет product frequency/deadline, quality tolerances и VRAM budget. Default random engine одинаков внутри одной toolchain, но cross-platform RNG equivalence не заявляется. Сравнивались три поля команд, а не полные массивы всех trajectories/costs.

Следующий skill для существующих failures — `systematic-debugger`: воспроизвести forced cppcheck по указанной команде, проверить ожидаемые поля state, исправить deterministic fixtures и исследовать lifetime warnings отдельным scope. Для дальнейшей GPU-работы после controlled baseline — контракт residency R2 и отдельная реализация от feat/gpu-adaptation; production behavior этим benchmark не менялся. Новые commits/merge/push не выполнялись.
