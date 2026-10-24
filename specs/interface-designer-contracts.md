# R2 GPU context — C++ contract v1

**Дата:** 2026-10-24. **Статус:** точное предложение интерфейсов; production implementation отсутствует.
**Платформа:** desktop RTX 3060, Linux ROS 2, одна device ordinal 0. Jetson/multi-GPU migration — будущий scope.
**Основа:** [архитектура](architect-architecture.md), [среда](performance-profiler-environment.md), source 9557655b + проверенные TASK-002 working fixes.
**Объявления:** [host prototype](../tasks/TASK-004/interface-designer-api.hpp), [native prototype](../tasks/TASK-004/interface-designer-device-api.hpp). Это declaration-only artifacts; после реализации header names берутся из архитектуры.

[PROPOSED] Все правила этого документа — contract choices v1, а не уже действующая функциональность и не утверждённые продуктовые performance budgets. [VERIFIED] Source facts: существующие helper signatures/float-double semantics в `include/nav2_mppi_controller/critics/*_cuda.hpp` и `src/critics/*_cuda.cu`; параметры batch_size [1,100000], time_steps [1,1000] в `src/optimizer.cpp`; score/layout в `critic_function.hpp`/`critic_data.hpp`.

## 1. Boundary, version, ABI

Native same-process C++ API, namespace `mppi::gpu`, kContractVersion=1. Ни IPC, ни serialization/file/wire format не вводятся. Все модули собираются согласованной toolchain/CRT; stable cross-compiler ABI не обещается. POD descriptors используют fixed-width integers, size_t только для host extents, native alignment/endianness без packing. Token handles содержат std::weak_ptr: они host-only, не POD и не копируются на GPU/в сеть. Использование STL здесь осознанно ограничено coherently rebuilt C++ modules, как существующие MPPI native interfaces.

`Context` noncopyable/nonmovable. Pimpl allocation/free выполняет provider `mppi_gpu_context`; внешняя оболочка принадлежит Optimizer. Все methods noexcept и возвращают Status; std::bad_alloc в provider переводится в HostMemoryUnavailable. Exceptions существующих CPU critics остаются в прежнем C++ error model. На границе controller InvalidInput/InvalidLimits превращаются в ControllerException, runtime fatal GPU status — NoValidControl; команда из partial data не публикуется.

`CriticFunction` и `CriticData` не меняются. `ReadOnlyInputsV1` / `CriticGpuV1` — дополнительные interfaces с out-of-line destructors/key functions, экспортируемыми одним shared target. Default visibility + единый RTTI required; нельзя прятать typeinfo в отдельных DSOs. `CriticGpuV1::gpu_contract_version()` возвращает 1; иной version не attach, такой plugin исполняется как opaque CPU consumer по прежнему score. Недопустимо объявить GPU plugin без CPU path совместимым таким fallback.

Новый owner field меняет Optimizer layout; встроенные marked critics также меняют derived layout. Нужна полная rebuild/install direct consumers. Unchanged old CPU plugin ABI проверяется downstream fixture, не обещается по одному анализу сигнатур. При следующем несовместимом extension вводится новый versioned interface; существующий V1 vtable не расширяется.

## 2. Limits и configuration

| Поле / controller parameter | Default | Диапазон и смысл |
|---|---:|---|
| Limits.max_rows | 100000 | internal cap 1..100000, не дублирует existing batch_size parameter |
| Limits.max_steps | 1000 | internal cap 1..1000, существующий time_steps сохраняется |
| gpu_context.max_map_cells | 16777216 | 1..UINT32_MAX, row-major cells, padding/pitch не поддерживается |
| gpu_context.max_device_bytes | 268435456 | 1..INT64_MAX bytes; provider payload capacity, не вся память driver/context |
| gpu_context.max_host_bytes | 268435456 | 1..INT64_MAX bytes; owned input/output payload capacity |
| gpu_context.measure_timings | true | bool, CUDA event timings как в текущих helpers; false допускается с явной меткой evidence |
| Limits.device_ordinal | 0 | только 0 в V1; иное значение — InvalidLimits |

Новые параметры read-only на configured lifetime: изменить через cleanup/configure. Имя namespace — `<controller_name>.gpu_context.*`. Старые `<controller>.<critic>.use_cuda=false` и enabled/default scoring values сохраняются. Limits bytes — инженерные defaults 256 MiB на host/device, не согласованный продуктовый бюджет; их изменение не меняет CPU mathematical limits. Ноль/отрицательные ROS integers — InvalidLimits до cast. Device ordinal не становится новым ROS parameter в V1.

Бюджет учитывает сумму capacities map/x/y/result/flags и bounded scalar buffers, включая peak replacement allocations. Runtime/events metadata и CUDA internal allocations измеряются отдельно и не входят в payload cap. Стратегия роста: exact required sizes, не geometric over-allocation; после drain старые invalid buffers освобождаются до replacement, если иначе нарушается cap. OOM не обязан сохранять старое device content. No allocation при неизменном warmed shape. No unbounded queues: один native operation или один live HostResult на context.

Для M map cells и P=N*T основные payload estimates: device M+12P+N+4 bytes; owned host staging/result приблизительно тот же порядок. При N=2000,T=56,M=1600: 1 347 604 bytes до implementation overhead. Checked sums/products делаются до allocation и narrowing. CapacityExceeded — CPU fallback на cycle; не silent clipping и не изменение batch/horizon.

## 3. Host inputs и validation

MapInput: data readable только на время синхронного snapshot; data_bytes **равно** width*height, width/height >0; row-major uint8 values, unknown=255 остаётся value, не ошибка формата. origin_x/origin_y/resolution — исходные double метры/метры на cell, finite, resolution>0. Контекст не владеет исходной Costmap2D, но владеет snapshot bytes после успешного begin/refresh. Новая control epoch не использует старые данные даже при прежнем адресе/shape.

BatchInput: x/y — contiguous column-major float, index t*rows+i; x_count=y_count=rows*steps, rows/steps >0 и в Limits. Aliased read-only x/y допускаются. Pointer alignment естественный для float. Перед выходом begin_batch/refresh данные скопированы в owned staging; caller buffers больше не нужны для DMA. CUDA headers, Eigen/ROS types и std::span C++20 в общем header отсутствуют.

Validation: null/count/shape/product/finite checks, including each x/y while staging; extrema сохраняются для constant-size operation-domain validation. Context не доказывает реальную доступность произвольного адреса native plugin — readable extent является обязанностью caller. Tokens не являются security sandbox/authentication. Ошибочный input не отправляется в CPU scorer как supposedly safe fallback.

ResultSpec validation отдельно от batch: finite nonnegative host_footprint_margin (метры), float geometry representable/positive для Cost kinds; normalized upper extrema в соответствующей float/double arithmetic должны быть finite и меньше 2^32 перед unsigned conversion. Для CPU footprint последующей обработки дополнительно проверяются double extrema с supplied margin. Adapter обязан передать не меньшую circumscribed footprint bound и проверить используемые yaw/footprint inputs по CPU contract. Это сохраняет определённый CPU numerical domain и отсекает unsafe casts; не вводит tolerance. Отдельные float/double lookup kernels и CostCritic stride остаются разными.

## 4. Tokens и отсутствие ABA

ControlToken, BatchToken и ResultToken — разные opaque host types, public default/copy, приватные поля. Пустой token никогда не валиден. Внутри weak identity configuration + uint64 control/batch/operation sequences. Weak identity не удерживает GPU resources или owner; после close/reconfigure expires. Сравнивается ownership/control block, не object address. Поэтому повторное выделение Context по тому же адресу не оживляет старый token. Process-global identity counter не нужен.

Новые counters начинаются с 1, zero reserved. Overflow не wrap: invalidate tokens и вернуть InvalidState до enqueue, закрыть/reconfigure identity вне активных leases. BatchToken меняется после каждой CPU generation, даже если index снова 0 после retry. Tokens не обходят state checks: read_result после release или вне active cycle — InvalidToken.

Все out parameters обнуляются на не-Ok (кроме read_stats, где out не меняется при ConcurrentUse). HostResult считается невалидным до successful read_result. Сам token не даёт права считать указатели живыми после release.

## 5. Context protocol

| Method | Preconditions | Success / lifetime |
|---|---|---|
| configure(Limits) | unconfigured или safely closed, без borrowed outputs | validate/store limits, fresh identity; explicit CUDA calls отсутствуют; host allocation error возвращается status |
| begin_control(MapInput,out) | idle, GPU requested owner path | validate/copy owner map snapshot; new ControlToken; capability probe lazy, cached missing runtime/device до reconfigure |
| begin_batch(control,BatchInput,out) | active control, no native/result lease | owned x/y snapshot, new BatchToken; старый batch invalid |
| invalidate_inputs(control) | no live lease; before opaque CPU score | drain if needed, mark map/x-y stale и invalidate batch; **не** копировать данные |
| refresh_inputs(control,map,xy,out) | stale inputs, no live lease, после opaque CPU score и только при следующем GPU demand | fresh owned snapshots, map revision/batch token; никаких driver reads caller buffers |
| read_result(result,out) | matching ready token | borrowed const values/flags, scalar all_collided и timings; no CUDA calls |
| release_result(result) | matching live result | invalidate borrowed view; workspace доступен следующему consumer |
| end_control(control) | no live native/result lease | drain, invalidate control/batch, idle; capacities retained |
| reset() | owner quiescent, no leases | drain/invalidate all epochs, retain capacity, cached fatal не очищается |
| close() | caller обеспечил отсутствие executing consumers | drain then free safely, invalidate identity; idempotent after success |
| read_stats(out) | object alive | consistent per-owner copy, nonblocking entry guard; no CUDA operations |

Если ни один enabled critic не запрашивает GPU, Optimizer не начинает GPU control scope; no explicit CUDA queries/copies/allocations. OFF backend даёт NotCompiled. Cached NoDevice/DriverUnavailable не probing каждый score; новая попытка capability setup — после reconfigure. Valid OOM/capacity/launch-rejected-drained disables GPU до end_control; next healthy cycle допускает максимум одну setup/reserve attempt, без retry loop/sleep.

Helper standalone создаёт private Context того же engine и begin/end cycle на каждый score; managed helper использует attached owner. Attach запрещён при live lease или к closed/несовместимому owner. Detach означает managed-closed, не право молча создать standalone engine. Fresh critic никогда не attach-нутый — standalone. Эти modes хранятся явно, nullptr недостаточно.

## 6. GPU operation и результат

Package-private native::acquire(ctx,batch,spec,lease,view): проверить token/domain/caps, reserve/drain growth, enqueue необходимые H2D ровно на owner stream и выдать pointers. View живёт только внутри lease, не передаётся CPU plugin. Все .cu entry связываются с configured device; не предполагается одинаковый OS thread у lifecycle и control calls.

Caller helper запускает только соответствующий ResultKind kernel на view.stream и передаёт launch status в native::complete. Complete enqueue D2H только в owned output staging, проверяет stream completion, выдаёт ResultToken лишь после успешного завершения. Native Lease move-only, один slot в provider, no per-acquire heap allocation. Complete потребляет lease при любом исходе; при failure outstanding storage остаётся owner-owned. Abandon/destructor lease прекращает публикацию результата и drain-ит уже queued work; не отменяет уже запущенный kernel магически.

| ResultKind | values / columns | collisions / all_collided |
|---|---|---|
| CostScoresFloat | rows=N, columns=1, N raw repulsive floats | N uint8 0/1 если track_collisions, иначе count=0; all_collided всегда 0/1 |
| CostCentersFloat | Q=1+(T-1)/point_step, N*Q floats, column-major | count=0; all_collided не используется |
| ObstacleCentersDouble | N*T floats, columns=T, double worldToMap | count=0; all_collided не используется |

point_step>0; для Cost не ограничивать T (step>T даёт Q=1). Если step не представим в existing int kernel parameter — UnsupportedShape/CPU, не truncation. Obstacle kind требует step=1. track_collisions разрешён только у CostScoresFloat. No operation при N=0 в managed Context: InvalidInput. Legacy CostCriticCuda::score_center zero-row fast path сохраняется отдельно (all_collided=true/no launch), а unsupported empty ObstaclesCritic input не становится поддержанным.

Consumer читает результат и выполняет прежнюю host weight/power/footprint/math ровно один раз, затем release_result. GPU failure до successful completion не меняет data.costs/fail/collision flags. CPU fallback пересчитывает только текущий critic с уже существующими previous contributions; OR collision semantics и порядок critic fail flags сохраняются.

## 7. CPU plugin barriers

ReadOnlyInputsV1 — marker обещания не менять map bytes/geometry и trajectory x/y/shape в пределах score. Изменения costs/fail/collision/path cache и своих parameter caches разрешены. GPU consumers наследуют marker. Встроенные read-only CPU critics получают marker **после source audit**; список/тесты входят в implementation plan. Class/plugin name не служит доказательством.

Неотмеченный CPU plugin исполняется как opaque: invalidate_inputs перед вызовом, после него только mark-needs-refresh в manager; snapshots обновляются lazy перед следующим реально requested GPU consumer. Не делать полный memcpy после каждого CPU critic, если больше GPU consumers нет. Disabled/unknown plugins нельзя пропустить: прежний score выполняется. Context concurrency/statistics не зависят от динамического marker plugin имени.

Для fully marked healthy sequence: карта H2D один раз на stable control scope, x/y один раз на batch. Opaque barrier, reset/growth/retry легально меняют counters. Старый unmarked binary CPU plugin остаётся functional с консервативным barrier; no claim hardware isolation от ошибочного native кода.

## 8. Status mapping и commit rules

| Code | Действие caller | Output validity |
|---|---|---|
| Ok | продолжить protocol | result valid только с issued ready token |
| NotCompiled / NoDevice / DriverUnavailable | CPU scope, logged once per configuration | no GPU result |
| CapacityExceeded / HostMemoryUnavailable / DeviceMemoryUnavailable | CPU-for-cycle, если отсутствие queued work или successful drain доказано | no partial result |
| UnsupportedShape | CPU-for-cycle только для valid CPU domain | no result |
| OperationFailedDrained / CpuForCycle | CPU текущего critic; без повторного device attempt в cycle | no result |
| InvalidLimits / InvalidInput | controlled ControllerException; не выполнять unsafe CPU scorer | no result |
| InvalidState / InvalidToken / OutstandingLease / ConcurrentUse | integration error, abort control; release/drain правильным owner до дальнейших операций | no result |
| Closed | managed CPU path либо lifecycle error по call site, не standalone recreation | no result |
| RestartRequired / DrainUnproven | stop GPU and control result, NoValidControl; retain unsafe owner storage | no result |

Status.stage указывает failing boundary, native_code — числовой vendor error или 0 без него. Code !=Ok никогда не публикует HostResult. ConcurrentUse обеспечивается try-entry guard без ожидания mutex; destruction параллельно API-вызову остаётся нарушением C++ lifetime, не обещанием memory-safe concurrent delete.

CUDA APIs могут вернуть предыдущую async error; произвольный non-success не классифицировать как обычный OOM. Allocation OOM fallback допустим до enqueue/после successful drain. После enqueue recoverable classification требует **cudaStreamSynchronize success** для этого stream; сброс last-error не является barrier. Unknown async errors классифицируются conservatively как fault. Destroy stream не заменяет proof completion: CUDA документирует отложенное освобождение. [NVIDIA stream API 12.4.1](https://docs.nvidia.com/cuda/archive/12.4.1/cuda-runtime-api/group__CUDART__STREAM.html).

Illegal address, launch timeout, assert и аналогичные process-inconsistent errors не восстанавливаются очисткой last-error; CUDA требует restart процесса для дальнейшей CUDA работы. Context возвращает RestartRequired, новые GPU calls того owner запрещены; underlying shared device failure может затронуть другие owners. [NVIDIA error types 12.4.1](https://docs.nvidia.com/cuda/archive/12.4.1/cuda-runtime-api/group__CUDART__TYPES.html).

**Предлагаемая fail-stop safety policy:** close возвращает DrainUnproven/RestartRequired, если безопасное освобождение не доказано; owner должен оставаться живым до coordinated process shutdown/recovery. Destructor делает best-effort close, но при unproven active DMA или live lease не освобождает memory с последующим продолжением: std::terminate как нарушение safe-destruction precondition. Это явное изменение fatal behavior experimental GPU helper path, не универсальный CPU-fallback. Supervisor restart orchestration вне Context; автоматические cudaDeviceReset/reset GPU/kill других процессов запрещены. В нормальном/recoverable пути все resources освобождаются RAII. Перед implementation policy должна попасть в review и death/fake-backend tests; реальные device-poisoning kernels на общей GPU не запускать.

R2 не обещает hard deadline для sync/close и не имеет stream-cancel timeout API: CUDA calls могут блокировать, а Async suffix сам по себе не гарантирует неблокирующее поведение. Product deadline/supervisor reaction — отдельная system requirement. [NVIDIA sync behavior 12.4.1](https://docs.nvidia.com/cuda/archive/12.4.1/cuda-runtime-api/api-sync-behavior.html).

## 9. Statistics и legacy migration

Stats — per owner, cumulative per configuration; read_stats после close возвращает last snapshot. Successful configure сбрасывает counters, reset не сбрасывает. counters uint64 saturate, не wrap. map/xy/result bytes и counts считаются для успешно **enqueued** copies; validity не выводится из counters. queued_launches отдельно от completed_launches (successful completion). allocations — device payload allocations, host_allocations — host payload allocations; growths — reserve plan changes; peaks — capacity sums, sync_nanoseconds — host waiting time. No sensitive pointer addresses or private source payload in diagnostics.

Timings: four float ms fields + valid; reused map/xy стадия имеет 0 ms; measure_timings=false выдаёт valid=false/zeros. Event-elapsed diagnostic failure после успешного data completion не меняет mathematical result, только timing_failures/valid flag. Comparison должен учитывать измерительный overhead и сопоставлять одинаковые режимы.

Существующие `*_cuda_compiled/device_available` и boolean helper methods сохраняют signatures. Helpers добавляют last_status()/read_stats() и делегируют engine; built-in adapters проверяют fatal status до CPU fallback. Legacy boolean direct users обязаны обновить handling для fatal safety policy.

Старые `*_cuda_launch_count/reset/last_timings` symbols сохраняются как **deprecated calling-thread diagnostics**: thread_local storage, saturating int completed count; reset_count не очищает timings. Это явное уточнение старого unsafe process-global behavior: вызов из другого thread не читает worker's latest operation. Никаких global locks/неатомарных global writes в hot path. В R2 без compiler deprecation attribute (не ломать -Werror downstream); @deprecated documentation + migration note. Все repository acceptance/benchmarks переходят на instance stats; legacy sequential same-thread test отдельно подтверждает совместимость, но не заменяет two-owner tests.

## 10. Test vectors и evidence plan

| Vector | Expected |
|---|---|
| M=40*40,N=2000,T=56, step=2, CostCentersFloat | 56000 floats, Q=28; map 1600 bytes, xy 896000 bytes per batch; payload estimate 1347604 bytes |
| two marked GPU critics, two iterations | healthy map_uploads=1, xy_uploads=2, separate precision kernels/results; no repeated allocation after warmup |
| point_step=100,T=56 | Q=1, valid; point_step=0 invalid; >INT_MAX valid unsupported shape → CPU |
| same pointers/sizes, changed map bytes next call | new control token + new map upload, no stale map |
| opaque CPU plugin between GPU consumers | invalidate/lazy refresh, stale batch token rejected |
| two contexts, copied old token, owner destroyed/reallocated same address | wrong/expired weak identity rejected, no ABA |
| null, bad count, nonfinite origin/resolution/x/y, product/cap overflow | InvalidInput/InvalidLimits or valid CapacityExceeded before enqueue; no truncation |
| float resolution casts to zero/nonfinite for Cost kind | InvalidInput; double mode remains separately validated |
| output lease not released before next acquire/end/close | OutstandingLease; cannot overwrite borrowed host result |
| fake post-launch error + drain success | OperationFailedDrained, untouched host costs, one current-critic CPU recompute |
| fake illegal-address/unproven drain | no HostResult; RestartRequired/DrainUnproven; fake backend proves no premature free, destructor death test |
| detach then score / reconfigure / repeated shutdown | no private CUDA engine resurrected; new attach required; old tokens invalid |
| legacy CPU plugin + OFF build | unchanged base/layout contract; no CUDA headers/dependency for OFF; conservative opaque barrier |

C++17/C++20 declaration syntax checks — artifact-level only; no link/runtime implementation tested. Future acceptance: complete OFF/ON build/CTest with actual cppcheck, float/double boundaries + score/flags/command parity, two owners, callback/lifecycle reconfigure (including validator prerequisite), fake fault injection, downstream plugin/relocation checks, ASan/UBSan/leak and device checks where available; measure matched 1000×5 controller workloads. No existing historical PASS is reused as proof of R2.

## 11. Files and next step

Header allocation/buffer/native bridge responsibilities remain in [architecture](architect-architecture.md). This contract adds read-only marker adoption in built-in critic headers and per-instance diagnostic migration to the affected scope. Public Context/marker headers define no CUDA types; native header stays package-private. No package/schema persistence migration, services, network auth or framing required; native plugin loading is trusted-process boundary, not isolation.

Next `writing-plans` must define reviewable implementation commits, integrate previously verified baseline/fixes into the common branch first, reserve/publish ABI-coherent builds and implement the declared tests. Default caps and fatal fail-stop are proposed engineering policy exposed for review; product timing/VRAM acceptance and supervisor restart integration remain explicitly outside this interface deliverable. No production code written by this step.
