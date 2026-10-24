# AGENTS.md — C++ / Embedded Accelerator Policy

These rules apply to every active skill and generated change. Repository-local policy, build scripts, CI, coding standards, and target constraints take precedence over generic accelerator guidance.

## Sources of Truth

1. **Enforcement:** CI, hooks, compiler diagnostics, linters, static analysis, hardware protection rules
2. **Repository policy:** this file and repository-local instructions
3. **Project profile:** `specs/project-profiler-profile.md` or an explicitly supplied equivalent
4. **Architecture and contracts:** files indexed by `specs/MANIFEST.md`
5. **Task documentation:** `tasks/TASK-NNN/`
6. **Skills and examples:** `.agents/skills/`, `.agents/references/`, `examples/`

When sources conflict, follow the highest applicable source and record the conflict.

## Tool Portability

- Canonical engineering procedures live in `.agents/skills/`; tool adapters MUST remain thin.
- Enabled integrations are recorded in `.accelerator.json`; the neutral package enables none by default.
- Cross-repository installation/update/uninstall MUST use `scripts/install-accelerator.py` and its `.accelerator-install.json` ownership record; do not overlay the release tree manually.
- Tool-specific files MUST be activated with `python3 scripts/configure-tools.py`; do not copy or delete whole adapter trees manually.
- Generated adapters MUST follow the current selection and be regenerated with `python3 scripts/sync-tool-adapters.py`; do not hand-maintain divergent copies.
- Configuration and synchronization MUST preserve conflicting/customized client files and every non-managed namespace.
- Tool-specific invocation syntax belongs in `TOOL-SUPPORT.md`, not in canonical skills or policy.
- Model selection is user/organization policy. Repository adapters MUST NOT pin a vendor model unless the project explicitly requires it.

## Project Discovery Before Modification

- MUST inspect the existing repository structure before proposing new directories or layers.
- MUST identify the project kind, host, target, architecture, compiler/toolchain, C++ standard, build system, test system, and hardware dependency.
- MUST read existing `CMakeLists.txt`, presets, toolchain files, CI, linker scripts, package manifests, and coding rules when present.
- MUST NOT assume CMake, Linux, exceptions, RTTI, the STL, dynamic allocation, POSIX, or a particular test framework.
- MUST preserve existing conventions unless a task explicitly changes them.

## GPU Development Branching

Project decision recorded on 2026-10-23:

- MUST use `feat/gpu-adaptation` as the shared integration branch for NVIDIA GPU adaptation.
- MUST create new GPU feature, optimization, and fix branches from `feat/gpu-adaptation`, using descriptive `feat/`, `perf/`, or `fix/` names.
- MUST integrate completed GPU changes back into `feat/gpu-adaptation` after the applicable `DOD.md` checks; each focused change belongs on its own branch.
- MUST preserve existing branches and their history; this convention does not authorize rebasing, deleting branches, pushing, or merging into `main`.
- The current target is desktop NVIDIA RTX 3060 (hardware clarification: 2026-10-24). Jetson adaptation is a future phase and is not a completion requirement for the desktop phase.

Example: `git switch feat/gpu-adaptation` followed by `git switch -c perf/mppi-transfer-reuse`. Creating that optimization branch from `main` would violate this convention.

Enforcement: check the current branch and base before implementation; review the integration target and applicable verification evidence before integration.

## File Naming and Documentation

- MUST prefix generated task/spec markdown files with the producing skill name: `{skill-name}-{purpose}.md`.
- Shared task control files `STATE.md` and `DEFERRED.md` are the only naming exceptions because multiple skills update them.
- MUST use zero-padded task directories: `tasks/TASK-001/`.
- MUST place living specifications directly in `specs/` and index them in `specs/MANIFEST.md`.
- MUST avoid duplicating decisions across specs; link to the source decision instead.
- SHOULD record the task baseline revision and pre-existing dirty/failing state in `STATE.md` before implementation.
- MUST put real but out-of-scope findings in `DEFERRED.md` instead of silently expanding the task.

## Evidence Provenance

- Factual claims in requirements, architecture, reviews, repository documentation, walkthroughs, and handoffs MUST be traceable to source, configuration, command output, test/measurement evidence, or an explicit human decision.
- Use `[VERIFIED]`, `[ASSUMED]`, and `[UNKNOWN]` when provenance is not already encoded by the artifact schema.
- Repository facts the agent can inspect are the agent's job to verify; ask the human only for decisions, inaccessible facts, or authorization.
- Generic knowledge packs are advisory. Repository policy, CI/build files, compiler/tool output, vendor documentation, and target evidence override them.

## Agent Behavior

- MUST execute only the invoked skill and then STOP.
- MUST NOT auto-chain skills or silently spawn agents.
- The scope of "the invoked skill" is set by what the triggering request actually asked for, not the skill's narrowest routine use. If the request already asked for both a diagnosis and a fix (or a fix and its verification), finishing that is completing the invoked skill's own scope, not chaining into a separate one — do not stop at a diagnosis and hand off a fix the request already asked for.
- MUST finish with `Context Summary` and `Next Steps`.
- MUST distinguish verified facts, assumptions, and unresolved constraints.
- MUST NOT claim target behavior from host-only evidence.

## C++ Correctness and Resource Safety

- MUST avoid undefined behavior, invalid lifetime, dangling references, out-of-bounds access, data races, and unchecked narrowing.
- MUST make ownership and lifetime explicit. Prefer RAII where the platform permits it.
- MUST release handles, descriptors, locks, mappings, sockets, DMA buffers, and device resources on every path.
- MUST handle partial initialization and partial failure safely.
- MUST respect repository policy for exceptions, RTTI, allocation, standard library usage, and language extensions.
- MUST validate sizes, ranges, alignment, endianness, packet lengths, and external data before use.

## Concurrency and Real-Time

When concurrency or timing matters:

- MUST document thread/task/ISR context and ownership of shared state.
- MUST check for blocking calls, unbounded work, priority inversion, lock ordering, and callbacks after object destruction.
- MUST NOT allocate, block, log heavily, or call non-ISR-safe APIs in interrupt context unless explicitly allowed.
- MUST treat latency, jitter, queue depth, stack usage, and execution-time limits as acceptance criteria when budgets exist.

## Embedded and Hardware Safety

When hardware is involved:

- MUST identify whether verification is host, simulator, SIL, target, or HIL.
- MUST NOT flash, erase, reset, power-cycle, modify fuses, write calibration/NVM, or access production hardware without explicit scope and a recoverable procedure.
- MUST preserve known-good firmware, configuration, and recovery instructions before destructive device operations.
- MUST verify register access, volatile semantics, barriers, alignment, packing, clock/reset dependencies, and peripheral initialization order where relevant.
- MUST state when required hardware or vendor tooling is unavailable.

## Security and Data Boundaries

- MUST NOT read or commit `.env`, credentials, private keys, tokens, production dumps, or `secrets/`.
- MUST treat client source, logs, dumps, symbols, tickets, generated context, and build artifacts as confidential unless an explicit project policy permits disclosure.
- MUST NOT upload client/repository material to public paste sites, gists, arbitrary MCP servers, external model endpoints, or third-party services without explicit authorization.
- MUST NOT inspect unrelated operator-home files, sibling client repositories, or external workspaces merely to gather context.
- MUST validate all untrusted boundaries: network packets, IPC, files, firmware images, command input, device data, and external APIs.
- MUST review integer/buffer safety, privilege boundaries, secure update/authentication, replay/downgrade risks, and sensitive logging when applicable.
- MUST NOT weaken certificate checks, signature validation, access controls, sandboxing, or secure boot without explicit approval.

## Verification

- MUST use `DOD.md` before claiming completion.
- MUST run applicable configure, build, test, static-analysis, sanitizer, target, and resource checks.
- MUST report exact commands and outcomes.
- MUST mark unsupported or unavailable checks as `NOT RUN` with a reason, not as PASS.
- MUST fix failures or hand off to the `systematic-debugger` skill; MUST NOT ignore them.
- MUST NOT bypass hooks with `--no-verify` or rewrite public history without explicit consent.

## Style

See `GOLDEN-PRINCIPLES.md`. Repository-local style wins over generic preferences.
