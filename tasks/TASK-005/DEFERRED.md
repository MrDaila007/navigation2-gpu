# R2 deferred scope and open constraints

- [UNKNOWN] Product deadlines/jitter, accepted memory budgets and full Nav2 workload profile remain unset. Contract caps are engineering defaults, not product acceptance.
- [VERIFIED historical] Device sanitizer attach and usable Nsight trace were unavailable during TASK-001; availability must be retested without privilege/driver changes, otherwise NOT RUN.
- [UNKNOWN] Dependency-overlay exact source identity, GPU CI, Jazzy/Lyrical runtime compatibility and genuinely toolkit-free environment remain distinct verification gaps; local Rolling results cannot close them.
- [VERIFIED scope] R3 full GPU scoring/footprint, R4 rollout/noise and R5 optimization math are outside R2. R2 alone does not remove one-critic CPU-produced x/y upload.
- [UNKNOWN] Supervisor restart orchestration, Jetson model/port, robot/SIL/HIL acceptance are future work. Fatal policy is tested with fakes; do not deliberately poison/reset the shared desktop GPU.
- [VERIFIED source] Validator loader replacement risk from TASK-002 is promoted to P0 prerequisite in the R2 plan. Other legacy test fixtures' direct lifecycle callbacks are not a blanket teardown rewrite; only failures affecting planned acceptance expand scope through explicit plan revision.

## P0 execution boundaries (2026-10-25)

- [VERIFIED source] Same live ParametersHandler across plugin replacement keeps existing parameter callbacks (setParamCallback ignores previously registered names), potentially referencing old plugin fields. P0 tests use the actual controller contract: shutdown, destroy handler, create a fresh one before initialize. Adding callback removal/registration tokens is not required for that contract and remains out of scope; revise the plan if later work requires same-handler replacement.
- [UNKNOWN] Failure inside a custom validator's initialize after callback registration and externally retained validator references beyond optimizer destruction need separate plugin-specific lifetime contracts. P0 covers real default-plugin creation failure/retry and normal controller cleanup, not arbitrary third-party callback ownership.
