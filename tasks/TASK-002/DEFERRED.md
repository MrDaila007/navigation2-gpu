# Deferred / unverified follow-up

- [VERIFIED source, UNKNOWN runtime impact] Optimizer::initialize replaces validator_loader_ while trajectory_validator_ may still own an old plugin. Full optimizer cleanup/reconfigure, including callback lifetime and validator failure paths, requires its own regression before claiming complete lifecycle coverage. Current fix covers motion-model replacement/destruction and benchmark helper cleanup.
- [VERIFIED] Existing unrelated test helpers call costmap on_configure directly and may still emit lifecycle warnings at their own teardown. The new baseline fixture and new lifetime regressions use transitions and are verified independently; this task does not rewrite all legacy fixtures.
- [UNKNOWN] Device sanitizer / usable Nsight trace remain infrastructure gaps recorded in TASK-001; no CUDA kernels or container capabilities changed here.
- [UNKNOWN] Simulator/robot/HIL and controlled performance budgets still need separate verification. This step is correctness cleanup on the available RTX 3060.
