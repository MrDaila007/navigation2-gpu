# Deferred findings from desktop GPU baseline

- [VERIFIED] Enabling AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 reveals ten existing uninitvar diagnostics for state.local_path_length in test/critic_manager_test.cpp, test/utils_test.cpp and test/critics_tests.cpp. These files are unchanged by baseline harness work. Evidence: isolated CPU build ament_cppcheck/cppcheck.txt. Must fix or diagnose before declaring full DoD readiness; not suppressed by this task.
- [VERIFIED] Controller teardown prints class_loader unload-while-live warnings; helper nodes warn lifecycle not shut down. Teardown is outside measured calls. Need focused lifecycle/ownership diagnosis; no production lifecycle behavior changed for profiling.
- [VERIFIED] Existing executable paths are build/<package>/benchmark/<name>, not directly under build/<package>. Corrected newly maintained planning command references after actual build discovery.

- [VERIFIED] Compute Sanitizer 2024.1.1 could not attach inside existing container (No attachable process found; launch timeout). Its two diagnostic processes were terminated by explicit PID; no GPU reset or container privilege changes. Device memcheck NOT RUN.
- [VERIFIED] Container Nsight Systems 2023.4.4 emits qdstrm but lacks its importer; trace summary not yet available. CLI exit 0 is not evidence of successful report conversion.
- Measurement collector rule: numeric stdout and ROS stderr MUST be separate; force RCUTILS_LOGGING_USE_STDOUT=0 and validate every scenario before continuing. Prior mixed-stream datasets are diagnostic artifacts only, excluded from reported statistics. This is enforced in performance-profiler-run.py.

## Resolution update (historical findings above retained)
