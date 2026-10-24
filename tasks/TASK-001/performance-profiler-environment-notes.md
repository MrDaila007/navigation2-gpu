# Additional baseline limitations

- No source changes after the final frame/footprint fixture build. Subsequent collector changes only separate numeric stdout from ROS diagnostics and enforce file validation.
- No figure is accepted from initial results/ (configuration error) or results-final/ (interleaved output). Only results-separated/ feeds the final analyzer.
- Compute Sanitizer could not attach; processes 78638 and 78631 from that diagnostic invocation were stopped by TERM. No privileges, GPU state or unrelated processes changed.
- Nsight Systems in the container generated nsys-cost.qdstrm but lacked importer. Host tool version is 2024.5.1; compatibility with 2023.4.4 intermediate data must be checked, not assumed.
- Dependency overlay from /ws/install is pre-existing, not rebuilt/pinned by this task. New MPPI package is isolated and source-hashed, but this is not yet a fully reproducible distribution image.
