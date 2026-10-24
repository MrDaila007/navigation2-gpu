# Full-controller CPU/CUDA baseline summary

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
