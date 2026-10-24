#!/usr/bin/env python3
"""Validate raw fixed-input results and summarize paired CPU/CUDA runs."""
import argparse
import math
from pathlib import Path
import statistics

parser = argparse.ArgumentParser()
parser.add_argument("results", type=Path)
args = parser.parse_args()
print("| Scenario | CPU mean ms | CUDA mean ms | CPU/CUDA | CPU p99 ms | CUDA p99 ms | CPU process time change | Per-repeat speedup range |")
print("|---|---:|---:|---:|---:|---:|---:|---|")
for kind in ("cost", "obstacles"):
    for fp in ("0", "1"):
        for scene in ("free", "bands"):
            label = f"{kind}-fp{fp}-{scene}"
            rows = [line.split(",") for line in
                    (args.results / f"{label}.log").read_text().splitlines()]
            summaries = [row for row in rows if row[0] == "SUMMARY"]
            parity = [row for row in rows if row[0] == "PARITY"]
            assert len(summaries) == 10 and len(parity) == 5, label
            assert all(row == ["PARITY", "command_bits", "1050", "PASS"]
                       for row in parity), label
            grouped = {}
            for backend in ("cpu", "cuda"):
                by_rep = sorted([row for row in summaries if row[1] == backend],
                                key=lambda row: int(row[2]))
                assert [int(row[2]) for row in by_rep] == list(range(5)), label
                samples = []
                for row in by_rep:
                    data = [entry for entry in rows if entry[0] == "SAMPLE"
                            and entry[1:3] == row[1:3]]
                    assert len(data) == 1000 and int(row[3]) == 1000, label
                    assert [int(entry[3]) for entry in data] == list(range(1000)), label
                    values = [float(entry[4]) for entry in data]
                    assert all(math.isfinite(value) and value > 0 for value in values), label
                    assert math.isclose(statistics.mean(values), float(row[4]), rel_tol=1e-9)
                    assert int(row[10]) == (2100 if backend == "cuda" else 0), label
                    samples.extend(values)
                grouped[backend] = (sorted(samples), by_rep)
            cpu, cpu_rows = grouped["cpu"]
            gpu, gpu_rows = grouped["cuda"]
            cpu_mean, gpu_mean = statistics.mean(cpu), statistics.mean(gpu)
            cpu_time = statistics.mean(float(row[9]) for row in cpu_rows)
            gpu_time = statistics.mean(float(row[9]) for row in gpu_rows)
            p99 = math.ceil(len(cpu) * 0.99) - 1
            ratios = [float(c[4]) / float(g[4]) for c, g in zip(cpu_rows, gpu_rows)]
            print(f"| {label} | {cpu_mean / 1000:.3f} | {gpu_mean / 1000:.3f} | "
                  f"{cpu_mean / gpu_mean:.3f}x | {cpu[p99] / 1000:.3f} | "
                  f"{gpu[p99] / 1000:.3f} | {(gpu_time / cpu_time - 1) * 100:+.1f}% | "
                  f"{min(ratios):.3f}..{max(ratios):.3f}x |")
print("\nPASS: 80,000 raw measured calls; 42,000 paired command comparisons including warmup; expected launches verified.")
