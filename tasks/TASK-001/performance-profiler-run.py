#!/usr/bin/env python3
"""Run serial, fixed-input full-controller CPU/CUDA comparisons in the ROS container."""
import argparse
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--workspace", type=Path, required=True)
parser.add_argument("--results", type=Path)
parser.add_argument("--samples", type=int, default=1000)
parser.add_argument("--repetitions", type=int, default=5)
args = parser.parse_args()
if not 1 <= args.samples <= 100000 or not 1 <= args.repetitions <= 20:
    parser.error("samples/repetitions outside harness limits")
binary = args.workspace / "build-gpu/nav2_mppi_controller/benchmark/mppi_gpu_baseline"
results = args.results or (args.workspace / "results")
results.mkdir(exist_ok=True)
for critic in ("cost", "obstacles"):
    for footprint in ("0", "1"):
        for scene in ("free", "bands"):
            name = f"{critic}-fp{footprint}-{scene}"
            output = results / f"{name}.log"
            # Exclusive create preserves any previous evidence.
            # ROS/background diagnostics must not interleave with numeric stdout.
            with output.open("x") as stream, (results / f"{name}.stderr.log").open("x") as errors:
                completed = subprocess.run(
                    [str(binary), "pair", critic, footprint, scene,
                     str(args.samples), str(args.repetitions)],
                    stdout=stream, stderr=errors, check=False,
                    env={**os.environ, "RCUTILS_LOGGING_USE_STDOUT": "0"})
            summaries = [line for line in output.read_text().splitlines()
                         if line.startswith(("SUMMARY,", "PARITY,", "BASELINE_ERROR:"))]
            print(name, "exit", completed.returncode, flush=True)
            print("\n".join(summaries), flush=True)
            if completed.returncode:
                raise SystemExit(completed.returncode)
            rows = [line.split(",") for line in output.read_text().splitlines()]
            samples = [row for row in rows if row[0] == "SAMPLE"]
            assert len(samples) == 2 * args.samples * args.repetitions, name
            for row in samples:
                float(row[4])
            assert len([row for row in rows if row[0] == "PARITY"]) == args.repetitions, name
