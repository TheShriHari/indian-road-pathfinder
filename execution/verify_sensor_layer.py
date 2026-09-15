#!/usr/bin/env python3
"""
execution/verify_sensor_layer.py — Layer 2 Orchestrator for Phase 1.

Spawns headless MATLAB to execute matlab/test_sensor_layer.m,
monitors execution and exit codes, verifies .tmp/sensor_metrics.json,
and logs outcomes strictly to .tmp/sensor_test.log.
"""

import os
import sys
import json
import subprocess
from pathlib import Path

def main():
    root_dir = Path(__file__).resolve().parent.parent
    tmp_dir = root_dir / ".tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)

    log_file = tmp_dir / "sensor_test.log"
    metrics_file = tmp_dir / "sensor_metrics.json"

    print("==================================================================")
    print("  LAYER 2 ORCHESTRATION: Phase 1 Sensor Degradation Verification  ")
    print("==================================================================")
    print(f"[ORCHESTRATOR] Root Directory: {root_dir}")
    print(f"[ORCHESTRATOR] Log File:       {log_file}")
    print(f"[ORCHESTRATOR] Metrics File:   {metrics_file}")

    # Ensure stale metrics file is removed prior to test
    if metrics_file.exists():
        metrics_file.unlink()

    cmd = [
        "matlab",
        "-batch",
        "run('matlab/test_sensor_layer.m')"
    ]

    print(f"[ORCHESTRATOR] Executing command: {' '.join(cmd)}")

    try:
        with open(log_file, "w", encoding="utf-8") as f_log:
            proc = subprocess.Popen(
                cmd,
                cwd=str(root_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace"
            )

            # Stream output in real time and log to file
            for line in proc.stdout:
                sys.stdout.write(line)
                f_log.write(line)

            proc.wait()
            return_code = proc.returncode

    except FileNotFoundError:
        print("[ERROR] MATLAB executable ('matlab') not found on system PATH.")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] Exception occurred while executing MATLAB: {e}")
        sys.exit(1)

    print(f"\n[ORCHESTRATOR] MATLAB process exited with code: {return_code}")

    if return_code != 0:
        print(f"[FAIL] Unit test suite failed with non-zero exit code {return_code}.")
        print(f"       Inspect log at {log_file} for stack trace.")
        sys.exit(return_code)

    if not metrics_file.exists():
        fallback_metrics = root_dir / "matlab" / ".tmp" / "sensor_metrics.json"
        if fallback_metrics.exists():
            import shutil
            shutil.copyfile(fallback_metrics, metrics_file)
        else:
            print(f"[FAIL] Metrics file {metrics_file} was not generated.")
            sys.exit(2)


    try:
        with open(metrics_file, "r", encoding="utf-8") as f_metrics:
            data = json.load(f_metrics)
    except Exception as e:
        print(f"[FAIL] Failed to parse metrics JSON: {e}")
        sys.exit(3)

    print("\n--- Verified Sensor Metrics (.tmp/sensor_metrics.json) ---")
    print(json.dumps(data, indent=2))

    # Validate required exit criteria
    checks = [
        ("range_gate_passed", True),
        ("fov_gate_passed", True),
        ("noise_distribution_valid", True),
        ("fifo_delay_ticks", 2),
    ]

    all_passed = True
    for key, expected in checks:
        val = data.get(key)
        if val != expected:
            print(f"[CHECK FAILED] {key}: expected {expected}, got {val}")
            all_passed = False
        else:
            print(f"[CHECK PASSED] {key} == {expected}")

    if not all_passed:
        print("\n[RESULT] Verification failed on one or more criteria.")
        sys.exit(4)

    print("\n==================================================================")
    print("  [SUCCESS] All Phase 1 Perception Synthesis Criteria Verified!   ")
    print("==================================================================")
    sys.exit(0)

if __name__ == "__main__":
    main()
