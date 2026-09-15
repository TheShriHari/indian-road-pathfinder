#!/usr/bin/env python3
"""
execution/verify_planner_fsm.py — Layer 2 Orchestrator for Phase 4.

Spawns headless MATLAB to execute matlab/test_planner_fsm.m,
monitors execution and exit codes, verifies .tmp/planner_metrics.json,
and logs outcomes strictly to .tmp/planner_test.log.
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

    log_file = tmp_dir / "planner_test.log"
    metrics_file = tmp_dir / "planner_metrics.json"

    print("==================================================================")
    print("  LAYER 2 ORCHESTRATION: Phase 4 Planning & FSM Verification      ")
    print("==================================================================")
    print(f"[ORCHESTRATOR] Root Directory: {root_dir}")
    print(f"[ORCHESTRATOR] Log File:       {log_file}")
    print(f"[ORCHESTRATOR] Metrics File:   {metrics_file}")

    if metrics_file.exists():
        metrics_file.unlink()

    cmd = [
        "matlab",
        "-batch",
        "run('matlab/test_planner_fsm.m')"
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
        print(f"[FAIL] Planner & FSM unit test suite failed with exit code {return_code}.")
        print(f"       Inspect log at {log_file} for stack trace.")
        sys.exit(return_code)

    if not metrics_file.exists():
        fallback_metrics = root_dir / "matlab" / ".tmp" / "planner_metrics.json"
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

    print("\n--- Verified Planner & FSM Metrics (.tmp/planner_metrics.json) ---")
    print(json.dumps(data, indent=2))

    all_passed = True

    # Check 1: Kinematic curvature valid
    if data.get("kinematic_curvature_valid") is not True:
        print("[CHECK FAILED] kinematic_curvature_valid is not True")
        all_passed = False
    else:
        print("[CHECK PASSED] kinematic_curvature_valid == True")

    # Check 2: Max curvature bound
    max_k = data.get("max_curvature_inv_m", 999.0)
    if max_k > 0.2222 + 1e-4:
        print(f"[CHECK FAILED] max_curvature_inv_m = {max_k} > 0.2222 m^-1")
        all_passed = False
    else:
        print(f"[CHECK PASSED] max_curvature_inv_m = {max_k} <= 0.2222 m^-1")

    # Check 3: Mean replanning latency
    mean_lat = data.get("mean_replanning_latency_ms", 999.0)
    if mean_lat >= 30.0:
        print(f"[CHECK FAILED] mean_replanning_latency_ms = {mean_lat} >= 30.0 ms")
        all_passed = False
    else:
        print(f"[CHECK PASSED] mean_replanning_latency_ms = {mean_lat} < 30.0 ms")

    # Check 4: P99 replanning latency
    p99_lat = data.get("p99_replanning_latency_ms", 999.0)
    if p99_lat >= 50.0:
        print(f"[CHECK FAILED] p99_replanning_latency_ms = {p99_lat} >= 50.0 ms")
        all_passed = False
    else:
        print(f"[CHECK PASSED] p99_replanning_latency_ms = {p99_lat} < 50.0 ms")

    # Check 5: FSM spatial hysteresis
    if data.get("fsm_hysteresis_verified") is not True:
        print("[CHECK FAILED] fsm_hysteresis_verified is not True")
        all_passed = False
    else:
        print("[CHECK PASSED] fsm_hysteresis_verified == True")

    # Check 6: Max longitudinal jerk
    max_jerk = data.get("max_longitudinal_jerk_mps3", 999.0)
    if max_jerk >= 1.0:
        print(f"[CHECK FAILED] max_longitudinal_jerk_mps3 = {max_jerk} >= 1.0 m/s^3")
        all_passed = False
    else:
        print(f"[CHECK PASSED] max_longitudinal_jerk_mps3 = {max_jerk} < 1.0 m/s^3")

    # Check 7: Max steering rate
    max_steer_rate = data.get("max_steering_rate_degps", 999.0)
    if max_steer_rate > 15.0 + 1e-3:
        print(f"[CHECK FAILED] max_steering_rate_degps = {max_steer_rate} > 15.0 deg/s")
        all_passed = False
    else:
        print(f"[CHECK PASSED] max_steering_rate_degps = {max_steer_rate} <= 15.0 deg/s")

    # Check 8: Status PASS
    if data.get("status") != "PASS":
        print(f"[CHECK FAILED] status = {data.get('status')} != PASS")
        all_passed = False
    else:
        print("[CHECK PASSED] status == PASS")

    if not all_passed:
        print("\n[RESULT] Verification failed on one or more Phase 4 criteria.")
        sys.exit(4)

    print("\n==================================================================")
    print("  [SUCCESS] All Phase 4 Planning & FSM Criteria Verified!         ")
    print("==================================================================")
    sys.exit(0)

if __name__ == "__main__":
    main()
