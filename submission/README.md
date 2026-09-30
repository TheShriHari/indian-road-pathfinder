# SIH PS-26037 Submission Deliverable Package

**Assembled:** 2026-09-30 16:01:08 +05:30

## Overview of Included Deliverables
1. **RoadRunner Scenario Models:** Canonical OpenDRIVE 1.6 (`.xodr`) networks for 5 Indian ODD scenarios + `CorridorPinch`.
2. **Benchmark Execution Traces:** 10 Hz kinematic traces (`*_parametric_trace.csv`) adhering to the canonical schema.
3. **Authoritative 1000-Trial Batch:** `final_submission_run.csv` providing verified metrics (90.6% completion, 3.58ms latency, 0.76 m/s³ jerk).
4. **Controller Verification:** `test_pure_pursuit_standalone.m` proving asymmetric jerk limiter operation (8.0 m/s³ emergency vs 0.95 m/s³ nominal).
5. **IDD Lite Perception:** Verification analysis of 1,607 label maps (116.7M pixels) documenting Indian road class distributions.
6. **Audit Guardrails:** `check-scene.ps1` and `CONTRIBUTING.md` enforcing `<file>:<line>` provenance.

See `manifest.txt` for exact SHA256 checksums of all packaged artifacts.
