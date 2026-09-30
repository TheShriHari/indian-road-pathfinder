# SUBMISSION_STATUS.md
**SIH PS-26037 — Submission Session Status**  
**Session date:** 2026-09-30  
**Last updated:** 14:14 IST (T ≈ 586 min remaining)

---

## ✅ Phase 1 — Repo Safety [COMPLETE]
- Both `master` and `eval/overnight-batch` pushed to `origin/TheShriHari/indian-road-pathfinder`
- Both branches at commit `ca0ef63` (and now updated further with truth repair commits below)
- Branches are identical — no divergence
- Fresh-clone smoke test: NOTE — 10-trial run_batch_tests smoke test deferred until MATLAB session is available; branch integrity confirmed via hash comparison of key files instead
- **Risk:** No single point of failure — repo now exists on remote

---

## ✅ Phase 2 — Truth Repair [COMPLETE]

### Verified Numbers (authoritative, used everywhere downstream)
All numbers from `validation/final_submission_run/final_submission_run.csv`:

| Metric | Verified Value | Source |
|---|---|---|
| Total trials | 1,000 | `final_submission_run.csv` |
| SUCCESS | 436 (43.6%) | same |
| SAFE_STOP | 470 (47.0%) | same |
| COLLISION | 42 (4.2%) | same |
| DEADLOCK | 28 (2.8%) | same |
| TIMEOUT | 23 (2.3%) | same |
| Scenario completion (SUCCESS+SAFE_STOP) | **90.6%** | computed from above |
| Mean replanning latency | **3.58 ms** | per-trial mean_latency_ms, averaged |
| P99 replanning latency | **41.21 ms** | 99th percentile of per-trial mean_latency_ms |
| Mean longitudinal jerk | **0.76 m/s³** | per-trial mean_jerk, averaged |
| Collision reduction vs. baseline | **56.25%** (96 → 42) | `final_submission_run.csv` + `batch_test_results_baseline.csv` |
| Control loop rate | **10 Hz** (dt = 0.1 s) | `run_single_scenario.m` line 231 |
| Tech stack | **100% MATLAB + Python, zero proprietary toolboxes** | repo contents |

### Files Fixed
| File | What was wrong | What was fixed |
|---|---|---|
| `README.md` | `100%` scenario completion; `12.4–16.8ms` latency; `0.18 m/s³` jerk; `1.2–2.4m` clearance — none traceable to verified CSV | Full metrics table replaced with verified numbers, traceable to `final_submission_run.csv` |
| `SIH_PRESENTATION_TECHNICAL_OUTLINE.md` | Slide 9 referenced `94.0%` + `0 collisions` as primary result; `$10–25ms$ latency` unverified | Added Stage 4 block with 1000-trial batch numbers; Slide 9 now shows authoritative batch figures |
| `sih-demo-video/src/scenes/segment3/RigorKPIs.tsx` | (already fixed in prior commit `ca0ef63`) | ✅ Already correct |
| `sih-demo-video/src/scenes/segment3/BugsFoundFixed.tsx` | (already fixed in prior commit `ca0ef63`) | ✅ Already correct |

### Fresh Re-run Status
- `validation/final_submission_run/final_submission_run.csv` exists (1001 rows, 1000 trials)
- Hash verified distinct from `batch_test_results_verified.csv`
- Outcome counts cross-checked via PowerShell: 436 SUCCESS / 470 SAFE_STOP / 42 COLLISION / 28 DEADLOCK / 23 TIMEOUT ✅
- **NOTE:** This CSV was generated prior to this session (timestamped in commit `ca0ef63`). A fresh MATLAB re-run has not been executed in this session due to MATLAB not being invoked interactively. The CSV is treated as the authoritative source since it was logged with seed_log.txt and is distinct from the audit's `batch_test_results_verified.csv`. If time allows before submission, a 10-trial smoke test via MATLAB should be run to confirm the pipeline still executes cleanly.

---

## ✅ Phase 3 — Controller Defect Fix [COMPLETE]

- `matlab/pure_pursuit_controller.m` now contains full asymmetric jerk limiter (committed in `ca0ef63`)
- Emergency braking (raw_accel < -2.0 or v_ref ≤ 0.05): **8.0 m/s³** jerk allowance
- Nominal deceleration: **0.95 m/s³** comfort limit
- Positive delta-a (throttle/brake release): **0.95 m/s³** strictly enforced
- Anti-chatter brake latch: 0.50s dwell timer
- Smoke test script written: `matlab/test_pure_pursuit_standalone.m` (3 test cases)
- **NOTE:** Smoke test has not been executed in MATLAB yet — it tests the logic by calling `pure_pursuit_controller.m` directly. Execution pending MATLAB invocation.

---

## 🔲 Phase 4 — RoadRunner Scene(s) [NOT STARTED]
- Hard cap applies
- Target: 1 scene, `CorridorPinch.rrscene`, scripted cattle/rickshaw agent
- Fallback: honest screenshot + "in-progress" note if cap hit

## 🔲 Phase 5 — IDD Lite Integration [NOT STARTED]
- `idd-lite.tar.gz` already in Downloads — no download needed
- Target: extract, tally classes, produce `data/processed/idd_lite_sample_check.png`
- **DO NOT TOUCH** `idd_mm_primary.zip` (downloading in background, out of scope)

## 🔲 Phase 6 — Subsystem Stubs [NOT STARTED — optional]
- Skip if Phases 4+5 consume their caps

## 🔲 Phase 7 — Video Re-render + Audio [NOT STARTED — non-negotiable]
- EXCLUDED from this session per user instruction (ignore all video/Remotion work)

## 🔲 Phase 8 — Final Packaging [IN PROGRESS]
- `SUBMISSION_STATUS.md` (this file) — being written
- Need: final git push + fresh-clone check

---

## Deferred / Out of Scope This Session
- **Video re-render and audio fix** — explicitly excluded by user. The current `out/final_production_ready.mp4` (156s) has **zero audio streams** — this is a known open defect that must be addressed before actual SIH portal submission.
- **Full MATLAB re-run** — `final_submission_run.csv` used as authoritative; recommend a 10-trial smoke test when MATLAB is available
- **IDD Multimodal Primary** (`idd_mm_primary.zip`, 6.5 GB) — downloading in background, out of scope tonight

---

## Honest Gaps Remaining
1. `out/final_production_ready.mp4` has **no audio** — deliverable requirement explicitly requires audio explanation
2. Zero RoadRunner scenes exist yet (Phase 4 not started)
3. IDD Lite extraction not done yet (Phase 5 not started)
4. Subsystem stubs for teammates: zero (Phase 6 not started)
5. MATLAB smoke test for controller fix not yet executed
