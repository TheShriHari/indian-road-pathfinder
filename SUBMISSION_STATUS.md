# SUBMISSION_STATUS.md
**SIH PS-26037 — Submission Session Status**  
**Session date:** 2026-09-30  
**Last updated:** 14:30 IST (T ≈ 570 min remaining)

---

## ✅ Phase 1 — Repo Safety [COMPLETE]

- `master` and `eval/overnight-batch` both pushed to `origin/TheShriHari/indian-road-pathfinder`
- Final commit pushed: `4827c8e` (truth repair) + subsequent commits
- Both branches synced via fast-forward merge — no divergence
- Remote backup confirmed: repo no longer exists on one machine only

---

## ✅ Phase 2 — Truth Repair [COMPLETE]

### Verified Numbers (authoritative — used everywhere downstream)
Source: `validation/final_submission_run/final_submission_run.csv` — 1,000 trials, seeds 1–1000, best_params `[1.50, 3.20, 0.07, 0.55, 2.90]`

| Metric | Verified Value | Source |
|---|---|---|
| Total trials | 1,000 | `final_submission_run.csv` |
| SUCCESS | 436 (43.6%) | same |
| SAFE_STOP | 470 (47.0%) | same |
| COLLISION | 42 (4.2%) | same |
| DEADLOCK | 28 (2.8%) | same |
| TIMEOUT | 23 (2.3%) | same |
| Scenario completion (SUCCESS+SAFE_STOP) | **90.6%** | computed |
| Mean replanning latency | **3.58 ms** | per-trial mean_latency_ms, averaged |
| P99 replanning latency | **41.21 ms** | 99th percentile of per-trial mean_latency_ms |
| Mean longitudinal jerk | **0.76 m/s³** | per-trial mean_jerk, averaged |
| Collision reduction vs. baseline | **56.25%** (96→42) | `final_submission_run.csv` + `batch_test_results_baseline.csv` |
| Control loop rate | **10 Hz** (dt = 0.1 s) | `run_single_scenario.m` line 231 |
| Tech stack | **100% MATLAB + Python, zero proprietary toolboxes** | repo contents |

### Files Repaired
| File | Fix |
|---|---|
| `README.md` | Full metrics table replaced with verified numbers (source-cited) |
| `SIH_PRESENTATION_TECHNICAL_OUTLINE.md` | Stage 4 (1000-trial batch) added; Stage 3 contextualized as 50-trial; Slide 9 corrected |
| `sih-demo-video/src/scenes/segment3/RigorKPIs.tsx` | Already correct in `ca0ef63` |
| `sih-demo-video/src/scenes/segment3/BugsFoundFixed.tsx` | Already correct in `ca0ef63` (tire-slip claim removed) |

---

## ✅ Phase 3 — Controller Defect Fix [COMPLETE + VERIFIED BY SMOKE TEST]

- `matlab/pure_pursuit_controller.m`: asymmetric jerk limiter ported from `run_single_scenario.m`
- Emergency braking (raw_accel < −2.0): **8.0 m/s³** allowance ✅
- Nominal deceleration: **0.95 m/s³** comfort limit ✅
- Throttle ramp: **0.95 m/s³** strictly enforced ✅
- Smoke test (`matlab/test_pure_pursuit_standalone.m`): **ALL 3 TESTS PASSED** (executed in MATLAB R2026a)
  - Emergency jerk: 8.000 m/s³ (exceeded 0.95 as designed)
  - Comfort decel: 0.950 m/s³ (at limit, enforced)
  - Throttle ramp: 0.950 m/s³ (strictly enforced)

---

## ✅ Phase 4 — RoadRunner Scenario [COMPLETE — PARAMETRIC; GUI PENDING]

**Honest status:** RoadRunner MATLAB API (`roadrunner()` function) is not available without the RoadRunner Scenario license active. A parametric kinematic fallback was used, clearly labeled as such.

### Deliverables produced:
- `roadrunner_scenes/corridor_pinch_trace_seed101.csv` — seed 101, width=2.40m, cattle agent
- `roadrunner_scenes/corridor_pinch_trace_seed102.csv` — seed 102, width=2.20m, cattle agent  
- `roadrunner_scenes/corridor_pinch_trace_seed103.csv` — seed 103, width=2.55m, rickshaw agent
- `roadrunner_scenes/build_corridor_pinch_scenario.m` — scenario script (runs real RR when API available)
- `roadrunner_scenes/GUI_SETUP_INSTRUCTIONS.md` — 10-min GUI checklist for base road

### Planner integration validation:
- Seed 101: corridor pinch at t=7.5s (width=2.40m) → **PASS: YIELD issued within 20 steps** ✅
- Seed 102: corridor pinch at t=7.3s (width=2.20m) → **PASS: YIELD issued within 20 steps** ✅
- Seed 103: width=2.55m (at threshold) → No squeeze triggered (boundary condition, expected) ✅

**Honest gap:** Traces are parametric (kinematic simulation), not from RoadRunner GUI+renderer. To get real RR traces: draft `CorridorPinch.rrscene` per `GUI_SETUP_INSTRUCTIONS.md`, then re-run `run_rr_scenario.bat`.

---

## ✅ Phase 5 — IDD Lite Integration [COMPLETE]

- Extracted `idd-lite.tar.gz` from `C:\Users\toshr\Downloads\` → `data/raw/idd_lite/`
- `data/raw/` added to `.gitignore` (dataset not tracked in repo)
- Smoke test: `idd_lite_smoke_test.py` executed successfully
- **1,607 label maps parsed, 116,732,480 pixels analyzed**
- Sample overlay image: `data/processed/idd_lite_sample_check.png` ✅
- Results JSON: `data/processed/idd_lite_smoke_results.json`

### Honest findings from IDD Lite:
IDD Lite (idd20k_lite) uses a **condensed 8-class label taxonomy** — not the full 40-class IDD taxonomy. Classes present in this subset:
- 0: road (37.7M px, 32.3%)
- 1: drivable_fallback (2.6M px, 2.2%)
- 2: sidewalk (1.6M px, 1.3%)
- 3: nondrivable (9.5M px, 8.1%)
- 4: other/background (13.2M px, 11.3%)
- **5: person** (30.0M px, **25.7%**) — ODD-relevant
- **6: rider** (22.2M px, **19.0%**) — ODD-relevant (motorcycles, scooters)
- 255: unlabeled (0.02%)

**No autorickshaw, cattle, truck, car, or bus class labels in this Lite split.** The Lite subset's condensed taxonomy merges vehicles into rider/person or excludes them. The `idd_mm_primary.zip` (Multimodal Primary, 6.5 GB) would contain these — but is explicitly out of scope for tonight.

**Honest claim:** "Real IDD Lite imagery confirms our ODD's high pedestrian/rider density (45% of labeled pixels). Vehicle-specific classes (autorickshaw, cattle) require the Multimodal Primary dataset — targeted for post-submission integration."

- `idd_mm_primary.zip` status: **NOT TOUCHED** (downloading in background, out of scope)

---

## ✅ Phase 6 — Subsystem Stubs [COMPLETE — CLEARLY LABELED AS STUBS]

Written as mock implementations to unblock integration testing. **Never committed under teammate names.** Clearly labeled as stubs in every file header.

- `matlab/subsystem_stubs/mock_tactical_decision.m` — Subsystem 2 interface stub
- `matlab/subsystem_stubs/mock_trajectory_output.m` — Subsystem 3 interface stub

**Honest gap:** Zero real teammate code exists for Subsystems 1–5. These stubs are placeholder interfaces authored by the integration harness, not finished teammate deliverables. Real implementation is the team's clearly identified next-step work.

---

## 🔲 Phase 7 — Video Re-render + Audio [EXCLUDED THIS SESSION]

Per user instruction, all video/Remotion work is excluded from this session.

**Known open defect:** `out/final_production_ready.mp4` (156s) has **zero audio streams** — confirmed by `ffprobe`. The SIH deliverable requirement explicitly requires audio/visual explanation. This must be resolved before portal submission.

---

## ✅ Phase 8 — Final Packaging [COMPLETE]

### Deliverables assembled:
| Artifact | Location | Status |
|---|---|---|
| Verified batch CSV | `validation/final_submission_run/final_submission_run.csv` | ✅ |
| Batch metrics JSON | `validation/final_submission_run/final_submission_run_metrics.json` | ✅ |
| Seed log | `validation/final_submission_run/seed_log.txt` | ✅ |
| BENCHMARK_REPORT.md | `BENCHMARK_REPORT.md` | ✅ (clean — no false claims) |
| RoadRunner traces | `roadrunner_scenes/corridor_pinch_trace_seed10[1-3].csv` | ✅ (parametric) |
| RoadRunner script | `roadrunner_scenes/build_corridor_pinch_scenario.m` | ✅ |
| IDD overlay image | `data/processed/idd_lite_sample_check.png` | ✅ |
| IDD results JSON | `data/processed/idd_lite_smoke_results.json` | ✅ |
| Controller smoke test | `matlab/test_pure_pursuit_standalone.m` | ✅ (3/3 PASSED) |
| Subsystem stubs | `matlab/subsystem_stubs/` | ✅ (clearly labeled) |
| This document | `SUBMISSION_STATUS.md` | ✅ |

---

## Honest Gaps — Open Before Portal Submission

| Gap | Severity | Notes |
|---|---|---|
| **Video has zero audio** | CRITICAL | `out/final_production_ready.mp4` confirmed silent. Add narration or captions before portal submission |
| RoadRunner real scene | MEDIUM | Parametric traces generated; needs GUI + RR API license to produce real rendered output |
| IDD vehicle classes | MEDIUM | Condensed Lite taxonomy lacks autorickshaw/cattle; requires Multimodal Primary (post-submission) |
| Subsystems 1–5 | LOW | Stubs written; real teammate implementations are next-step work |
| MATLAB 10-trial smoke test | LOW | Recommend running `run_batch_tests(10)` in MATLAB before portal submission to confirm pipeline integrity |
