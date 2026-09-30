# Contributing & Scientific Provenance Guidelines

## 🔒 Mandatory Numeric Claims Policy

To maintain scientific integrity across all deliverables for SIH PS-26037:

1. **Explicit File:Line Provenance:**
   - Every quantitative claim (percentages, frequencies, latencies, sample sizes, jerk values, collision counts) stated in `README.md`, slide decks, reports, or PR descriptions **must cite its exact source file and line number**:
     `[Source: <filepath>:<line_number>]`
   - *Example:* "90.6% scenario completion (436 SUCCESS + 470 SAFE_STOP) [Source: validation/final_submission_run/final_submission_run.csv:1-1001]"

2. **Authoritative Benchmark Data:**
   - The authoritative data source for all performance figures is:
     `validation/final_submission_run/final_submission_run.csv` (1,000 trials, seeds 1–1000).
   - Any modification or optimization to controller/planner code requires re-running the test batch script to produce fresh timestamped CSV logs before updating documentation.

3. **RoadRunner Scene Artifacts:**
   - `.rrscene` files smaller than 50 KB are treated as templates/placeholders (`RRSCENE_PLACEHOLDER`).
   - Canonical geometry is defined in `roadrunner_scenes/CorridorPinch.xodr`.
   - Any claim of a populated RoadRunner simulation must record a valid entry in `roadrunner_scenes/production_log.txt` with an updated SHA256 and production tool details.

---

## PR Review Checklist: Numeric Claims & Provenance

Before merging any Pull Request, verify:
- [ ] Does the PR introduce any numeric performance metrics?
- [ ] Is every metric traceable to an existing CSV, JSON, or test log file?
- [ ] Is the citation formatted as `<filepath>:<line>`?
- [ ] Does `powershell -ExecutionPolicy Bypass -File .\check-scene.ps1` exit with status 0?
- [ ] Are simulation fallback paths explicitly flagged if proprietary licenses are unavailable?
