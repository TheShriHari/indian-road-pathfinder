# RoadRunner GUI Setup — CorridorPinch Base Road
## 10-Minute Checklist (do this before running build_corridor_pinch_scenario.m)

### Launch RoadRunner
```
"C:\Program Files\RoadRunner R2026a\bin\win64\AppRoadRunner.exe"
```
Or: File → Open Project → select this repo as workspace.

---

### Step 1: Create New Scene (~3 min)
1. File → New Scene → Blank
2. Save As → `roadrunner_scenes/CorridorPinch.rrscene`

### Step 2: Draft the Road (~4 min)
1. Select **Road Plan Tool** (keyboard: `R`)
2. Click to place waypoints along a straight East-West road:
   - Start: `(0, 0)`, End: `(100, 0)`
   - Road width: **4.0 m** (set in Road Properties panel, right side)
3. In Road Properties:
   - **Lane count**: 1 (single undivided carriageway)
   - **Lane width**: 3.7 m (Indian rural standard, narrow end)
   - **Lane markings**: None (unmarked rural road)
   - **Shoulder**: enabled, width 0.3 m, material "Dirt/Gravel"
4. At x=45m, use the Road Deform tool to introduce a slight shoulder erosion:
   - Right-click road at x=45 → Add Point
   - Nudge shoulder boundary 0.3m inward on one side (simulates eroded edge)
   - This is the corridor pinch point

### Step 3: Add Ego Vehicle (~1 min)
1. Actors panel → Add Vehicle → Sedan (or Compact Car)
2. Place at `(0, 0, 0)`, heading East (yaw = 0°)
3. Name it `Ego`

### Step 4: Add Dynamic Agent (~2 min)
1. Actors panel → Add Vehicle → **Auto-Rickshaw** (or Animal if available)
   - If Auto-Rickshaw not in library: use Compact Car, rename to `DynamicAgent_Rickshaw`
2. Place at `(45, -4, 0)`, heading North (yaw = 90°)
3. In Actor Properties → Behavior → add waypoint at `(45, 4, 0)` (crosses road)
4. Set speed: **1.5 m/s** (slow crossing, as per ODD cattle behavior)

### Step 5: Save and Verify (~0 min)
- File → Save
- Confirm the file exists at `roadrunner_scenes/CorridorPinch.rrscene`

---

### Then run the scenario script:
```matlab
cd matlab  % or from repo root:
run('../roadrunner_scenes/build_corridor_pinch_scenario.m')
```

---

### Fallback (if GUI setup is not completed in time)
The script `build_corridor_pinch_scenario.m` includes a **parametric kinematic fallback**
that generates a physically plausible trace CSV without a real RoadRunner scene.
The fallback is clearly labeled in the output and in the trace CSV.
Honest claim: "RoadRunner scene authoring in progress; parametric trace generated pending full RR integration."
