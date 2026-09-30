# RoadRunner Scenes: Canonical Indian ODD Scenarios (SIH PS-26037)

This directory contains deterministic OpenDRIVE 1.6 (`.xodr`) road networks, parametric trajectory traces, and RoadRunner scene definitions for five core scenarios representing the Indian Operational Design Domain (ODD).

---

## 1. Canonical Scenarios Catalog

| Scenario | Road Length | Lane Width | Obstacle / Pinch Geometry | Dynamic Agent(s) | Primary Challenge |
|---|---|---|---|---|---|
| **VillageRoad** | 100.0 m | 3.7 m (nominal) $\to$ 2.4 m (pinch @ 45m) | Eroded dirt shoulders, severe corridor narrowing | Crossing cattle (`cattle_01`, 1.2 m/s) | Asymmetric jerk-limited braking, corridor bottleneck yield |
| **UrbanIntersection** | 120.0 m | 4.0 m $\to$ 3.2 m (pinch @ 60m) | Unsignalized crossroad, blind corner | Merging auto-rickshaw (`rickshaw_merge`, 3.5 m/s) | Informal merging negotiation, gap acceptance |
| **HighwayMerge** | 160.0 m | 4.2 m $\to$ 3.0 m (pinch @ 80m) | On-ramp merge zone | Slow-moving vehicle / tractor (`slow_tractor`, 2.2 m/s) | High-speed differential collision avoidance |
| **MarketDense** | 80.0 m | 3.2 m $\to$ 2.1 m (pinch @ 35m) | Narrow street, stationary vendors | Pedestrians (`pedestrian_vendor`, `shopper_crossing`, ~1.0 m/s) | Multi-agent creeping, high collision-risk negotiation |
| **CattleCrossing** | 110.0 m | 3.6 m $\to$ 2.2 m (pinch @ 55m) | Unmarked rural road, grazing boundary | Cattle herd (`lead_cow`, `trailing_calf`, 1.2–1.4 m/s) | Sudden intrusion safe stop, deadlock prevention |

---

## 2. Artifact Types for Each Scenario

For each scenario `<SceneName>`:
1. `<SceneName>.xodr` — **Canonical deterministic road geometry** generated programmatically in OpenDRIVE 1.6 XML.
2. `<SceneName>_parametric_trace.csv` — Deterministic kinematic trajectory trace log recorded at 10 Hz ($\Delta t = 0.1\,\text{s}$).
3. `<SceneName>_fallback_manifest.txt` — Machine-readable manifest declaring whether programmatic RoadRunner API or parametric fallback was executed.
4. `<SceneName>.rrscene` — Populated RoadRunner scene (generated via GUI import or programmatic API).

### Parametric Trace CSV Schema
Every trace log adheres strictly to the benchmark schema:
```text
time, ego_x, ego_y, ego_yaw, agent_id, agent_x, agent_y, collision_flag, replans, planning_latency_ms, control_latency_ms
```

---

## 3. Reproduction Instructions

### Programmatic Generation (MATLAB):
```matlab
cd('roadrunner_scenes');
% Build all 5 scenarios (attempts RoadRunner API if available, else logs parametric fallback)
make_rrscene_or_fallback();

% Or build a single scenario:
make_rrscene_or_fallback('VillageRoad');
```

### Manual GUI Import & Save (RoadRunner Desktop):
1. Launch RoadRunner: `C:\Program Files\RoadRunner R2026a\bin\win64\AppRoadRunner.exe`.
2. Select **File → Import → OpenDRIVE** and choose `roadrunner_scenes/<SceneName>.xodr`.
3. Verify road layout and pinch section.
4. Select **File → Save Scene** and save to `roadrunner_scenes/<SceneName>.rrscene`.
5. Run the integrity check:
   ```powershell
   .\check-scene.ps1 -SceneName <SceneName>
   ```
