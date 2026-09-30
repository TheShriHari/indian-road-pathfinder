# RoadRunner Scenes: CorridorPinch

This directory contains road geometry, scene definitions, and trajectory logs for the CorridorPinch bottleneck scenario.

## Artifact Provenance & Current Status
- `CorridorPinch.rrscene`: Placeholder template (~6 KB). SHA256: `2305016F3988C0BFEE27A4FBE7D313BA24F932105C41AEA9A03565BDA13DD696`
- `CorridorPinch.xodr`: **Canonical deterministic road geometry** generated programmatically via OpenDRIVE 1.6 XML. SHA256: `10DD51391CFEE2CBE63287FE0BEC03FF4135346E97F3F80DFB7A36C952E9AD69`
- `CorridorPinch_initial_trajectory.csv`: Kinematic trajectory export for ego and crossing dynamic agent.

## Exact Reproduction Instructions

### Reproduce CorridorPinch.rrscene (manual via GUI):
1. Open RoadRunner R2026a (`C:\Program Files\RoadRunner R2026a\bin\win64\AppRoadRunner.exe`).
2. Select **File → Import → OpenDRIVE** and choose `roadrunner_scenes/CorridorPinch.xodr`.
3. Inspect road geometry (100m total length, 3.7m nominal lane, 0.3m shoulders, 3.2m pinched bottleneck at x=45m).
4. Add actors if desired (`Ego` at [0,0,0], `DynamicAgent_Rickshaw` crossing at x=45m).
5. Select **File → Save Scene** and overwrite `roadrunner_scenes/CorridorPinch.rrscene`.

### Programmatic reproduction (requires RoadRunner MATLAB connector & license):
1. Launch MATLAB with Automated Driving Toolbox and RoadRunner Scenario installed.
2. Run in MATLAB:
```matlab
projectRoot = 'C:\path\to\indian-road-pathfinder';
rr = roadrunner(projectRoot);
importScene(rr, 'roadrunner_scenes/CorridorPinch.xodr', 'OpenDRIVE', openDRIVEImportOptions());
saveScene(rr, 'roadrunner_scenes/CorridorPinch.rrscene');
```
3. Run verification:
```powershell
.\verify_rrscene.ps1
```
