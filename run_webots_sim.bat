@echo off
REM =============================================================================
REM  run_webots_sim.bat — Launch Webots 3D Indian Rural Road Simulation
REM  SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads
REM =============================================================================

echo =============================================================================
echo   Launching Webots 3D Indian Rural Road Simulation
echo   Scenario: 1 Pothole, 2 Pedestrians Crossing, 1 Oncoming Auto-Rickshaw
echo =============================================================================

python "%~dp0webots_sim.py" %*
