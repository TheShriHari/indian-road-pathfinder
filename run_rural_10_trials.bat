@echo off
REM =========================================================================
REM  run_rural_10_trials.bat — 1-Click Rural 10-Combination SUMO Simulation
REM  SIH PS-26037: Adaptive Path Planning for Indian Roads
REM =========================================================================

echo =========================================================================
echo   Running 10 Rural Road Scenario Combinations in Eclipse SUMO...
echo   (2 Crossing Pedestrians, 1 Pothole, 1 Oncoming Auto-Rickshaw)
echo =========================================================================

python "%~dp0sumo_rural_batch.py" --gui %*
pause
