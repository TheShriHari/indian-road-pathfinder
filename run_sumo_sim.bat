@echo off
REM =========================================================================
REM  run_sumo_sim.bat — 1-Click Eclipse SUMO Autonomous Simulation Launcher
REM  SIH PS-26037: Adaptive Path Planning for Indian Roads
REM =========================================================================

echo =========================================================================
echo   Starting Eclipse SUMO Indian Road Autonomous Simulation...
echo =========================================================================

python "%~dp0sumo_sim.py" --gui %*
pause
