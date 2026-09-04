@echo off
REM =============================================================================
REM  restart_carla_fresh.bat  —  Kill All CARLA Instances & Start Fresh (Low Graphics)
REM  SIH PS-26037: Adaptive Path Planning for Indian Roads
REM =============================================================================

echo =============================================================================
echo   [1/2] Terminating all running CARLA and bridge processes...
echo =============================================================================

REM Kill CarlaUE4 and shipping binaries
taskkill /F /IM CarlaUE4.exe /T >nul 2>&1
taskkill /F /IM CarlaUE4-Win64-Shipping.exe /T >nul 2>&1

REM Optional: wait a moment for GPU memory / ports to release
timeout /t 2 /nobreak >nul

echo   [DONE] Processes stopped. GPU resources and ports (2000, 2001, 2002) released.
echo.
echo =============================================================================
echo   [2/2] Starting CARLA fresh with Low-Graphics / Low-Effort flags...
echo =============================================================================

REM Call the low-effort launcher
call "%~dp0start_carla_low_effort.bat" %*
