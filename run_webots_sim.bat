@echo off
REM =============================================================================
REM  run_webots_sim.bat — Launch Webots 3D Indian Rural Road Simulation
REM  SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads
REM =============================================================================

setlocal enabledelayedexpansion

echo =============================================================================
echo   Launching Webots 3D Indian Rural Road Simulation
echo   Scenario: 1 Pothole, 2 Crossing Pedestrians, 1 Oncoming Auto-Rickshaw
echo   Camera  : Elevated Bird's Eye View along Road Centerline
echo =============================================================================

REM 1. Terminate any lingering/frozen Webots processes to free port 1234
echo [*] Checking and cleaning up lingering Webots processes...
taskkill /F /IM webots.exe /IM webots-bin.exe /IM webotsw.exe /IM webots-controller.exe >nul 2>&1
timeout /t 1 /nobreak >nul

set PROJECT_DIR=%~dp0
set WORLD_FILE=%PROJECT_DIR%webots\worlds\indian_rural_road.wbt

REM 2. Try launching via Python coordinator
where python >nul 2>&1
if %ERRORLEVEL% equ 0 (
    echo [*] Launching via Python simulator coordinator...
    python "%PROJECT_DIR%webots_sim.py" %*
    if %ERRORLEVEL% equ 0 goto done
    echo [!] Python launcher exited with code %ERRORLEVEL%, falling back to direct Webots launch...
)

REM 3. Fallback: Locate Webots directly
set WEBOTS_BIN=
if exist "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe" (
    set "WEBOTS_BIN=C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"
) else if exist "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" (
    set "WEBOTS_BIN=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
) else if defined WEBOTS_HOME (
    if exist "%WEBOTS_HOME%\msys64\mingw64\bin\webotsw.exe" set "WEBOTS_BIN=%WEBOTS_HOME%\msys64\mingw64\bin\webotsw.exe"
)

if defined WEBOTS_BIN (
    echo [*] Launching Webots GUI binary directly: "%WEBOTS_BIN%"
    set "WEBOTS_HOME=C:\Program Files\Webots"
    set "PYTHONPATH=%WEBOTS_HOME%\lib\controller\python;%PYTHONPATH%"
    set "PATH=%WEBOTS_HOME%\msys64\mingw64\bin;%WEBOTS_HOME%\lib\controller;%PATH%"
    start "" "%WEBOTS_BIN%" --mode=realtime "%WORLD_FILE%"
    echo [*] Webots 3D simulation window launched successfully!
    goto done
) else (
    echo [ERROR] Could not find Webots installation.
    echo Please make sure Webots R2025a is installed at C:\Program Files\Webots
    echo or set the WEBOTS_HOME environment variable.
    pause
    exit /b 1
)

:done
echo.
echo [*] Simulation process initiated.
