@echo off
REM =========================================================================
REM  start_carla_low_effort.bat  —  CARLA Simulator Low-Effort / Low-Graphics Mode
REM  SIH PS-26037: Adaptive Path Planning for Indian Roads
REM =========================================================================

echo =========================================================================
echo   Starting CARLA in Low-Graphics / Low-Effort Mode
echo   (Forces DX11, Quality=Low, 20 FPS, 800x600 Window)
echo   Prevents Unreal Engine D3D Device Lost (0x887A0020) GPU crashes.
echo =========================================================================

REM 1. Check if CARLA_ROOT environment variable is set
if exist "%CARLA_ROOT%\CarlaUE4.exe" (
    set "CARLA_EXE=%CARLA_ROOT%\CarlaUE4.exe"
    goto :LAUNCH
)

REM 2. Check local directories
if exist "CarlaUE4.exe" (
    set "CARLA_EXE=CarlaUE4.exe"
    goto :LAUNCH
)
if exist "..\carla\CarlaUE4.exe" (
    set "CARLA_EXE=..\carla\CarlaUE4.exe"
    goto :LAUNCH
)
if exist "..\CarlaUE4.exe" (
    set "CARLA_EXE=..\CarlaUE4.exe"
    goto :LAUNCH
)

REM 3. Check typical C: and D: drive paths
for %%D in (
    "C:\CARLA_0.9.15\CarlaUE4.exe"
    "C:\CARLA_0.9.14\CarlaUE4.exe"
    "C:\CARLA_0.9.13\CarlaUE4.exe"
    "C:\carla\CarlaUE4.exe"
    "D:\CARLA_0.9.15\CarlaUE4.exe"
    "D:\CARLA_0.9.14\CarlaUE4.exe"
    "D:\CARLA_0.9.13\CarlaUE4.exe"
    "D:\carla\CarlaUE4.exe"
) do (
    if exist %%D (
        set "CARLA_EXE=%%~D"
        goto :LAUNCH
    )
)

echo.
echo [INFO] CarlaUE4.exe was not detected in standard directories.
echo Using python launcher to perform deep search...
python "%~dp0start_carla.py" %*
goto :EOF

:LAUNCH
echo [FOUND] CarlaUE4 at: %CARLA_EXE%
echo [LAUNCHING] With low-effort flags...
"%CARLA_EXE%" -dx11 -quality-level=Low -benchmark -fps=20 -windowed -ResX=800 -ResY=600 %*
