<#
.SYNOPSIS
    Launches Webots 3D Indian Rural Road Simulation.
.DESCRIPTION
    Launches webots_sim.py with optional flags for real-time viewing,
    standalone autonomous navigation, or MATLAB TCP co-simulation bridge.
.EXAMPLE
    .\run_webots_sim.ps1
    .\run_webots_sim.ps1 -Bridge
    .\run_webots_sim.ps1 -Mode fast
#>
param(
    [switch]$Bridge = $false,
    [string]$Mode = "realtime",
    [switch]$NoRendering = $false
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonScript = Join-Path $scriptDir "webots_sim.py"

$argsList = @("--mode=$Mode")
if ($Bridge) { $argsList += "--bridge" }
if ($NoRendering) { $argsList += "--no-rendering" }

Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "  Launching Webots 3D Indian Rural Road Simulation" -ForegroundColor Green
Write-Host "  Features: 1 Pothole, 2 Crossing Pedestrians, 1 Oncoming Auto-Rickshaw" -ForegroundColor Yellow
Write-Host "=========================================================================" -ForegroundColor Cyan

python $pythonScript @argsList
