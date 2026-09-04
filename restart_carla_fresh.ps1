<#
.SYNOPSIS
    Kills all existing CARLA processes and launches a fresh instance with low graphics.
.DESCRIPTION
    Stops CarlaUE4.exe and CarlaUE4-Win64-Shipping.exe, waits for port release,
    and runs start_carla_low_effort.ps1.
.EXAMPLE
    .\restart_carla_fresh.ps1
    .\restart_carla_fresh.ps1 -Headless
#>
param(
    [switch]$Headless = $false,
    [int]$Fps = 20,
    [string]$Quality = "Low",
    [string]$CarlaPath = ""
)

Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "  [1/2] Terminating all active CARLA simulator processes..." -ForegroundColor Yellow
Write-Host "========================================================================="

$procNames = @("CarlaUE4", "CarlaUE4-Win64-Shipping")
foreach ($name in $procNames) {
    $running = Get-Process -Name $name -ErrorAction SilentlyContinue
    if ($running) {
        Write-Host "Killing $name (PID(s): $($running.Id -join ', '))..." -ForegroundColor DarkYellow
        Stop-Process -Name $name -Force -ErrorAction SilentlyContinue
    }
}

Start-Sleep -Seconds 2

Write-Host "  [OK] CARLA stopped. VRAM and TCP ports 2000-2002 released." -ForegroundColor Green
Write-Host ""
Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "  [2/2] Launching fresh CARLA in Low-Graphics / Low-Effort Mode..." -ForegroundColor Cyan
Write-Host "========================================================================="

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$launchScript = Join-Path $scriptDir "start_carla_low_effort.ps1"

$params = @{
    Fps = $Fps
    Quality = $Quality
}
if ($Headless) { $params["Headless"] = $true }
if ($CarlaPath) { $params["CarlaPath"] = $CarlaPath }

& $launchScript @params
