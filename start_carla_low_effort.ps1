<#
.SYNOPSIS
    Starts CARLA in Low-Graphics / Low-Effort mode to prevent D3D GPU crashes.
.DESCRIPTION
    Forces DirectX 11, quality-level=Low, 20 FPS cap, 800x600 window.
    Optionally supports -Headless for zero-display offscreen rendering.
.EXAMPLE
    .\start_carla_low_effort.ps1
    .\start_carla_low_effort.ps1 -Headless
#>
param(
    [switch]$Headless = $false,
    [int]$Fps = 20,
    [string]$Quality = "Low",
    [string]$CarlaPath = ""
)

Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "  CARLA Low-Graphics / Low-Effort Launcher (SIH PS-26037)" -ForegroundColor Cyan
Write-Host "  Prevents Unreal Engine D3D Device Lost (0x887A0020) crashes." -ForegroundColor Yellow
Write-Host "========================================================================="

$exe = $null

if ($CarlaPath -and (Test-Path $CarlaPath)) {
    $exe = $CarlaPath
} elseif ($env:CARLA_ROOT -and (Test-Path "$env:CARLA_ROOT\CarlaUE4.exe")) {
    $exe = "$env:CARLA_ROOT\CarlaUE4.exe"
} else {
    $candidates = @(
        ".\CarlaUE4.exe",
        "..\carla\CarlaUE4.exe",
        "..\CarlaUE4.exe",
        "C:\CARLA_0.9.15\CarlaUE4.exe",
        "C:\CARLA_0.9.14\CarlaUE4.exe",
        "C:\CARLA_0.9.13\CarlaUE4.exe",
        "C:\carla\CarlaUE4.exe",
        "D:\CARLA_0.9.15\CarlaUE4.exe",
        "D:\CARLA_0.9.14\CarlaUE4.exe",
        "D:\CARLA_0.9.13\CarlaUE4.exe",
        "D:\carla\CarlaUE4.exe"
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) {
            $exe = (Resolve-Path $c).Path
            break
        }
    }
}

if (-not $exe) {
    Write-Host "[INFO] CarlaUE4.exe not found in standard paths. Delegating to Python launcher..." -ForegroundColor Yellow
    $pyArgs = @("start_carla.py", "--fps", "$Fps", "--quality", "$Quality")
    if ($Headless) { $pyArgs += "--headless" }
    python @pyArgs
    exit $LASTEXITCODE
}

Write-Host "[FOUND] CarlaUE4 at: $exe" -ForegroundColor Green

$launchArgs = @("-dx11", "-quality-level=$Quality", "-benchmark", "-fps=$Fps")
if ($Headless) {
    $launchArgs += "-RenderOffScreen"
    Write-Host "[MODE] Running Headless (-RenderOffScreen)" -ForegroundColor Magenta
} else {
    $launchArgs += @("-windowed", "-ResX=800", "-ResY=600")
    Write-Host "[MODE] Running Windowed (800x600, $Fps FPS, Quality=$Quality)" -ForegroundColor Green
}

Write-Host "[LAUNCHING] $exe $($launchArgs -join ' ')" -ForegroundColor Cyan
& $exe @launchArgs
