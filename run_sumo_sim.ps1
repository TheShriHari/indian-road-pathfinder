<#
.SYNOPSIS
    Launches the Eclipse SUMO Indian Road simulation.
.EXAMPLE
    .\run_sumo_sim.ps1
    .\run_sumo_sim.ps1 -Headless
    .\run_sumo_sim.ps1 -Bridge
#>
param(
    [switch]$Headless = $false,
    [switch]$Bridge = $false
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$scriptPath = Join-Path $scriptDir "sumo_sim.py"

$argsList = @()
if ($Headless) {
    $argsList += "--headless"
} else {
    $argsList += "--gui"
}

if ($Bridge) {
    $argsList += "--bridge"
}

python $scriptPath @argsList
