<#
.SYNOPSIS
    Runs the 10 Rural Road Scenario Combinations in Eclipse SUMO.
.EXAMPLE
    .\run_rural_10_trials.ps1
    .\run_rural_10_trials.ps1 -Headless
    .\run_rural_10_trials.ps1 -Trial 3
#>
param(
    [switch]$Headless = $false,
    [int]$Trial = 0
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$scriptPath = Join-Path $scriptDir "sumo_rural_batch.py"

$argsList = @()
if ($Headless) {
    $argsList += "--headless"
} else {
    $argsList += "--gui"
}

if ($Trial -gt 0) {
    $argsList += "--trial"
    $argsList += "$Trial"
}

python $scriptPath @argsList
