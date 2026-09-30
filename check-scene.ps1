param(
    [string]$SceneName = ""
)

$scenarios = @("VillageRoad", "UrbanIntersection", "HighwayMerge", "MarketDense", "CattleCrossing", "CorridorPinch")
if ($SceneName -ne "") {
    $scenarios = @($SceneName)
}

$allPass = $true
foreach ($sc in $scenarios) {
    $rrscene = ".\roadrunner_scenes\$sc.rrscene"
    $xodr    = ".\roadrunner_scenes\$sc.xodr"
    
    if (Test-Path $rrscene) {
        $sizeMB = (Get-Item $rrscene).Length / 1MB
        if ($sizeMB -lt 0.05) {
            Write-Output "[$sc] RRSCENE_PLACEHOLDER - using parametric fallback (size ${sizeMB:N4} MB)"
            if (-not (Test-Path $xodr)) {
                Write-Output "[$sc] MISSING_XODR"
                $allPass = $false
            }
        } else {
            Write-Output "[$sc] RRSCENE_VALID (size ${sizeMB:N4} MB)"
        }
    } else {
        if (Test-Path $xodr) {
            Write-Output "[$sc] XODR_CANONICAL_FALLBACK (OpenDRIVE present, rrscene pending GUI export)"
        } else {
            Write-Output "[$sc] MISSING_GEOMETRY (Neither rrscene nor xodr found)"
            $allPass = $false
        }
    }
}

if ($allPass) {
    exit 0
} else {
    exit 1
}
