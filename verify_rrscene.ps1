# verify_rrscene.ps1 — Quick machine check for CorridorPinch scene integrity
$scenePath = Join-Path $PSScriptRoot "roadrunner_scenes\CorridorPinch.rrscene"
$xodrPath  = Join-Path $PSScriptRoot "roadrunner_scenes\CorridorPinch.xodr"

Write-Host "=== RoadRunner Scene Verification Check ===" -ForegroundColor Cyan

if (-not (Test-Path $scenePath)) {
    Write-Host "[FAIL] Scene file not found at: $scenePath" -ForegroundColor Red
    exit 1
}

$fileItem = Get-Item $scenePath
$sizeBytes = $fileItem.Length
$sizeMB = $sizeBytes / 1MB
$hash = (Get-FileHash $scenePath -Algorithm SHA256).Hash

Write-Host "File Name:      $($fileItem.Name)"
Write-Host "File Size:      $sizeBytes bytes ($("{0:N4}" -f $sizeMB) MB)"
Write-Host "SHA256 Hash:    $hash"
Write-Host "Last Modified:  $($fileItem.LastWriteTime)"

if ($sizeMB -lt 0.05) {
    Write-Host "`n[NOTICE] RRSCENE_PLACEHOLDER: Scene size ($("{0:N4}" -f $sizeMB) MB) is below 0.05 MB threshold." -ForegroundColor Yellow
    Write-Host "         This indicates an empty/template scene. Canonical geometry relies on OpenDRIVE." -ForegroundColor Yellow
    
    if (Test-Path $xodrPath) {
        $xodrHash = (Get-FileHash $xodrPath -Algorithm SHA256).Hash
        Write-Host "         [OK] Canonical OpenDRIVE geometry found: $xodrPath" -ForegroundColor Green
        Write-Host "         OpenDRIVE SHA256: $xodrHash"
        exit 0
    } else {
        Write-Host "         [ERROR] Canonical OpenDRIVE (.xodr) asset missing!" -ForegroundColor Red
        exit 2
    }
} else {
    Write-Host "`n[PASS] RRSCENE_VALID: Populated scene detected ($("{0:N4}" -f $sizeMB) MB)." -ForegroundColor Green
    exit 0
}
