$path = ".\roadrunner_scenes\CorridorPinch.rrscene"
if (-not (Test-Path $path)) { Write-Output "MISSING_RRSCENE"; exit 1 }
$sizeMB = (Get-Item $path).Length / 1MB
if ($sizeMB -lt 0.05) {
  Write-Output "RRSCENE_PLACEHOLDER - using parametric fallback (size ${sizeMB:N4} MB)"
  # Ensure fallback artifact exists:
  if (-not (Test-Path ".\roadrunner_scenes\CorridorPinch.xodr")) { Write-Output "MISSING_XODR"; exit 2 }
  exit 0  # success but flagged placeholder
} else {
  Write-Output "RRSCENE_VALID (size ${sizeMB:N4} MB)"
  exit 0
}
