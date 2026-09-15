# clone_repos.ps1 — Shallow-clone all reference open-source repos
# Run from project root: .\resources\clone_repos.ps1

$reposDir = Join-Path $PSScriptRoot "repos"
New-Item -ItemType Directory -Path $reposDir -Force | Out-Null

$repos = @(
    "https://github.com/CPFL/Autoware",
    "https://github.com/commaai/openpilot",
    "https://github.com/argoai/argoverse-api",
    "https://github.com/AtsushiSakai/PythonRobotics"
)

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "  SIH PS 26037 — Reference Repository Cloner" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""

foreach ($repoUrl in $repos) {
    $repoName = Split-Path $repoUrl -Leaf
    $destPath = Join-Path $reposDir $repoName

    if (Test-Path $destPath) {
        Write-Host "[SKIP]  $repoName — already cloned" -ForegroundColor Yellow
        continue
    }

    Write-Host "[CLONE] $repoName" -ForegroundColor Green
    Write-Host "        $repoUrl"
    git clone --depth 1 $repoUrl $destPath
    if ($LASTEXITCODE -eq 0) {
        Write-Host "[OK]    Cloned to $destPath`n" -ForegroundColor Green
    } else {
        Write-Host "[FAIL]  Could not clone $repoName`n" -ForegroundColor Red
    }
}

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "  Done. Repos in: $reposDir" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
