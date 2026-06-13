$envFile = Join-Path $PSScriptRoot "pi_env.ps1"

if (Test-Path $envFile) {
    . $envFile
} else {
    . (Join-Path $PSScriptRoot "pi_env.example.ps1")
    Write-Host "Using scripts/pi_env.example.ps1. Copy it to scripts/pi_env.ps1 and edit your Pi settings." -ForegroundColor Yellow
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$localSrc = Join-Path $repoRoot "src"

ssh "$PI_USER@$PI_HOST" "mkdir -p '$PI_SRC'"

Get-ChildItem -Force $localSrc | ForEach-Object {
    scp -r $_.FullName "$PI_USER@$PI_HOST`:$PI_SRC/"
}
