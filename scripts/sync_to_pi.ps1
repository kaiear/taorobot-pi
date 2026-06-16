$envFile = Join-Path $PSScriptRoot "pi_env.ps1"

if (Test-Path $envFile) {
    . $envFile
} else {
    . (Join-Path $PSScriptRoot "pi_env.example.ps1")
    Write-Host "Using scripts/pi_env.example.ps1. Copy it to scripts/pi_env.ps1 and edit your Pi settings." -ForegroundColor Yellow
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$localSrc = Join-Path $repoRoot "src"
$sshTarget = "$PI_USER@$PI_HOST"
$sshArgs = @()

if ($PI_KEY -and (Test-Path $PI_KEY)) {
    $sshArgs += @("-i", $PI_KEY)
}

ssh @sshArgs $sshTarget "mkdir -p '$PI_SRC'"

Get-ChildItem -Path $localSrc -Directory -Filter "__pycache__" -Recurse -Force | Remove-Item -Recurse -Force

Get-ChildItem -Force $localSrc | ForEach-Object {
    if ($_.Name -eq "__pycache__") {
        return
    }
    scp @sshArgs -r $_.FullName "$sshTarget`:$PI_SRC/"
}

ssh @sshArgs $sshTarget "find '$PI_SRC' -type d -name __pycache__ -prune -exec rm -rf {} +"
