$envFile = Join-Path $PSScriptRoot "vm_env.ps1"

if (Test-Path $envFile) {
    . $envFile
} else {
    . (Join-Path $PSScriptRoot "vm_env.example.ps1")
    Write-Host "Using scripts/vm_env.example.ps1. Copy it to scripts/vm_env.ps1 and edit your VM settings if needed." -ForegroundColor Yellow
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$localSrc = Join-Path $repoRoot "src"
$sshTarget = "$VM_USER@$VM_HOST"
$sshArgs = @()

if ($VM_KEY -and (Test-Path $VM_KEY)) {
    $sshArgs += @("-i", $VM_KEY)
}

ssh @sshArgs $sshTarget "mkdir -p '$VM_SRC'"

Get-ChildItem -Path $localSrc -Directory -Filter "__pycache__" -Recurse -Force | Remove-Item -Recurse -Force

Get-ChildItem -Force $localSrc | ForEach-Object {
    if ($_.Name -eq "__pycache__") {
        return
    }
    scp @sshArgs -r $_.FullName "$sshTarget`:$VM_SRC/"
}

ssh @sshArgs $sshTarget "find '$VM_SRC' -type d -name __pycache__ -prune -exec rm -rf {} +"
