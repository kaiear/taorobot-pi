param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Command
)

$envFile = Join-Path $PSScriptRoot "vm_env.ps1"

if (Test-Path $envFile) {
    . $envFile
} else {
    . (Join-Path $PSScriptRoot "vm_env.example.ps1")
    Write-Host "Using scripts/vm_env.example.ps1. Copy it to scripts/vm_env.ps1 and edit your VM settings if needed." -ForegroundColor Yellow
}

$sshTarget = "$VM_USER@$VM_HOST"
$sshArgs = @()

if ($VM_KEY -and (Test-Path $VM_KEY)) {
    $sshArgs += @("-i", $VM_KEY)
}

ssh @sshArgs $sshTarget $Command
