param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Command
)

$envFile = Join-Path $PSScriptRoot "pi_env.ps1"

if (Test-Path $envFile) {
    . $envFile
} else {
    . (Join-Path $PSScriptRoot "pi_env.example.ps1")
    Write-Host "Using scripts/pi_env.example.ps1. Copy it to scripts/pi_env.ps1 and edit your Pi settings." -ForegroundColor Yellow
}

$sshTarget = "$PI_USER@$PI_HOST"
$sshArgs = @()

if ($PI_KEY -and (Test-Path $PI_KEY)) {
    $sshArgs += @("-i", $PI_KEY)
}

ssh @sshArgs $sshTarget $Command
