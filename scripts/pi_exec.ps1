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

function Select-PiHost {
    $hosts = @()
    if ($PI_HOST_TAILSCALE) { $hosts += $PI_HOST_TAILSCALE }
    if ($PI_HOST_LAN) { $hosts += $PI_HOST_LAN }
    if ($PI_HOST -and ($hosts -notcontains $PI_HOST)) { $hosts += $PI_HOST }

    foreach ($candidate in $hosts) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        Write-Host "Testing Raspberry Pi SSH at $($candidate):22 ..."
        if (Test-NetConnection -ComputerName $candidate -Port 22 -InformationLevel Quiet -WarningAction SilentlyContinue) {
            Write-Host "Using Raspberry Pi host: $candidate"
            return $candidate
        }
    }

    throw "No reachable Raspberry Pi SSH host. Tried: $($hosts -join ', ')"
}

$PI_HOST = Select-PiHost
$sshTarget = "$PI_USER@$PI_HOST"

ssh @sshArgs $sshTarget $Command
