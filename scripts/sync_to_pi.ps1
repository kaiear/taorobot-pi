$envFile = Join-Path $PSScriptRoot "pi_env.ps1"

if (Test-Path $envFile) {
    . $envFile
} else {
    . (Join-Path $PSScriptRoot "pi_env.example.ps1")
    Write-Host "Using scripts/pi_env.example.ps1. Copy it to scripts/pi_env.ps1 and edit your Pi settings." -ForegroundColor Yellow
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$localSrc = Join-Path $repoRoot "src"
$sshArgs = @(
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=8",
    "-o", "ConnectionAttempts=1"
)

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

ssh @sshArgs $sshTarget "mkdir -p '$PI_SRC'"

Get-ChildItem -Path $localSrc -Directory -Filter "__pycache__" -Recurse -Force | Remove-Item -Recurse -Force

Get-ChildItem -Force $localSrc | ForEach-Object {
    if ($_.Name -eq "__pycache__") {
        return
    }
    scp @sshArgs -r $_.FullName "$sshTarget`:$PI_SRC/"
}

ssh @sshArgs $sshTarget "find '$PI_SRC' -type d -name __pycache__ -prune -exec rm -rf {} +"
