& (Join-Path $PSScriptRoot "sync_to_pi.ps1")
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& (Join-Path $PSScriptRoot "sync_to_vm.ps1")
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
