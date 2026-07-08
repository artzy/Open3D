# Global relocalization lost 시나리오 자동 self-test
# Usage: .\run_reloc_self_test.ps1 [-Profile low|medium|high]

param(
    [string]$Profile = "low"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Exe = Join-Path $PSScriptRoot "bin\Release\RealTimeSLAMRealSense.exe"
$LogOut = Join-Path $Root "Analysis\global_reloc_self_test_out.txt"

if (-not (Test-Path $Exe)) {
    Write-Error "Executable not found: $Exe`nBuild Release first."
}

Write-Host "Running global reloc self-test (profile=$Profile)..."
& $Exe --reloc_self_test --profile $Profile --global_reloc 1 2>&1 |
    Tee-Object -FilePath $LogOut

$code = $LASTEXITCODE
Write-Host "EXIT_CODE=$code"
if ($code -ne 0) {
    Write-Error "RELOC_SELF_TEST failed (exit $code). See $LogOut"
}
Write-Host "PASS — see $LogOut"
exit $code
