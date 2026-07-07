# Open3D GUI runtime resources -> SLAM/resources
# Source: CMake build output (build/bin/resources)
# Exe lookup: SLAM/bin/{Config}/*.exe -> ../../resources (see Application.cpp FindResourcePath)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Src = Join-Path (Split-Path -Parent $Root) "build\bin\resources"
$Dst = Join-Path $Root "resources"

if (-not (Test-Path $Src)) {
    Write-Error "Open3D resources not found: $Src`nBuild Open3D first (cmake --build build --target GUI)."
}

New-Item -ItemType Directory -Force -Path $Dst | Out-Null
robocopy $Src $Dst /E /NFL /NDL /NJH /NJS /nc /ns /np | Out-Null
if ($LASTEXITCODE -ge 8) { exit $LASTEXITCODE }

$count = (Get-ChildItem $Dst -Recurse -File).Count
Write-Host "Copied $count files to $Dst"
