# Rebuild record3d_cpp with /MT to match Open3DExample.props (static CRT).
# Run from repo root or any cwd.
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Join-Path $Root "3rdparty\record3d\CMakeLists.txt"))) {
    $Root = "d:\study\Open3D"
}
$Src = Join-Path $Root "3rdparty\record3d"
$Build = Join-Path $Src "build"

Write-Host "Configuring record3d (MultiThreaded CRT)..."
cmake -S $Src -B $Build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded `
    -DCMAKE_POLICY_DEFAULT_CMP0091=NEW
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Building record3d_cpp Release..."
cmake --build $Build --config Release --target record3d_cpp -j 8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Done: $(Join-Path $Build 'Release\record3d_cpp.lib')"
