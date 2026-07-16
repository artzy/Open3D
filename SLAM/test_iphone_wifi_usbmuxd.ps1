# iPhone SLAM Wi-Fi 연결 테스트 (무선 usbmuxd)
#
# 중요:
# - Record3D의 "Wi-Fi Streaming"(WebRTC)은 공식 C++ 라이브러리가 USB 전용이라
#   현재 IPhoneSLAMRealSense.exe가 직접 받지 않습니다.
# - Wi-Fi에서도 ARKit 포즈를 쓰려면 Record3D는 "USB Streaming"을 켠 채
#   Apple "네트워크를 통해 연결"(무선 usbmuxd)로 터널링해야 합니다.
#
# 사전 준비 (한 번):
# 1) USB로 iPhone 연결 → iTunes/Apple Devices에서 해당 기기
#    "Wi-Fi를 통해 이 iPhone과 동기화/연결" 체크
# 2) 케이블 분리 후에도 기기 아이콘이 남아 있는지 확인
# 3) PC와 iPhone이 같은 Wi-Fi (핫스팟 USB 테더는 Wi-Fi 테스트가 아님)
# 4) Record3D: Settings → Live RGBD → USB Streaming ON → Record 시작
#
# 사용:
#   .\test_iphone_wifi_usbmuxd.ps1
#   .\test_iphone_wifi_usbmuxd.ps1 -Seconds 60 -PoseSource fused

param(
    [int]$Seconds = 90,
    [string]$PoseSource = "fused",
    [string]$Device = "CUDA:0"
)

$ErrorActionPreference = "Stop"
$ExeDir = Join-Path $PSScriptRoot "bin\Release"
$Exe = Join-Path $ExeDir "IPhoneSLAMRealSense.exe"
$Analysis = Join-Path (Split-Path $PSScriptRoot -Parent) "Analysis"
New-Item -ItemType Directory -Force -Path $Analysis | Out-Null

if (-not (Test-Path $Exe)) {
    Write-Error "Not found: $Exe — build IPhoneSLAMRealSense Release first."
}

function Get-AppleUsbPresent {
    $devs = Get-PnpDevice -ErrorAction SilentlyContinue |
        Where-Object { $_.FriendlyName -match 'Apple iPhone|Apple Mobile Device USB' -and $_.Status -eq 'OK' }
    return [bool]$devs
}

Write-Host "=== iPhone Wi-Fi (wireless usbmuxd) SLAM test ==="
Write-Host "Apple USB present: $(Get-AppleUsbPresent)"

if (Get-AppleUsbPresent) {
    Write-Host ""
    Write-Host "지금 USB(또는 USB 테더 이더넷)로 연결되어 있습니다."
    Write-Host "Wi-Fi 테스트를 위해:"
    Write-Host "  1) iTunes에서 'Wi-Fi로 연결' 활성화"
    Write-Host "  2) USB 케이블을 뽑으세요 (Apple Mobile Device Ethernet도 사라져야 함)"
    Write-Host "  3) Record3D USB Streaming + Record 유지"
    Write-Host "케이블을 뽑으면 자동으로 계속합니다 (최대 120초 대기)..."
    $deadline = (Get-Date).AddSeconds(120)
    while ((Get-Date) -lt $deadline) {
        if (-not (Get-AppleUsbPresent)) {
            Write-Host "USB 장치 사라짐 — 무선 usbmuxd로 시도합니다."
            Start-Sleep -Seconds 3
            break
        }
        Start-Sleep -Seconds 2
    }
    if (Get-AppleUsbPresent) {
        Write-Error "USB가 여전히 감지됩니다. 케이블을 분리한 뒤 다시 실행하세요."
    }
}

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$out = Join-Path $Analysis "iphone_wifi_slam_$stamp.out.txt"
$err = Join-Path $Analysis "iphone_wifi_slam_$stamp.err.txt"
Write-Host "Starting SLAM for ${Seconds}s → $out"

$p = Start-Process -FilePath $Exe `
    -ArgumentList @("--device", $Device, "--profile", "iphone", "--pose_source", $PoseSource) `
    -WorkingDirectory $ExeDir `
    -RedirectStandardOutput $out `
    -RedirectStandardError $err `
    -PassThru -NoNewWindow

$deadline = (Get-Date).AddSeconds($Seconds)
while (-not $p.HasExited -and (Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 500
}
if (-not $p.HasExited) {
    Stop-Process -Id $p.Id -Force
    Start-Sleep -Seconds 1
}

Write-Host "=== HEAD ==="
Get-Content $out -TotalCount 20 -ErrorAction SilentlyContinue
Write-Host "=== STATS ==="
$strong = @(Select-String -Path $out -Pattern "tier strong" -ErrorAction SilentlyContinue).Count
$lost = @(Select-String -Path $out -Pattern "Tracking lost" -ErrorAction SilentlyContinue).Count
$frames = @(Select-String -Path $out -Pattern "SLAM frame \d+" -ErrorAction SilentlyContinue).Count
$connected = @(Select-String -Path $out -Pattern "Connected to Record3D|stream ready|Found iOS device" -ErrorAction SilentlyContinue).Count
$failUsb = @(Select-String -Path $out -Pattern "No iOS devices found|Failed to start Record3D|Timed out waiting" -ErrorAction SilentlyContinue).Count
Write-Host "connected_signals=$connected fail_signals=$failUsb frames=$frames strong=$strong lost=$lost"
Write-Host "logs: $out"
if ($failUsb -gt 0 -or $connected -eq 0) {
    Write-Host ""
    Write-Host "무선 연결 실패 시 확인:"
    Write-Host "  - iTunes Wi-Fi 연결 체크 + 같은 Wi-Fi"
    Write-Host "  - Record3D는 USB Streaming (Wi-Fi Streaming/WebRTC 아님)"
    Write-Host "  - 케이블 완전 분리 후 수 초 대기"
    exit 2
}
exit 0
