# iPhone Wi-Fi SLAM 재시도 #2 결과 (2026-07-16)

## 결론

**같은 Wi-Fi LAN에서 Record3D Wi-Fi Streaming → Chromium(Playwright) → Open3D CPU SLAM 경로로 성공.**

| 항목 | 결과 |
|------|------|
| iPhone | `192.168.0.110` (PC `192.168.0.142`, USB 테더 없음) |
| 연결 | WebRTC (Record3D Wi-Fi Streaming) |
| 수신 | Playwright Chromium + 공식 `record3d-wifi-demo` |
| SLAM | Open3D Tensor Model (CPU, voxel=0.02, 179×240) |
| 프레임 | **200** / track_ok **199** / fail **0** / hash **196** |
| 산출 | `Analysis/iphone_wifi_webrtc_slam/wifi_chrome_slam_points.ply` (~1.1MB) |
| 리포트 | `Analysis/iphone_wifi_webrtc_slam/wifi_chrome_slam_report.json` |
| 로그 | `Analysis/iphone_wifi_chrome_slam_run6.txt` |

## 경로 요약

1. Record3D Live RGBD → **Wi-Fi** → Waiting for Connection
2. PC: `python -m http.server 8765` in `3rdparty/record3d-wifi-demo`
3. `python SLAM/tools/record3d_wifi_chrome_slam_test.py --host 192.168.0.110 --seconds 25`

## 검증 중간 결과

- **Grab smoke**: native 1164×776, 40 grabs / 0.7s (`record3d_wifi_grab_smoke.py`)
- **aiortc**: ICE connected 후 프레임 0 (이전과 동일, Windows H.264 이슈)
- **pip Open3D**: CUDA 모듈 없음 → CPU SLAM 사용 (고해상도에서 과거 hang)

## 한계 (변함없음)

- Wi-Fi Streaming은 **ARKit 포즈 없음** → odometry only
- Depth는 lossy HSV 인코딩
- C++ `IPhoneSLAMRealSense` USB/usbmuxd 경로와는 별개 (포즈 prior는 USB 쪽)

## 재실행

```powershell
cd d:\study\Open3D
# demo 서버가 8765에서 떠 있어야 함
python -u SLAM\tools\record3d_wifi_chrome_slam_test.py --host 192.168.0.110 --seconds 25
```
