# iPhone Wi-Fi SLAM VGA (640×480) 테스트 (2026-07-16)

## 결과: 성공

| 항목 | 값 |
|------|-----|
| host | `192.168.0.110` |
| SLAM 해상도 | **640×480** |
| frames | 60 / track_ok 59 / fail 0 |
| hash | 724 |
| ply | `Analysis/iphone_wifi_webrtc_slam/wifi_chrome_slam_vga_points.ply` (~3.1MB) |
| report | `Analysis/iphone_wifi_webrtc_slam/wifi_chrome_slam_vga_report.json` |
| log | `Analysis/iphone_wifi_slam_vga_run.txt` |

## 실행

```powershell
python -u SLAM\tools\record3d_wifi_chrome_slam_test.py --host 192.168.0.110 --seconds 25 --max-frames 60 --slam-width 640 --slam-height 480
```
