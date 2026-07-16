# iPhone SLAM USB 실행 테스트 (2026-07-16 14:34)

## 결과: 성공

| 항목 | 값 |
|------|-----|
| 실행 파일 | `SLAM/bin/Release/IPhoneSLAMRealSense.exe` |
| 연결 | USB Record3D (`이더넷 2` Apple Mobile Device Ethernet = OK) |
| 디바이스 | productId=4776, udid=00008120-00010C4A0168C01E |
| 스트림 | 192×256, depth_scale=1000, CUDA:0 |
| pose_source | fused — ARKit calibration locked |
| 시간 | ~50초 timebox 후 강제 종료 |
| 프레임 | **2497** (last #2496) |
| hash | max **20495**/40000 |
| tier | strong 2489 / weak 7 / init 1 (**lost 0**) |
| keyframes | 23 |
| ARKit | calibration locked (fused) |

## 로그
- `Analysis/iphone_slam_usb_exec_out.txt`
- `Analysis/iphone_slam_usb_exec_err.txt`

## 재실행
```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\IPhoneSLAMRealSense.exe --device CUDA:0 --profile iphone --pose_source fused
```
Record3D에서 **USB Streaming** + Record 필요.
