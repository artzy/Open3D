# SLAM RealTimeSLAMRealSense 실행 테스트

**일시:** 2026-07-08  
**대상:** `SLAM/bin/Release/RealTimeSLAMRealSense.exe`  
**환경:** Windows, Intel RealSense D415 (314522061035), CUDA:0

## 테스트 결과

| 테스트 | 명령 | 결과 |
|--------|------|------|
| 도움말 | `--help` | OK (Usage 출력) |
| 장치 목록 | `-l` | OK (D415 인식) |
| Live SLAM (GUI) | `-c rs_d415_slam.json --profile low --align` | OK (15s 크래시 없음) |
| Live SLAM (GUI) | `-c rs_d415_slam.json --profile low --no-align` | OK (22s 크래시 없음) |
| Live SLAM (headless) | stdout/stderr 리다이렉트 | **FAIL** — 첫 RGB-D 프레임 캡처 실패 |

## headless 실패 원인

`Start-Process -RedirectStandardOutput`로 콘솔만 리다이렉트하면 RealSense 첫 프레임이 비어 `Failed to capture the first RGB-D frame`로 종료됩니다.  
GUI 창을 띄운 실행은 정상 동작합니다.

## 권장 실행

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\RealTimeSLAMRealSense.exe --device CUDA:0 -c d:\study\Open3D\examples\test_data\rs_d415_slam.json --profile low --align
```

- 종료: ESC 또는 창 닫기 → `scene.ply`, `scene_mesh.ply`, `trajectory.log` 저장
- `rs_slam_lowmem.json`(960×540) + `--no-align` 시 align 경고 가능 → `--align` 권장

## 로그

- `Analysis/SLAM_RealTimeSLAM_test.log`
- `Analysis/SLAM_RealTimeSLAM_test_err.log`
