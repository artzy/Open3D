# RealTimeSLAMRealSense 실행 테스트

**날짜:** 2026-06-16  
**빌드:** `build\bin\examples\Release\RealTimeSLAMRealSense.exe` (Release)  
**환경:** Windows, Intel RealSense D415 (314522061035), CUDA:0

## 빌드

```powershell
cmake --build d:\study\Open3D\build --config Release --target RealTimeSLAMRealSense
```

결과: **성공**

## CLI 테스트

| # | 명령 | 결과 | exit |
|---|------|------|------|
| 1 | (인자 없음) | 도움말 출력 | 1 |
| 2 | `--help` | 도움말 출력 | 1 |
| 3 | `-l` | D415 장치 목록 출력 | 0 |
| 4 | `--profile invalid` | LogError 후 비정상 종료 | -1073740791 |
| 5 | `--use_bag_file no_such.bag` | bag 열기 실패 LogError | -1073740791 |

참고: Open3D `LogError`는 예외 throw → Windows fast-fail exit code. `return 1`이 아님.

## 라이브 SLAM (RealSense D415)

작업 디렉터리: `d:\study\Open3D` (기본 config `examples/test_data/rs_d415_slam.json` 사용)

### profile low (25초, `--no-align`)

- CUDA:0, 640x480 @ 30fps
- **~694 frames** 처리 (~28 fps)
- hash blocks: **337** (정지 장면)
- stderr: 없음
- `scene.ply` / `trajectory.log`: 강제 종료로 미생성 (정상 — ESC/창 닫기 시 저장)

### profile medium (15초)

- **~397 frames** (~26 fps)
- hash blocks: **2106**
- stderr: 없음

## 실행 예

```powershell
cd d:\study\Open3D
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe --device CUDA:0 --profile medium
```

장치 목록:

```powershell
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe -l
```

## 로그 파일

- `Analysis/RealTimeSLAMRealSense_stdout.log` — low profile 25s
- `Analysis/RealTimeSLAMRealSense_medium.log` — medium profile 15s

## 결론

**실행 테스트 통과.** 빌드, RealSense 연결, CUDA SLAM 루프, 시각화 창 기동 모두 정상.
