# RealTimeSLAMRealSense C++ 예제

## 개요

`examples/cpp/RealTimeSLAMRealSense.cpp`는 Intel RealSense D415 등 RGB-D 카메라로 **실시간 Dense SLAM**을 수행하는 단일 파일 C++ 예제입니다.

- SLAM 코어: `t::pipelines::slam::Model` (OfflineSLAM.cpp와 동일 패턴)
- 입력: `t::io::RealSenseSensor` (라이브) / `t::io::RSBagReader` (bag)
- GUI: legacy `visualization::Visualizer` (실시간 포인트클라우드 갱신)

## 관련 파일

| 파일 | 역할 |
|------|------|
| `examples/cpp/RealTimeSLAMRealSense.cpp` | 메인 예제 |
| `examples/test_data/rs_d415_slam.json` | D415 기본 설정 (640×480, HIGH_ACCURACY) |
| `examples/cpp/OnlineSLAMRealSense.cpp` | 고급 GUI + object freeze (별도 예제) |

## 빌드

```powershell
cmake -S d:\study\Open3D -B d:\study\Open3D\build `
  -DBUILD_LIBREALSENSE=ON -DBUILD_CUDA_MODULE=ON -DBUILD_GUI=ON -DBUILD_EXAMPLES=ON
cmake --build d:\study\Open3D\build --config Release --target RealTimeSLAMRealSense
```

## 실행 (D415)

```powershell
cd d:\study\Open3D\build\bin\examples\Release

# 장치 목록
.\RealTimeSLAMRealSense.exe -l

# 라이브 SLAM (D415 기본 config)
.\RealTimeSLAMRealSense.exe -c ..\..\..\..\examples\test_data\rs_d415_slam.json --device CUDA:0

# VRAM 절약
.\RealTimeSLAMRealSense.exe -c rs_d415_slam.json --profile low --no-align

# bag 재생
.\RealTimeSLAMRealSense.exe --use_bag_file capture.bag --profile medium

# 녹화 + SLAM
.\RealTimeSLAMRealSense.exe -c rs_d415_slam.json --record session.bag
```

## 출력

종료 시 현재 디렉터리에 생성:

- `scene.ply` — TSDF에서 추출한 포인트클라우드
- `trajectory.log` — 카메라 궤적

## SLAM 프로필

| `--profile` | block_count | depth_max | 용도 |
|-------------|-------------|-----------|------|
| `low` | 16,384 | 2 m | 저 VRAM / 빠른 처리 |
| `medium` (기본) | 40,000 | 3 m | D415 실내 일반 |
| `high` | 50,000 | 5 m | 넓은 공간 |

## OnlineSLAMRealSense vs RealTimeSLAMRealSense

| | RealTimeSLAMRealSense | OnlineSLAMRealSense |
|--|----------------------|---------------------|
| 파일 수 | 단일 cpp | cpp + OnlineSLAMUtil.h (1500+ lines) |
| GUI | legacy Visualizer | Filament O3DVisualizer |
| 기능 | Track + Integrate + PCD | + raycast preview, object mesh freeze, perf preset |
| 학습/시작 | 권장 | 고급 튜닝용 |
