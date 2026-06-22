# OfflineReconstructionRealSense C++ 예제

## 개요

`examples/cpp/OfflineReconstructionRealSense.cpp`는 RealSense `.bag` 파일을 읽어 **오프라인 Dense SLAM 재구성**을 수행합니다.

- 입력: `t::io::RSBagReader` (RealSense bag)
- SLAM: `t::pipelines::slam::Model` (OfflineSLAM.cpp와 동일)
- 출력: `scene.ply`, `scene_mesh.ply`, `trajectory.log`

## 관련 예제

| 예제 | 용도 |
|------|------|
| **OfflineReconstructionRealSense** (신규) | bag → 오프라인 재구성 |
| `OfflineSLAM.cpp` | color/depth 폴더 → 오프라인 SLAM |
| `RealTimeSLAMRealSense.cpp` | 라이브/bag → 실시간 SLAM |
| `RealSenseBagReader.cpp` | bag 재생/프레임 추출 |

## 빌드

```powershell
cmake --build d:\study\Open3D\build --config Release --target OfflineReconstructionRealSense
```

## 실행

```powershell
cd d:\study\Open3D\build\bin\examples\Release

# D415 등으로 녹화한 bag
.\OfflineReconstructionRealSense.exe --input capture.bag --device CUDA:0 --profile medium

# 샘플 데이터 (JackJack L515 bag, 자동 다운로드)
.\OfflineReconstructionRealSense.exe --default_dataset jack_jack --vis

# 프레임 수 제한 / 간격 샘플링
.\OfflineReconstructionRealSense.exe --input capture.bag --max_frames 300 --frame_step 2

# 출력 경로 지정
.\OfflineReconstructionRealSense.exe --input capture.bag `
  --pointcloud output.ply --mesh output_mesh.ply --trajectory cam.log
```

## D415 권장 파라미터

| `--profile` | depth_max | block_count | 용도 |
|-------------|-----------|-------------|------|
| `low` | 2 m | 16,384 | 빠른 테스트 |
| `medium` (기본) | 3 m | 40,000 | 실내 일반 |
| `high` | 5 m | 50,000 | 넓은 공간 |

실내 중거리 D415: `--profile medium`, 필요 시 `--voxel_size 0.006`.

## 워크플로우 (D415)

1. `RealTimeSLAMRealSense.exe --record session.bag` 또는 `RealSenseRecorder`로 bag 녹화
2. `OfflineReconstructionRealSense.exe --input session.bag`로 고품질 메시 생성
