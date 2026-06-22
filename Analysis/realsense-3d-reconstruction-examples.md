# RealSense 3D Reconstruction 예제/코드 정리

> 참조 문서: https://www.open3d.org/docs/latest/tutorial/sensor/realsense.html  
> 소스: `docs/tutorial/sensor/realsense.rst`

## 1. 오프라인 씬 재구성 (문서 메인 예제)

RealSense `.bag` 파일을 입력으로 **fragment 생성 → 등록 → 정합 → TSDF 통합** 파이프라인 실행.

| 항목 | 경로 |
|------|------|
| 설정 | `examples/python/reconstruction_system/config/realsense.json` |
| 실행 | `examples/python/reconstruction_system/run_system.py` |
| 컬러맵 최적화 | `examples/python/reconstruction_system/color_map_optimization_for_reconstruction_system.py` |
| bag 자동 변환 | `examples/python/reconstruction_system/initialize_config.py` (`.bag` → depth/color/intrinsic) |
| JackJack 샘플 | `examples/python/reconstruction_system/data_loader.py` → `jackjack_data_loader()` |

```powershell
cd examples\python\reconstruction_system
python run_system.py --make --register --refine --integrate config\realsense.json
python color_map_optimization_for_reconstruction_system.py --config config\realsense.json
```

샘플 데이터: `python scripts/download_dataset.py L515_test` 또는 `JackJackL515Bag`

---

## 2. Tensor 기반 재구성 (TSDF / Dense SLAM)

`.bag` 지원, `common.py`의 `extract_rgbd_frames()`로 프레임 추출.

| 기능 | 파일 |
|------|------|
| TSDF 통합 | `examples/python/t_reconstruction_system/integrate.py` |
| Dense SLAM (CLI) | `examples/python/t_reconstruction_system/dense_slam.py` |
| Dense SLAM (GUI) | `examples/python/t_reconstruction_system/dense_slam_gui.py` |
| 전체 파이프라인 | `examples/python/t_reconstruction_system/run_system.py` |
| Ray casting | `examples/python/t_reconstruction_system/ray_casting.py` |
| bag 처리 공통 | `examples/python/t_reconstruction_system/common.py` |

```powershell
cd examples\python\t_reconstruction_system
python dense_slam_gui.py --default_dataset jack_jack
# 또는 config에 path_dataset: your.bag 지정
```

문서: `docs/tutorial/t_reconstruction_system/dense_slam.rst`, `integration.rst`

---

## 3. 실시간 온라인 SLAM (라이브 카메라 / bag)

| 언어 | 파일 | 설명 |
|------|------|------|
| C++ | `examples/cpp/OnlineSLAMRealSense.cpp` | RealSense 라이브/bag → TSDF SLAM + GUI |
| C++ 유틸 | `examples/cpp/OnlineSLAMUtil.h` | ReconstructionWindow, Track+Integrate |
| Python | `examples/python/visualization/online_processing.py` | 라이브 RGB-D → 포인트클라우드/GUI |
| Python | `examples/python/io/realsense_io.py` | RealSense discovery + 라이브 PCD |

```powershell
# C++ (BUILD_LIBREALSENSE=ON, BUILD_GUI=ON 빌드 필요)
bin\examples\OnlineSLAMRealSense --config rs-config.json
bin\examples\OnlineSLAMRealSense --use_bag_file L515_test.bag

# Python
python examples\python\visualization\online_processing.py
python examples\python\io\realsense_io.py rgbd.bag
```

로컬 설명: `docs/OnlineSLAMRealSense.md`

---

## 4. 캡처 / 재생 유틸

| 파일 | 역할 |
|------|------|
| `examples/cpp/RealSenseRecorder.cpp` | 라이브 캡처 + bag 녹화 + GUI |
| `examples/cpp/RealSenseBagReader.cpp` | bag 재생 뷰어 |
| `examples/python/reconstruction_system/sensors/realsense_recorder.py` | **레거시** pyrealsense2 녹화 |
| `examples/python/reconstruction_system/sensors/realsense_pcd_visualizer.py` | **레거시** pyrealsense2 PCD |

---

## 5. Open3D 내장 RealSense API (C++ / Python)

| 경로 | 내용 |
|------|------|
| `cpp/open3d/t/io/sensor/realsense/RealSenseSensor.cpp` | 라이브 캡처 |
| `cpp/open3d/t/io/sensor/realsense/RSBagReader.cpp` | bag 읽기 |
| `cpp/pybind/t/io/sensor.cpp` | Python 바인딩 |
| `python/test/t/io/test_realsense.py` | 단위 테스트 |

Python API:
- `o3d.t.io.RealSenseSensor` — 라이브 카메라
- `o3d.t.io.RSBagReader` / `RGBDVideoReader` — bag 파일
- `o3d.t.io.RealSenseSensorConfig` — JSON 설정

---

## 6. 접근 방식 비교

| 목적 | 권장 예제 |
|------|-----------|
| bag → 고품질 오프라인 메시 | `reconstruction_system/run_system.py` |
| bag/프레임 → Tensor TSDF | `t_reconstruction_system/integrate.py` |
| 실시간 SLAM (방 규모) | `OnlineSLAMRealSense.cpp` 또는 `dense_slam_gui.py` |
| 라이브 PCD 시각화 | `online_processing.py`, `realsense_io.py` |
| 데이터 수집 | `RealSenseRecorder.cpp` |

**참고:** `reconstruction_system/sensors/` 하위 pyrealsense2 예제는 Open3D v0.12 이전 방식. 현재는 Open3D 내장 `o3d.t.io.RealSenseSensor` 사용 권장.
