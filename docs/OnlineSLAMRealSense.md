# OnlineSLAMRealSense 예제 설명

`examples/cpp/OnlineSLAMRealSense.cpp`는 **Intel RealSense 카메라(또는 bag 파일)로 RGB-D 영상을 받아 실시간 3D 재구성(SLAM)**을 하는 GUI 예제의 **진입점(entry point)**입니다. SLAM·GUI 본체는 같은 폴더의 `OnlineSLAMUtil.h`에 있습니다.

## 한 줄 요약

RealSense에서 컬러+깊이 프레임을 읽고 → 카메라 포즈 추적 → TSDF 볼륨에 통합 → 3D 포인트 클라우드·궤적을 화면에 보여주는 **온라인 RGB-D SLAM 데모**.

## 파일 구조

```
OnlineSLAMRealSense.cpp  →  입력 소스 설정 (Live / Bag)
         ↓
ReconstructionWindow (OnlineSLAMUtil.h)  →  Track + Integrate + GUI
```

- `OnlineSLAMRealSense.cpp`: 입력 소스만 고르고 GUI를 띄우는 역할
- 실제 SLAM 루프: `ReconstructionWindow` (`OnlineSLAMUtil.h`)
- `OnlineSLAMRGBD.cpp`와 구조가 거의 같고, 입력만 **이미지 폴더** 대신 **RealSense**를 사용

## `main()` 처리 순서

### 1. 도움말 / 초기 설정

- `-h`, `--help` 또는 인자 없음 → `PrintHelp()` 후 종료
- `-V` → 로그 레벨 Debug, 없으면 Info

### 2. RealSense 장치 목록

- `-l` / `--list-devices` → 연결된 RealSense 목록 출력 후 종료

### 3. 실행 옵션

| 옵션 | 의미 |
|------|------|
| `-c` / `--config` | RealSense 설정 JSON (`rs-config.json` 등) |
| `--align` | 컬러·깊이 스트림 정렬 |
| `--record` | 실행 중 bag 파일로 녹화 |
| `--use_bag_file` | bag 파일에서 RGB-D 재생 (경로 지정) |
| `--device` | 연산 디바이스 (기본값 `CUDA:0`) |
| `--profile` | SLAM 메모리 프로필: `low` / `medium`(기본) / `high` |
| `--perf` | SLAM 속도 프로필: `fast`(기본) / `balanced` / `quality` |
| `--no-align` | 라이브 캡처 시 depth-to-color 정렬 생략 (~33ms/frame 절약) |

### 4. 입력 소스 분기

#### A. 라이브 카메라 (`use_bag_file == false`)

1. 설정 JSON 읽기 (선택)
2. `RealSenseSensor::ListDevices()` / `InitSensor()` / `StartCapture()`
3. `get_rgbd_image_input` 콜백: `rs.CaptureFrame(true, align_streams)`
4. 메타데이터에서 `intrinsic_t`(내부 파라미터), `depth_scale` 추출

#### B. Bag 파일 (`--use_bag_file`)

1. `RSBagReader`로 녹화 파일 열기
2. `get_rgbd_image_input` 콜백: `bag_reader.NextFrame()`
3. EOF 이후 빈 `RGBDImage` 반환 → 루프 종료

### 5. GUI 실행

```cpp
auto& app = gui::Application::GetInstance();
app.Initialize();
app.AddWindow(std::make_shared<examples::online_slam::ReconstructionWindow>(
        get_rgbd_image_input, intrinsic_t, default_params, device, mono));
app.Run();
```

`ReconstructionWindow`에 넘기는 것:

- RGB-D 입력 콜백
- 카메라 내부 파라미터 (`intrinsic_t`)
- 기본 SLAM 파라미터 (`depth_scale` 등)
- 연산 디바이스 (`CUDA:0` / `CPU:0` 등)

### 6. 정리

- 라이브: `rs.StopCapture()`
- Bag: `bag_reader.Close()`

## `ReconstructionWindow`가 하는 일 (`OnlineSLAMUtil.h`)

이 cpp 파일에는 없지만, 여기서 넘긴 인자로 다음이 실행됩니다.

1. **Resume/Pause** 토글 후 `t::pipelines::slam::Model` 생성
   - VoxelBlockGrid 기반 TSDF (공간 해시맵 + 16³ 복셀 블록)
2. **백그라운드 스레드** `UpdateMain()`에서 매 프레임:
   - `TrackFrameToModel()` — 이전 모델 대비 RGB-D 오도메트리(포즈 추정)
   - `Integrate()` — TSDF 볼륨에 깊이 통합
   - `SynthesizeModelFrame()` — 모델에서 raycast (가상 컬러/깊이)
   - `ExtractPointCloudExcludingFrozen()` — 주기적으로 frozen 블록 제외 partial extract
   - **SegmentationWorker** — DBSCAN·기하 분류·안정성 추적 후 mesh freeze
3. **GUI**
   - 좌측: 파라미터 슬라이더, 입력/raycast 이미지 탭, Info 탭
   - 우측: 3D 뷰 (live point cloud, frozen object mesh, 카메라 frustum, 궤적)
4. **창 닫을 때** `scene.ply`(잔여 point cloud), `objects/`(frozen mesh + block keys), `trajectory.log` 저장

### SLAM 파라미터 예시

| 파라미터 | 역할 |
|----------|------|
| `depth_scale` | 깊이 값 스케일 (보통 1000) |
| `voxel_size` | TSDF 복셀 크기 |
| `trunc_multiplier` | 표면 두께 (truncation 거리 배수) |
| `block_count` | 해시맵 블록 수 (메모리) |
| `estimated_points` | 포인트 클라우드 버퍼 크기 추정 |
| `depth_max` | 이 거리 이상 깊이는 배경으로 제거 |
| `depth_diff` | 추적 시 이상값 제거용 깊이 차이 |
| `update_interval` | 3D 표면 갱신 주기 (프레임 수) |
| `update_surface` | 표면 업데이트 on/off |
| `raycast_color` | raycast 컬러 이미지 사용 |
| `auto_freeze` | 안정 클러스터 자동 mesh freeze (기본 on) |
| `stability_frames` | freeze 전 연속 안정 extract 횟수 (기본 5) |
| `dbscan_eps_multiplier` | DBSCAN eps = multiplier × voxel_size (기본 2.0) |
| `min_cluster_points` | freeze 대상 최소 클러스터 점 수 (기본 5000) |

자세한 freeze 동작: [Analysis/object-mesh-freeze.md](../Analysis/object-mesh-freeze.md)

## CUDA와의 관계

```cpp
std::string device_code =
        utility::GetProgramOptionAsString(argc, argv, "--device", "CUDA:0");
core::Device device(device_code);
```

- `--device CUDA:0` → RGB-D·SLAM 연산을 GPU에서 수행
- `get_rgbd_image_input_(idx).To(device_)`로 프레임을 GPU로 업로드
- `slam::Model`도 같은 `device`로 생성

CUDA 빌드(`BUILD_CUDA_MODULE=ON`)가 필요합니다. CPU만 쓰려면 `--device CPU:0`.

RTX 3060 등 Ampere GPU는 compute capability **8.6** (`CMAKE_CUDA_ARCHITECTURES=86`).

## 메모리 프로필 (`--profile`)

RTX 3060 12GB 기준. TSDF VRAM은 블록당 ~48 KB.

| 프로필 | block_count | estimated_points | depth_max | TSDF VRAM (추정) |
|--------|-------------|------------------|-----------|------------------|
| `low` | 16,384 | 1.5M | 2.0 m | ~0.8 GB |
| `medium` (기본) | 40,000 | 6M | 3.0 m | ~1.9 GB |
| `high` | 50,000 | 8M | 5.0 m | ~2.4 GB |

RealSense 저해상도 캡처: `examples/test_data/rs_slam_lowmem.json` (540p/480p, 30fps).

## Performance preset (`--perf`)

`--profile`(VRAM)과 독립. 기본값 **fast**.

| preset | odom iter | GUI interval | raycast color |
|--------|-----------|--------------|---------------|
| `fast` (기본) | 3/2/1 | 3 | off |
| `balanced` | 4/2/1 | 2 | on |
| `quality` | 6/3/1 | 1 | on |

자세한 튜닝: [Analysis/slam-performance.md](../Analysis/slam-performance.md), [Analysis/realsense-vram-optimization.md](../Analysis/realsense-vram-optimization.md)

## 실행 예시

```bat
OnlineSLAMRealSense.exe --device CUDA:0 --profile medium --perf fast
# GUI: Auto freeze on, Stability frames 5 — frozen object 증가 시 Live surface points 감소 확인
OnlineSLAMRealSense.exe -l
OnlineSLAMRealSense.exe --use_bag_file record.bag --profile low --perf quality
OnlineSLAMRealSense.exe --perf fast -c examples\test_data\rs_slam_lowmem.json --no-align
OnlineSLAMRealSense.exe --align --record out.bag
```

배치 파일 예 (`build/bin/examples/Release/OnlineSLAMRealSense.bat`):

```bat
@echo off
cd /d "%~dp0"
OnlineSLAMRealSense.exe --device CUDA:0 %*
```

## 출력 파일

창을 닫을 때 **현재 작업 디렉터리**에 생성:

| 파일 | 내용 |
|------|------|
| `scene.ply` | 재구성된 3D 포인트 클라우드 |
| `trajectory.log` | 카메라 궤적 (PinholeCameraTrajectory) |

## 관련 파일

| 파일 | 역할 |
|------|------|
| `examples/cpp/OnlineSLAMRealSense.cpp` | RealSense/bag 입력 + GUI 시작 |
| `examples/cpp/OnlineSLAMUtil.h` | SLAM 루프, GUI (`ReconstructionWindow`) |
| `examples/cpp/OnlineSLAMRGBD.cpp` | 동일 SLAM, 이미지 폴더 입력 |
| `docs/tutorial/sensor/realsense.rst` | RealSense 센서 일반 문서 |
| `docs/tutorial/t_reconstruction_system/dense_slam.rst` | Tensor SLAM 관련 튜토리얼 |

## 정리

| 구분 | 내용 |
|------|------|
| 역할 | RealSense RGB-D → 실시간 SLAM GUI 데모의 **main** |
| 핵심 API | `t::io::RealSenseSensor`, `t::pipelines::slam::Model`, `gui::Application` |
| 이 파일의 책임 | CLI 파싱, 입력 소스 설정, GUI 시작 |
| SLAM/GUI 로직 | `OnlineSLAMUtil.h`의 `ReconstructionWindow` |
| 출력 | 종료 시 `scene.ply`, `trajectory.log` |
