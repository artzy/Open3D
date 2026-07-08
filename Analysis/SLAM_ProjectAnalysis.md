# SLAM 프로젝트 분석

**날짜:** 2026-07-08
**대상:** `d:\study\Open3D\SLAM\` (Visual Studio 솔루션, 브랜치 `ad_poly`)

## 1. 개요

Open3D 예제(`examples/cpp`)의 RGB-D Dense SLAM 코드를 독립 Visual Studio 솔루션으로 이전한 프로젝트.
CMake로 빌드된 Open3D(`build/`)를 정적 링크하며, Intel RealSense D415 라이브 캡처와 파일/bag 재생을 모두 지원한다.

**전제 조건:** Open3D가 `d:\study\Open3D\build`에 CMake로 빌드되어 있어야 함 (CUDA 12.6, librealsense, Filament, MKL 포함).

## 2. 폴더 구조

```
SLAM/
├── slam.sln                          # VS2022 솔루션 (Debug/Release x64)
├── OnlineSLAMRGBD.vcxproj            # 파일 재생 SLAM (새 GUI)
├── OnlineSLAMRealSense.vcxproj       # RealSense 라이브 SLAM (새 GUI)
├── RealTimeSLAMRealSense.vcxproj     # RealSense 라이브 SLAM (레거시 뷰어)
├── Open3DExample.props               # 공통 include/link 설정 (generate_props.py로 생성)
├── generate_props.py                 # CMake vcxproj → props 재생성 스크립트
├── copy_resources.ps1                # GUI 리소스 수동 동기화
├── .gitignore                        # bin/obj/.vs/log 제외
├── cpp/                              # 소스 (examples/cpp에서 복사 후 독자 수정)
│   ├── OnlineSLAMRGBD.cpp            (198줄)  파일 재생 엔트리
│   ├── OnlineSLAMRealSense.cpp       (257줄)  RealSense 엔트리 (profile/perf 프리셋)
│   ├── RealTimeSLAMRealSense.cpp     (863줄)  단일 파일 SLAM+레거시 뷰어
│   ├── OnlineSLAMUtil.h              (1601줄) ReconstructionWindow 핵심 구현
│   └── ObjectMeshPipeline.h          (453줄)  클러스터 분할→객체 freeze 파이프라인
├── resources/                        # Filament 셰이더·폰트·IBL (64개)
└── bin/Release/                      # 출력 (exe + tbb12.dll + zlib1.dll)
```

## 3. 실행 파일 3종 비교

| | OnlineSLAMRGBD | OnlineSLAMRealSense | RealTimeSLAMRealSense |
|---|---|---|---|
| 입력 | lounge/bedroom 데이터셋, 폴더 | RealSense 라이브, bag | RealSense 라이브, bag |
| GUI | 새 GUI (Filament, `SceneWidget`) | 새 GUI (Filament) | 레거시 `VisualizerWithKeyCallback` (OpenGL) |
| 객체 freeze | O (`ObjectMeshPipeline`) | O | X (포인트 클라우드만) |
| 파라미터 패널 | O (`PropertyPanel`) | O | X (CLI 옵션만) |
| 출력 | 창 닫을 때 없음 | 없음 | `scene.ply`, `scene_mesh.ply`, `trajectory.log` |
| 공유 코드 | `OnlineSLAMUtil.h` | `OnlineSLAMUtil.h` | 자체 완결 (단일 파일) |

## 4. 핵심 아키텍처 (`OnlineSLAMUtil.h` — ReconstructionWindow)

### 스레드 구성 (4개)
1. **UpdateMain** (`update_thread_`): 캡처 → `TrackFrameToModel` 오도메트리 → `Integrate` → `SynthesizeModelFrame` → GUI 이미지 post
2. **ExtractWorker** (`extract_thread_`): 비동기 `ExtractPointCloud` (조건변수 트리거, `kMaxRenderPoints=100k` 다운샘플)
3. **SegmentationWorker** (`segmentation_thread_`): DBSCAN 클러스터링 → 객체 freeze (`kMaxSegmentationPoints=200k` 캡)
4. **GUI 메인 스레드**: Filament 렌더, `PostToMainThread`로 geometry 갱신

### 추적 계층 (TrackingTier)
- `kStrong` (fitness≥0.15, translation<0.12): 포즈 갱신 + integrate
- `kWeak` (fitness≥0.08, translation<0.30): 이전 포즈로 integrate만
- `kOutlier` (translation≥0.50): 거부
- `kFail`: integrate 건너뜀, 실패 시 depth_diff 2배로 재시도

### 메모리 안전장치
- 해시 용량: 초기 40k → 최대 50k 블록, `kHashIntegrateFillRatio=0.95` 이상이면 integrate 스킵
- `GetLiveExtractBudget`: 해시 크기에 따라 extract 버짓 2M~5M 캡
- `GetEffectiveUpdateInterval`: 해시 성장 시 GUI 갱신 주기 자동 확대 (75→200프레임)
- Filament vertex buffer 재사용 불가 → 항상 `RemoveGeometry`+`AddGeometry` 재빌드

### 카메라 뷰 (뒤집힘 수정)
- SLAM 월드는 +Y 아래 (카메라 관례) ↔ Open3D 기본 +Y 위
- `ConfigureSlamCameraView`: `SetupCamera` 후 `LookAt(up=(0,-1,0))` 재적용
- `camera_view_initialized_`로 실제 geometry에서 1회만 피팅, 이후 `SetCenterOfRotation`만

## 5. 객체 Freeze 파이프라인 (`ObjectMeshPipeline.h`)

```
표면 포인트 클라우드 (≤200k)
  → ClusterDBSCAN (eps=voxel×배수, min 5000점)
  → ClassifyCluster: 평면 inlier·OBB 비율로 wall / box / cylinder / generic 분류
  → ObjectFreezeTracker: 시그니처(centroid+OBB IoU) 매칭, stability_frames(5) 연속 관측 시 freeze
  → FreezeBlocks(block_keys): TSDF 해시 블록 동결
  → 메시 생성: wall/box/cylinder → OBB 기반 프리미티브, generic → ExtractTriangleMeshIncluding
  → GUI: "object_<id>"로 defaultLit 머티리얼 추가
```

## 6. RealTimeSLAMRealSense 특이사항 (863줄 단일 파일)

- **SlamWorker 스레드** + 메인 스레드 레거시 뷰어(30fps `PollEvents`) 구조
- 첫 프레임: 250ms 워밍업 + 최대 30회 재시도 (실패해도 abort 없이 정상 종료)
- hash 92% 도달 시: **integrate만 중단, tracking 계속** — 창 제목 `HASH FULL (tracking only)`
- frame-to-frame bridge: 모델 추적 실패 시 이전 프레임 대비 오도메트리로 포즈 연결
- 뷰: `ApplySlamViewOrientation` (`SetUp(0,-1,0)`, `SetFront(0,0,-1)`), `AddGeometry(..., false)`로 자동 view reset 차단
- 종료 시 2-pass extract로 `scene.ply`/`scene_mesh.ply` 저장

## 7. 빌드 시스템

- `Open3DExample.props`: CMake가 생성한 예제 vcxproj에서 추출한 include/lib 목록 (`generate_props.py`)
  - include: `cpp/`(Open3D 소스), `SLAM/cpp`, glew/glfw/fmt/filament/eigen (external)
  - link: `Open3D.lib` + 3rdparty 약 80개 (`/force:multiple` 필요 — zlib 중복 정의)
  - 매크로: `$(Open3DRoot)` = 리포 루트, `$(BuildRoot)` = `build/`
- PostBuild: 리소스 robocopy → `SLAM/resources`, `tbb12.dll`·`zlib1.dll` 복사
- 디버거: `LocalDebuggerCommand`/`TargetPath` 명시 (Ctrl+F5 대응), 작업 디렉터리 = `$(OutDir)`

```powershell
& "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" `
  "d:\study\Open3D\SLAM\slam.sln" /p:Configuration=Release /p:Platform=x64
```

## 8. 알려진 제약·주의점

| 항목 | 내용 |
|---|---|
| zlib 링크 경고 | vcpkg zlib.lib와 zlibstatic.lib 중복 → LNK4006 + `/FORCE` (동작엔 문제 없음) |
| headless 실행 | 콘솔 리다이렉트 시 RealSense 첫 프레임 실패 — GUI 실행 필요 |
| `--no-align` + 고해상 config | `Aligned image pair must have the same resolution` 경고 → `--align` 권장 |
| Odometry singular | 특징 없는 뷰/저 overlap에서 정상 발생 — WARNING 후 해당 프레임 스킵 |
| hash full | 대공간 스캔 시 `--profile high --block_count 80000` 권장 |
| 파일 재생 크래시 | auto_freeze OFF + 33ms 페이싱 + 100k 렌더 캡으로 완화 |

## 9. 관련 문서

- `Analysis/SLAM_Resources.md` — 리소스 배치
- `Analysis/OnlineSLAMRGBD_CrashFix.md` — 파일 재생 크래시 분석
- `Analysis/SLAM_CameraFlipFix.md` — 뷰 뒤집힘 수정
- `Analysis/SLAM_RealTimeSLAMRealSense-run-test.md` — 실행 테스트
- `Analysis/RealTimeSLAMRealSense*.md` — 원본 예제 분석 (examples/cpp 기준)
