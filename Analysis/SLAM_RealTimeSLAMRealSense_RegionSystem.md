# RealTimeSLAMRealSense 영역 감지·Freeze·폴리곤 변환 시스템

## 구현 요약

`RealTimeSLAMRealSense`에 스캔 중 안정된 영역을 자동 감지하여 TSDF 블록을 freeze하고 삼각형 메시로 추출·표시·저장하는 시스템을 추가했다.

### 변경 파일

| 파일 | 내용 |
|------|------|
| `SLAM/cpp/ObjectMeshPipeline.h` | `tsdf_mesh_only`, `defer_model_ops` 플래그, `ApplyFreezeAndExtractMesh()` 분리 |
| `SLAM/cpp/RealTimeSLAMRealSense.cpp` | RegionWorker, model_mutex, SharedState 메시 큐, CLI, 저장 |

### 아키텍처

- **SlamWorker**: `model_mutex`로 Track/Integrate/Synthesize/Extract 보호. `--regions` 시 `ExtractPointCloudExcludingFrozen` 사용.
- **RegionWorker**: 조건변수로 surface pcd 수신 → DBSCAN/tracker(잠금 없음) → freeze+메시 추출(`model_mutex`) → PLY/JSON 저장 → 메인 스레드 큐.
- **메인 루프**: `TakeRegionMesh` → `AddGeometry(..., reset_bounding_box=false)`.

## CLI 옵션

```
--regions                 영역 freeze 시스템 활성화 (기본 off)
--region_min_points N     최소 클러스터 포인트 (기본 5000)
--region_stability N      freeze 전 안정 프레임 수 (기본 5)
--region_interval N       영역 검사 주기 (기본 60)
--region_dir PATH         출력 디렉터리 (기본 regions)
```

## 출력

- `regions/region_<id>.ply` — TSDF 삼각형 메시
- `regions/regions.json` — id, type, AABB, block/vertex count, frame_id, timestamp
- 종료 시 `scene.ply`, `scene_mesh.ply`, `trajectory.log` (기존과 동일)

## GUI (OnlineSLAMRealSense 스타일)

legacy Visualizer 대신 Filament `SceneWidget` + 좌측 패널 GUI를 사용한다.

| 토글 | 기능 |
|------|------|
| **Cloud capture ON/OFF** | RGB-D 캡처 및 SLAM integration 일시정지/재개 |
| **Polygon ON/OFF** | freeze된 영역 TSDF 삼각형 메시 표시/숨김 |
| **Point cloud ON/OFF** | freeze된 영역 포인트 클라우드 표시/숨김 (폴리곤과 1:1 짝) |

영역 freeze 시 `region_<id>_mesh` + `region_<id>_pcd` 쌍으로 씬에 추가된다.

### 추가 파일

- `SLAM/cpp/RealTimeSLAMUtil.h` — `RealTimeSLAMWindow` GUI 클래스


```powershell
& "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\amd64\MSBuild.exe" `
  "d:\study\Open3D\SLAM\RealTimeSLAMRealSense.vcxproj" /p:Configuration=Release /p:Platform=x64
```

**결과**: Release x64 빌드 성공.

## 실행 테스트 (2026-07-08)

### 1. CLI 도움말

`--help`에 `--regions` 계열 옵션 5개 정상 표시 확인.

### 2. `--regions` 라이브 시작 (8초)

```
RealTimeSLAMRealSense.exe --regions --profile low --region_interval 30
```

- RealSense D415 캡처 시작
- `Region freeze enabled: min_points=5000, stability=5, interval=30, dir=regions` 로그 확인
- SLAM frame 0~1 정상 진행, 크래시 없음
- 8초 내 freeze 조건 미충족 → `regions/` 미생성 (예상 동작)

### 3. 회귀 (`--regions` 없음, 5초)

- `Region freeze enabled` 로그 없음
- SLAM frame 0~4 정상 진행

### 4. 수동 완전 검증 (권장)

한 벽면을 5회 이상 안정 관측:

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\RealTimeSLAMRealSense.exe --regions --device CUDA:0 --profile low
```

확인 항목:
- (a) 영역 메시가 뷰어에 추가되고 해당 영역 라이브 포인트 제외
- (b) `regions/region_0.ply`, `regions/regions.json` 생성
- (c) 뷰 뒤집힘 없음
- (d) ESC 종료 후 `scene.ply`에 전체 장면 포함
