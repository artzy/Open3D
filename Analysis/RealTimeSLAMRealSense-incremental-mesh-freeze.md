# RealTimeSLAMRealSense — 점진적 Mesh Freeze

## 개요

`RealTimeSLAMRealSense` 실행 중 TSDF에서 추출한 **미동결 surface point cloud**를 분석해, 안정된 클러스터를 mesh로 전환하고 TSDF 블록을 **freeze**합니다. freeze된 영역은 라이브 point cloud에서 제외되고, mesh는 뷰어에 누적 표시됩니다.

OnlineSLAMRealSense와 동일한 `ObjectMeshPipeline` / `Model::FreezeBlocks` semantics를 재사용하며, 공통 래퍼는 `IncrementalMeshFreeze.h`에 있습니다.

## 파이프라인

```
Integrate + Track (Strong only)
       ↓
ExtractPointCloudExcludingFrozen (주기: update_interval / freeze_interval)
       ↓
IncrementalMeshFreeze::ProcessSurface
  ├─ DBSCAN 클러스터
  ├─ ObjectFreezeTracker (N프레임 안정)
  ├─ FreezeBlocks(block_keys)
  └─ ExtractTriangleMeshIncluding → mesh
       ↓
GUI: live pcd (미동결) + frozen mesh AddGeometry (누적)
       ↓
objects/object_{id}.ply, pcd_{id}.ply, frozen_blocks.json
```

## 핵심 파일

| 파일 | 역할 |
|------|------|
| `examples/cpp/IncrementalMeshFreeze.h` | ProcessSurface, SaveEntries, frozen_blocks.json |
| `examples/cpp/ObjectMeshPipeline.h` | DBSCAN, 기하 분류, FreezeBlocks, mesh 추출 |
| `examples/cpp/RealTimeSLAMRealSense.cpp` | SLAM 루프 연동, CLI, Legacy Visualizer 다중 mesh |

## CLI

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--auto_freeze` | on | 점진적 mesh freeze 활성 |
| `--no-auto_freeze` | | 기존 단일 scene.ply 워크플로우 (freeze 없음) |
| `--freeze_interval N` | = `--update_interval` | surface 분석 주기 |
| `--min_cluster_points N` | 5000 | mesh 후보 최소 포인트 |
| `--stability_frames N` | 5 | 동일 클러스터 연속 관측 프레임 |
| `--objects_dir PATH` | `objects` | mesh/pcd/json 저장 경로 |

## 출력 디렉터리

```
objects/
  object_0.ply          # freeze된 mesh
  pcd_0.ply             # freeze 직전 클러스터 point cloud (선택)
  frozen_blocks.json    # id, type, mesh, block_keys
scene.ply               # 종료 시 미동결 잔여 point cloud
scene_mesh.ply          # 종료 시 미동결 잔여 mesh
trajectory.log
```

## freeze 트리거 조건

`ObjectMeshPipeline::SegmentationConfig` 기준:

- `min_cluster_points = 5000`
- `stability_frames = 5`
- `dbscan_eps = voxel_size × 2.0`
- `auto_freeze = true`

안정 클러스터 확정 시:

1. `CollectBlockKeys` → TSDF block keys
2. `Model::FreezeBlocks(block_keys)` — 이후 live extract에서 제외
3. `ExtractTriangleMeshIncluding(3.0, -1, block_keys)` — mesh 생성
4. (선택) 클러스터 bounds로 crop한 pcd → `pcd_{id}.ply`

## Open3D 한계 (중요)

TSDF voxel을 GPU에서 **물리 삭제**하는 API는 없습니다. **FreezeBlocks**는 “라이브 추출/표시 대상에서 제외”하는 정석 방식입니다. integrate는 계속 수행되며, frozen 블록만 extract 필터에 들어갑니다.

## Hash nearly full (92%) 정책

- **Integrate / tracking**: 일시 중단 (기존 정책 유지)
- **Freeze**: **허용** — frozen 블록을 live extract에서 빼면 hash 압력 완화에 도움

## Visualizer

- `render_pcd`: 미동결 surface (pointer reuse + `UpdateGeometry`)
- `frozen_meshes[]`: 새 mesh마다 `AddGeometry` 1회 (frozen mesh는 불변)
- 50개 이상 frozen mesh 시 경고 로그

## 수동 테스트 시나리오

### 1. 단일 벽/박스 천천히 스캔

1. `RealTimeSLAMRealSense -c examples/test_data/rs_d415_slam.json --profile medium`
2. 벽면 또는 큰 박스를 5프레임 이상 안정적으로 비춤
3. **기대**: `objects/object_0.ply` 생성, live pcd에서 해당 영역 감소, 뷰어에 mesh 추가

### 2. frozen_blocks.json 일관성

1. 스캔 후 `objects/frozen_blocks.json` 확인
2. **기대**: `id`, `type`, `mesh`, `block_keys`가 `object_*.ply`와 일치

### 3. Hash full + freeze 공존

1. `--block_count`를 낮게 설정하거나 큰 공간 스캔
2. 창 제목에 `HASH FULL` 표시 확인
3. **기대**: integrate는 멈추지만 freeze는 동작 가능, hash size 유지 또는 완화

### 4. `--no-auto_freeze` 회귀

1. `RealTimeSLAMRealSense ... --no-auto_freeze`
2. **기대**: `ExtractPointCloud` (frozen 제외 없음), `objects/` 미생성, 기존 단일 scene 저장

### 5. 종료 저장

1. ESC 또는 창 닫기
2. **기대**: `scene.ply` / `scene_mesh.ply`는 **미동결 잔여**만, frozen mesh는 `objects/`에 보존

## 빌드

```powershell
cmake --build d:\study\Open3D\build --config Release --target RealTimeSLAMRealSense
```

Core library 변경 없음 — `Model::FreezeBlocks`, `ExtractPointCloudExcludingFrozen`, `ExtractTriangleMeshIncluding` API만 사용.

## 2단계 (선택, 미구현)

`OnlineSLAMUtil.h`의 `FrozenObjectEntry` / save JSON을 `IncrementalMeshFreeze`로 이전해 중복 제거. 동작 변경 없음.
