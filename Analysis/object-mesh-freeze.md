# 런타임 객체 분리 및 Mesh Freeze

## 개요

SLAM 실행 중 추출된 point cloud에서 **안정된 구조(벽·박스·원기둥 등)** 를 자동으로 감지해 mesh asset으로 **freeze**합니다. freeze된 TSDF 블록은 이후 partial extract에서 제외되어 live point 수·extract/GUI 비용이 감소합니다.

## 아키텍처

```
ExtractWorker (async)
  └─ Model::ExtractPointCloudExcludingFrozen()
  └─ surface_.pcd 갱신
       ↓
SegmentationWorker (async, CPU)
  └─ DBSCAN 클러스터링
  └─ 기하 분류 (wall/box/cylinder/generic)
  └─ ClusterSignature 안정성 추적 (N frames)
  └─ freeze 시 Model::FreezeBlocks() + mesh 생성
       ↓
GUI Main Thread
  └─ frozen mesh: AddGeometry("object_{id}")
  └─ live points: UpdateGeometry("points")
```

## 코어 API

### `VoxelBlockGrid`

- `ExtractPointCloudExcluding(weight, budget, exclude_block_keys)`
- `ExtractPointCloudIncluding(weight, budget, include_block_keys)`
- `ExtractTriangleMeshExcluding(...)` / `ExtractTriangleMeshIncluding(...)`

`exclude_block_keys` / `include_block_keys`는 `(N, 3)` Int32 CPU 텐서입니다.

### `slam::Model`

- `FreezeBlocks(block_keys)` — frozen set에 병합 (Integrate는 계속 수행)
- `GetFrozenBlockKeys()` — 누적 frozen keys 반환
- `ExtractPointCloudExcludingFrozen(...)` — frozen 블록 제외 extract
- `ExtractTriangleMeshIncluding(...)` — 클러스터 블록만 mesh extract

## 분류 heuristic (`ObjectMeshPipeline.h`)

| 타입 | 판별 조건 | Mesh |
|------|-----------|------|
| **wall** | plane inlier ≥ 85%, OBB 최短축/중간축 < 0.15 | 얇은 OBB box |
| **box** | 3축 extent 유의미, plane inlier < 85% | OBB → CreateBox + transform |
| **cylinder** | 2 eigenvalue 유사·1 작음, 단면 circle RMSE < 0.03 | CreateCylinder + PCA pose |
| **generic** | 위 조건 불일치 | TSDF partial mesh extract, 실패 시 OBB box |

## 안정성 (auto freeze)

- extract마다 `ClusterSignature` (centroid, OBB extent, type) 갱신
- centroid 거리 < 0.15 m, extent IoU > 0.7 이면 동일 클러스터로 매칭
- **Stability frames** (기본 5) 연속 매칭 시 freeze

## GUI 파라미터 (Reconstruction settings)

| 슬라이더 | 기본값 | 설명 |
|---------|--------|------|
| Auto freeze | on | 자동 mesh freeze |
| Stability frames | 5 | freeze 전 필요 연속 안정 extract |
| DBSCAN eps x voxel | 2.0 | DBSCAN 거리 = multiplier × voxel_size |
| Min cluster points | 5000 | 작은 노이즈 클러스터 제외 |

## 메모리·성능 효과 (개념)

| 항목 | freeze 전 | freeze 후 |
|------|-----------|-----------|
| 벽 plane (~200k points) | ~3 MB GPU/CPU PCD | 수백~수천 triangle mesh |
| Extract 대상 | 전체 active blocks | frozen block 제외 |
| Info 패널 | Surface points | Live surface points + Frozen objects |

성공 기준: frozen object 수가 늘수록 **Live surface points**와 extract spike가 감소하는 것.

## 종료 시 저장

| 파일 | 내용 |
|------|------|
| `scene.ply` | frozen 제외 잔여 point cloud |
| `objects/object_{id}.ply` | frozen object mesh |
| `objects/frozen_blocks.json` | id, type, mesh 경로, block keys |
| `trajectory.log` | 기존과 동일 |

## 튜닝 가이드

- **잘못된 freeze (움직이는 물체)**: Stability frames ↑, Min cluster points ↑
- **freeze가 너무 느림**: Stability frames ↓ (3~4), Min cluster points ↓ (3000)
- **cylinder 오분류**: generic mesh fallback (자동)
- **DBSCAN 과분할/미분할**: DBSCAN eps x voxel 조정 (1.5~3.0)

## 테스트

### Unit (C++)

`cpp/tests/t/geometry/VoxelBlockGrid.cpp` — `ExtractExcludingBlocks`:
integrate 후 일부 block exclude → partial point 수 감소 검증.

### Integration (bag)

```powershell
OnlineSLAMRealSense.exe --use_bag_file record.bag --profile medium --perf fast --device CUDA:0
```

30~50 extract 후 Info 탭에서 `Frozen objects: N (...)` 및 `Live surface points` 감소 확인.

## 관련 파일

- `cpp/open3d/t/geometry/VoxelBlockGrid.{h,cpp}` — partial extract
- `cpp/open3d/t/pipelines/slam/Model.{h,cpp}` — FreezeBlocks
- `examples/cpp/ObjectMeshPipeline.h` — cluster/classify/mesh
- `examples/cpp/OnlineSLAMUtil.h` — worker/GUI 통합
