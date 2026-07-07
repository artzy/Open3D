# 평면 대체 시 소스 포인트 저장 및 GUI 비교

## 개요

auto-freeze로 포인트가 평면 quad mesh(`PlaneSurfaceAtlas`) 또는 객체 mesh로
대체될 때, **대체 직전의 원본 포인트**를 `source_{id}.ply`로 보관하고 GUI에서
평면 mesh와 독립적으로 토글·비교한다.

대상: `OnlineSLAMRGBD`, `OnlineSLAMRealSense` (`OnlineSLAMUtil.h`),
`RealTimeSLAMRealSense` (`IncrementalMeshFreeze.h` — 디스크 저장만).

## 캡처 시점

`ProcessExtractedSurface` (`ObjectMeshPipeline.h`):

- **평면 타일**: `AddTile`마다 `PlaneSurfaceAtlas::source_archive`에 plane
  distance 필터 통과 점 누적 → surface rebuild emit 시 snapshot 생성
- **객체 클러스터**: freeze 시 `candidates[c].points`에서 snapshot 생성

`PrepareSourceSnapshot`으로 `voxel_size × source_pcd_downsample`(기본 2×)
voxel downsample 후 GUI/디스크에 사용.

## 아티팩트 (`objects/`)

| 파일 | 설명 |
|------|------|
| `source_{id}.ply` | 대체 전 원본 포인트 (downsampled) |
| `surface_{id}.ply` / `object_{id}.ply` | freeze mesh |
| `frozen_blocks.json` | `source_point_cloud`, `source_point_count`, `compare_rmse` |

## GUI (OnlineSLAM)

Settings 패널 독립 토글:

| 토글 | 기본 | 효과 |
|------|------|------|
| **Show source cloud** | off | `source_{id}` 포인트 표시 |
| **Show plane mesh** | on | wall/floor/ceiling `region_{id}` mesh 표시 |

비평면 객체 mesh(sofa 등)는 **Show plane mesh**와 무관하게 항상 표시.

조합:

- source on / plane off → 교체 전 스캔만
- source off / plane on → 평면만 (기본)
- 둘 다 on → 오버레이 비교 (live TSDF points는 자동 숨김)
- 둘 다 off → live pcd + 비평면 frozen mesh

Info 패널: `#id type area rmse mm (N pts)` per frozen region.

## RMSE

- mesh가 있으면 **점→mesh vertex** 거리 RMS (`ComputeMeshRmse`)
- 평면 overlay 소스는 fit plane에 **투영** 후 downsample (`ProjectPointsOntoPlane`)
- mesh와 동일 셀만 포함 (`GetMeshAlignedSourcePoints`, `occupy_threshold` 일치)

## 갭 완화 (2026-07-03)

| 원인 | 대응 |
|------|------|
| TSDF truncation band (법선 방향) | `ProjectPointsOntoPlane` |
| mesh 미생성 sparse 셀 (면 내) | occupied 셀과 동일 gate로 source 필터 |
| RMSE가 plane-only | mesh 기준 `ComputeMeshRmse` 우선 |
| lattice corner vs 셀 내 점 범위 | 셀별 quad를 `min_u/max_u × min_v/max_v`로 생성 |
| `SnapToNeighbors`가 평면 밖으로 당김 | snap 후 `RepositionVerticesOnPlane` |
| live TSDF `points`와 source overlay 혼동 | **Show source cloud** on 시 live `points` 숨김 |

## 설정 (`SegmentationConfig`)

| 항목 | 기본 | 의미 |
|------|------|------|
| `save_source_points` | true | snapshot 생성/저장 |
| `source_pcd_downsample` | 2.0 | snapshot voxel = voxel_size × 배수 |

## 검증

Release 빌드: `OnlineSLAMRGBD`, `RealTimeSLAMRealSense` 성공.

수동 (Lounge ~90초, planar tiles on):

1. freeze 후 `objects/source_*.ply` 존재, point_count > 0
2. `frozen_blocks.json`에 `source_point_cloud`, `compare_rmse`
3. GUI 토글로 plane/source 독립 show/hide
4. Info 패널 RMSE 표시
