# 소파 옆면 평면 오분류 수정

## 증상

스캔 중 소파 본체는 marching-cubes 메시로 정상 freeze되지만, **소파 옆면**
(arm/side panel)이 흰색 평면 quad(`PlaneSurfaceAtlas`)로 대체됨.

## 원인

건축 표면 필터가 **OR 조건**이었다:

```
y_extent >= min_wall_height  OR  area >= min_patch_area_m2
```

소파 옆면은 높이 ~0.8–1.0 m로 `min_wall_height`(0.8 m)에 걸릴 수 있고,
TSDF 추출 점 밀도가 쌓이면 `EstimateSurfaceArea`(점수 × voxel²)가 1.5 m²를
넘기 쉽다. **면적만으로 wall 승격**되어 평면 atlas로 들어감.

두 경로 모두 동일한 허점:

1. `PartitionPlanarTiles` — `DetectPlanarPatches` 패치 경로
2. `ProcessExtractedSurface` — DBSCAN 잔여 클러스터가 `ClassifyCluster`로
   `kWall` 재분류되는 경로

## 수정 (`ObjectMeshPipeline.h`)

### 1. 높이 필수 (OR 제거)

`PassesArchitecturalWallFilter`:

- **y span ≥ min_wall_height** (필수)
- 면적은 보조: `patch_area >= min_patch_area_m2 × 0.25` (노이즈 sliver 제거)
- 면적 단독으로는 wall 승격 불가

### 2. y span = 실제 점 범위

OBB y 투영 대신 `ComputeWorldYSpan` (점 cloud의 max_y − min_y, y-down
좌표) 사용 — 가구 OBB가 과대평가하는 경우를 줄임.

### 3. min_wall_height 상향

0.8 m → **1.2 m**. 일반 실내 벽(2.4 m+)의 부분 스캔은 통과, 소파·등받이
(~0.5–0.9 m)는 거부.

## 기대 동작

- 소파 옆면/등받이 → `kGeneric` → marching cubes 실제 형상 freeze
- 실제 벽 → y span ≥ 1.2 m → 평면 atlas 성장 유지

## 검증

Release 빌드 (`OnlineSLAMRGBD`, `RealTimeSLAMRealSense`) 성공.

수동: Lounge 또는 실기 스캔 후 소파 옆면에 `surface_*` 평면이 생기지
않고 `object_*` 또는 live pcd 형상이 유지되는지 확인.

## 조정

| 항목 | 기본값 | 의미 |
|------|--------|------|
| `min_wall_height` | 1.2 m | wall 승격 최소 수직 span |
| `min_patch_area_m2` | 1.5 m² | floor/ceiling 시드 + wall 보조 면적 하한 |

천장 높이가 낮은 공간에서 벽이 평면화되지 않으면 `min_wall_height`를
0.1 m 단위로 낮춰 조정 (0.8 m 이하로는 가구와 구분 어려움).
