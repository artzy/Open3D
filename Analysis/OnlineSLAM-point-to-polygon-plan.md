# 포인트 클라우드 → 적절한 크기 폴리곤 재구성 계획

## 1. 목표

현재는 **건축 표면을 평면 단위(freeze region + PlaneSurfaceAtlas)** 로 얼린 뒤,
고정 격자(0.25 m 셀) quad mesh로 대체한다. 목표는 동일한 온라인 SLAM 맥락에서
**포인트 분포·곡률·경계에 맞는 크기의 폴리곤(삼각형/quad)** 으로 재구성하는 것이다.

| 구분 | 현재 | 목표 |
|------|------|------|
| Freeze 단위 | RegionRegistry 안정 타일(≈1 m) → coplanar surface 병합 | 유지하되 **메시 생성 단위**를 세분화 |
| 폴리곤 크기 | `cell_size = tile_size/4` 고정 | **적응형** (밀도·오차·경계 기반) |
| 경계 | 격자 clip + SnapToNeighbors | **실제 2D 윤곽** + 인접면 스냅 |
| 비평면 | DBSCAN → marching cubes | 유지; 필요 시 **국소 평면 패치** 추가 |
| watertight | open surface (의도) | 1단계는 open 유지, 종료 후 선택적 폐합 |

**요구사항 (고정)**

- 온라인 증분: 프레임마다 전역 재계산 없이 **dirty region만** 갱신
- 기존 `RegionRegistry` / `FreezeBlocks` / `source_snapshot` 흐름 유지
- 건축 필터(가구→wall 오분류 방지) 유지
- CPU 렌더·PLY 저장 경로 호환 (`FrozenObjectCandidate`, GUI 토글)
- Tensor CPU+CUDA 경로는 후속; 1단계는 legacy `geometry::` 로 구현

**테스트 기준 (고정)**

- Lounge 90~150 s: 표면 수·면적·closure·RMSE 회귀 없음
- 바닥/벽: mesh가 **확인된 포인트 밖으로 확장하지 않음** (기존 coverage gate 유지)
- 소파/가구: 평면 atlas로 흡수되지 않고 객체 mesh 유지
- D415 실기: 모서리 스냅·시각적 갭·Info RMSE

---

## 2. 현재 파이프라인 요약

```
TSDF extract (surface_pcd)
  → PartitionPlanarTiles (DetectPlanarPatches, 1 m 타일)
  → RegionRegistry (stability_frames) → FreezeBlocks
  → PlaneSurfaceAtlas.AddTile (0.25 m 셀 누적)
  → BuildSurfaceMesh (occupied cell → per-cell quad)
  → SnapToNeighbors + RepositionVerticesOnPlane
  → frozen emit (region_{id}, source_{id})
  → 잔여 DBSCAN → marching cubes (object_{id})
```

**한계 (평면 단위 freeze의 구조적 한계)**

1. **폴리곤 크기가 공간과 무관** — sparse 구역도 0.25 m quad, dense 구역도 동일
2. **격자 정렬** — 경계가 scan 윤곽과 어긋남 (셀 min/max_u clip으로 완화했으나 격자 artifact 잔존)
3. **평면 가정** — 미세 경사·텍스처 변화는 단일 plane + flat mesh로 평탄화
4. **freeze = 표면 타입 결정** — 타일이 한 번 wall/floor로 고정되면 이후 세밀 재분할 어려움

---

## 3. 접근 전략: 3계층 재구성

평면 freeze **정책**은 유지하고, **메시 생성기**만 교체·확장한다.

```
[Layer A] Region / freeze  (변경 최소)
[Layer B] Surface partition  (평면 위 2D 패치 분할)
[Layer C] Polygon mesher  (적응형 크기 폴리곤)
```

### Layer B — 표면 위 2D 패치

각 `PlaneSurfaceAtlas::Surface` 안에서:

1. 누적 `source_archive` + `cells`를 평면 (u,v)로 투영
2. **패치 경계** 추출 (아래 알고리즘 중 선택·조합)
3. 패치마다 독립 mesh + metadata (`patch_id`, bounds, avg_error)

| 방법 | 장점 | 단점 | 온라인 적합 |
|------|------|------|-------------|
| **A. Adaptive quadtree** | 구현 단순, 기존 셀 구조 확장 | 경계 계단 | ★★★ |
| **B. 2D alpha shape / concave hull** | scan 윤곽 반영 | 점 추가 시 hull 재계산 | ★★ |
| **C. Greedy plane graph (PolyFit lite)** | 건축 모서리 선명 | 전역 최적화 성향 | ★ (오프라인) |

**1단계 채택: A (adaptive quadtree) + B (경계만 hull)**

- **내부**: quadtree refinement (오차·밀도 기준)
- **외곽**: occupied 영역의 2D concave hull로 clip (alpha = 2× voxel_size)

### Layer C — 적응형 폴리곤 크기

**“적절한 크기” 정의 (정량)**

| 파라미터 | 의미 | 초기값 |
|----------|------|--------|
| `poly_min_edge` | 최소 변 길이 | `2 × voxel_size` (~6 mm) |
| `poly_max_edge` | 최대 변 길이 | `cell_size` (0.25 m) |
| `poly_max_plane_error` | 평면 fit 잔차 허용 | `0.5 × surface_merge_dist` |
| `poly_min_points` | 리프 노드 최소 점수 | `min_cell_points` |

**Refinement 규칙 (quadtree, per surface)**

```
리프 셀 (u,v 범위, 점 집합 P):
  if |P| < poly_min_points → 폐기 (기존 occupy gate와 동일)
  if plane_rmse(P) > poly_max_plane_error → 4분할
  if max_edge > poly_max_edge AND |P| > threshold → 4분할
  if max_edge < poly_min_edge * 2 → 병합 시도 (인접 형제)
  else → 리프: quad 1개 (min_u..max_u × min_v..max_v)
```

- 리프 quad는 현재 `BuildSurfaceMesh`와 동일하게 `CornerPosition` + plane 투영
- **SnapToNeighbors**는 리프 경계 정점에만 적용 (기존 boundary_vertices 로직 재사용)

**대안 (2단계): Delaunay + edge collapse**

- (u,v) Delaunay triangulation 후 `poly_max_edge` 초과 변 split
- quad보다 삼각형 수 증가하나 곡률·경사에 유리 → 2단계 옵션

---

## 4. 알고리즘 파이프라인 (목표)

```
ProcessExtractedSurface (기존)
  … AddTile → dirty surface …
  RebuildDirtyMeshes:
    1. Collect occupied cells (기존 gate)
    2. [NEW] BuildAdaptivePatchMesh(surface):
         a. Project archive points → (u,v)
         b. Optional: 2D alpha boundary mask
         c. Quadtree refine → leaf quads/tris
         d. SnapToNeighbors + RepositionVerticesOnPlane
    3. Emit FrozenObjectCandidate (+ source_snapshot, dense overlay 선택)
```

**포인트 → 폴리곤 입력**

- **Primary**: `source_archive` (freeze 직전 누적, plane distance 필터 통과)
- **Gate**: 기존 `occupy_threshold` / `min_cell_coverage` — **확인된 영역만** mesh
- **RMSE**: 리프 패치 plane_rmse 및 mesh vertex distance (기존 `compare_rmse` 확장)

---

## 5. 데이터 구조 변경 (안)

`PlaneSurfaceAtlas::Surface` 확장:

```cpp
struct PatchLeaf {
    double min_u, max_u, min_v, max_v;
    int point_count;
    double plane_rmse;
    Eigen::Vector3d color;
};

struct Surface {
    // … existing …
    std::vector<PatchLeaf> patches;  // last rebuild
    geometry::PointCloud source_archive;
};
```

`FrozenObjectCandidate` / JSON (선택):

```json
"mesh_mode": "adaptive_quadtree",
"patch_count": 142,
"poly_min_edge": 0.006,
"poly_max_edge": 0.25
```

GUI Info: `#id wall 5.1 m2  patches 142  rmse 3.2 mm`

---

## 6. 구현 단계

### Phase 0 — 설계 고정 (1~2일)

- [ ] 파라미터 기본값 확정 (`SegmentationConfig` 확장)
- [ ] `BuildSurfaceMesh` vs `BuildAdaptivePatchMesh` feature flag
- [ ] 회귀 테스트 시나리오 문서화 (Lounge + D415)

### Phase 1 — Adaptive quadtree mesher (1주)

**파일**: `ObjectMeshPipeline.h` (또는 `PlanarPolygonMesher.h` 분리)

- [ ] `BuildAdaptivePatchMesh` 구현, flag `use_adaptive_polygons`
- [ ] `RebuildDirtyMeshes`에서 분기
- [ ] boundary_vertices: 리프 외곽 edge (기존 snap 호환)
- [ ] `BuildDenseCellSourceOverlay` → 리프 grid로 통일
- [ ] Lounge 회귀: area ±10%, RMSE ±20%, closure 유지

### Phase 2 — 2D 경계 clip (3~5일)

- [ ] occupied (u,v) 점으로 alpha shape / concave hull
- [ ] hull 내부 리프만 렌더 (외부 quad 제거)
- [ ] 경계 artifact 테스트 (러그·가구 경계)

### Phase 3 — GUI / artifact (2~3일)

- [ ] Info 패널 patch_count, mesh_mode
- [ ] `frozen_blocks.json` 필드
- [ ] 토글: `Show polygon mesh` (기존 plane mesh 대체 또는 병행)

### Phase 4 — 선택 후처리 (범위 외, 문서만)

- 스캔 종료 후 PolyFit / plane arrangement → watertight B-rep
- Open3D 의존성 추가 없이 외부 도구 연동

---

## 7. 기존 코드 재사용

| 기능 | 재사용 |
|------|--------|
| Freeze / stability | `RegionRegistry`, `ProcessExtractedSurface` |
| Coplanar merge | `PlaneSurfaceAtlas::AddTile` |
| Corner snap | `SnapToNeighbors`, `ArePotentialNeighbors` |
| Source 비교 | `AttachSourceSnapshot`, `GetMeshAlignedSourcePoints` |
| 건축 필터 | `PassesArchitecturalWallFilter`, `PartitionPlanarTiles` |
| 객체 경로 | DBSCAN + marching cubes (변경 없음) |

**피할 것**

- freeze 단위를 점 단위/프레임 단위로 쪼개기 (TSDF·registry 폭발)
- 전역 Delaunay/alpha 매 프레임 (온라인 비용)
- Open3D 3rdparty 패치

---

## 8. 리스크와 완화

| 리스크 | 완화 |
|--------|------|
| quadtree 과분할 → 삼각형 폭증 | `poly_max_edge`, leaf count cap per surface |
| hull 재계산 비용 | dirty surface만; hull은 coarse grid (0.05 m) 샘플 |
| snap + reproject vs hull 경계 충돌 | snap 후 hull 밖 vertex clip |
| 가구 edge에서 hull 팽창 | 건축 필터 + hull alpha 보수적 |
| RealTime 경로 GUI 없음 | `IncrementalMeshFreeze` 동일 mesh API |

---

## 9. 검증 계획

**자동 (C++ unit, `cpp/tests/` 또는 examples 내 test hook)**

- 합성 평면 + 구멍 + 노이즈: mesh area ≤ point convex area
- Refinement: high-noise 구역만 leaf 수 증가
- `compare_rmse` ≤ 기존 fixed-grid 대비 개선 또는 동등

**수동**

1. Lounge: adaptive on/off side-by-side PLY
2. D415: 바닥 타일 경계 artifact 감소 확인
3. source vs polygon overlay: 갭·RMSE Info 패널
4. 소파 옆면 wall 오분류 재발 없음

---

## 10. 의사결정 요약

| 질문 | 결정 |
|------|------|
| freeze 단위 변경? | **아니오** — region/tile freeze 유지 |
| 평면 가정? | **1단계 유지** — surface당 1 plane + adaptive 2D mesh |
| 폴리곤 vs 삼각형? | **1단계 quad** (기존 렌더·snap 호환) |
| 적절한 크기? | **quadtree**: plane error + edge length + point count |
| watertight? | **후속** PolyFit 오프라인 |

---

## 구현 상태 (2026-07-07)

- `PlanarPolygonMesher.h`: adaptive quadtree mesher
- `SegmentationConfig.use_adaptive_polygons` (기본 **OFF**)
- `poly_use_boundary_hull`: occupied-cell convex hull clip (기본 OFF)
- GUI Settings: **Adaptive polygons**, **Polygon boundary hull**
- JSON: `mesh_mode`, `patch_count` on objects/surfaces

## 롤아웃

| flag | 동작 |
|------|------|
| `use_adaptive_polygons=false` | legacy `BuildSurfaceMesh` (0.25 m grid) |
| `use_adaptive_polygons=true` | `BuildAdaptivePatchMesh` quadtree |

회귀 통과 후 기본 ON 검토.

## 설정 (`SegmentationConfig` 추가)

| 항목 | 기본 | 의미 |
|------|------|------|
| `use_adaptive_polygons` | false | adaptive vs legacy |
| `poly_min_edge` | 0 (→ 2×voxel) | 리프 최소 변 |
| `poly_max_edge` | 0 (→ cell_size) | 리프 최대 변 |
| `poly_max_plane_error` | 0 (→ 0.5×merge_dist) | 분할 RMSE |
| `poly_max_leaf_count` | 5000 | surface당 리프 상한 |
| `poly_use_boundary_hull` | false | hull clip |
| `poly_hull_alpha` | 0.012 | hull 파라미터 (예약) |

## 파일

| 파일 | 역할 |
|------|------|
| [`examples/cpp/PlanarPolygonMesher.h`](../examples/cpp/PlanarPolygonMesher.h) | quadtree mesher, hull clip |
| [`examples/cpp/ObjectMeshPipeline.h`](../examples/cpp/ObjectMeshPipeline.h) | config, 분기, dense overlay |
| [`examples/cpp/OnlineSLAMUtil.h`](../examples/cpp/OnlineSLAMUtil.h) | GUI 토글, JSON, Info |
| [`cpp/tests/examples/PlanarPolygonMesher.cpp`](../cpp/tests/examples/PlanarPolygonMesher.cpp) | 단위 테스트 |

## 테스트

- C++: `cpp/tests/examples/PlanarPolygonMesher.cpp`
- 수동: Lounge 90~150 s, flag OFF/ON A/B, D415 실기

---

## 11. 다음 액션 (완료)

1. ~~Phase 0: config + 분기~~
2. ~~Phase 1: BuildAdaptivePatchMesh~~
3. Lounge A/B 수동 확인 (사용자)

관련 문서: [`OnlineSLAM-plane-growing-mesh.md`](OnlineSLAM-plane-growing-mesh.md),
[`OnlineSLAM-source-point-snapshot.md`](OnlineSLAM-source-point-snapshot.md)
