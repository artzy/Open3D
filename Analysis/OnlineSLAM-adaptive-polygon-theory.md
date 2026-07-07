# 적응형 폴리곤 재구성 — 적용된 이론적 배경

Online SLAM freeze 파이프라인에서 **포인트 클라우드 → 적응형 폴리곤(quad mesh)** 으로
대체하는 데 사용된 기하·알고리즘·설계 원칙을 정리한다.

관련 구현: [`PlanarPolygonMesher.h`](../examples/cpp/PlanarPolygonMesher.h),
[`ObjectMeshPipeline.h`](../examples/cpp/ObjectMeshPipeline.h)

---

## 1. 문제 정식화

### 1.1 입력

- TSDF에서 추출한 **표면 포인트 클라우드** \( \mathcal{P} = \{p_i\} \)
- RANSAC/패치 검출로 얻은 **건축 평면** \( \pi: n^\top x + d = 0 \) (wall/floor/ceiling)
- freeze된 타일마다 누적된 **source archive** (평면 대체 직전 원본 점)

### 1.2 출력

- 평면 \( \pi \) 위의 **open quad mesh** \( \mathcal{M} \): 얇은 2.5D 표면 근사
- 각 quad는 확인된 관측 영역 안에만 존재 (extrapolation 금지)
- 인접 평면과 **모서리 스냅**으로 틈 최소화 (완전 watertight 아님)

### 1.3 “적절한 크기”의 의미

고정 격자(0.25 m)는 **공간 해상도와 무관**하게 동일 크기 quad를 쓴다.
적응형 방식은 다음을 동시에 만족하는 리프 크기 \( h \) 를 선택한다.

| 기준 | 의미 |
|------|------|
| **기하 오차** | 리프 내 점들의 평면 잔차 RMSE \( \le \epsilon \) |
| **공간 스케일** | \( h_{\min} \le h \le h_{\max} \) (voxel ~ cell_size) |
| **관측 밀도** | sparse 구역은 큰 quad, noisy/dense 구역은 작은 quad |
| **온라인 비용** | surface당 리프 수 상한 \( L_{\max} \) |

이는 **오차 기반 적응형 표면 단순화(error-driven surface simplification)** 의
2D 특수 경우로 볼 수 있다.

---

## 2. 기하학적 토대

### 2.1 평면 좌표계 (u, v)

법선 \( n \)과 world 축으로부터 **정준(canonical) 접공간 기저** \( (u, v) \) 를 고정한다.

\[
p = u\, u_u + v\, v_v + \text{proj}_\pi(p)
\]

- \( u = u^\top p \), \( v = v^\top p \) (평면 내 2D 좌표)
- 셀 인덱스·freeze 성장 시 축이 흔들리지 않도록 **surface 생성 시 한 번 고정**

이는 **매니폴드 파라미터화(manifold parameterization)** 의 선형 근사이며,
곡률이 작은 건축면(wall/floor)에서 유효하다.

### 2.2 평면 투영과 TSDF truncation band

TSDF 추출 점은 이상적으로 평면 위가 아니라 **truncation band** 두께만큼 법선 방향으로
퍼져 있다. 비교·RMSE·투영 시:

\[
p' = p - (n^\top p + d)\, n
\]

- **mesh vertex**: 항상 \( \pi \) 위 (`CornerPosition`)
- **적응형 분할 판정**: **원본 점** \( p \) 로 plane RMSE 계산 (band 두께 = 분할 신호)
- **GUI overlay / snapshot**: 투영·다운샘플 후 표시

즉, **렌더 mesh는 평면 위**, **적응형 해상도는 관측 두께·잡음에 반응**한다.

### 2.3 평면 RMSE (분할 기준)

리프 노드 점집합 \( P \subset \mathcal{P} \) 에 대해:

\[
\text{RMSE}(P, \pi) = \sqrt{\frac{1}{|P|} \sum_{p \in P} (n^\top p + d)^2}
\]

\( \text{RMSE} > \epsilon \) 이면 quadtree **4분할**. 이는 **piecewise-planar approximation**
에서 구간별 평면 적합도가 허용치를 넘을 때만 세분화하는 classical quadtree / BSP
아이디어와 같다.

---

## 3. 알고리즘 구조: 3계층 분리

freeze **정책**과 mesh **생성**을 분리한다.

```
Layer A  RegionRegistry + FreezeBlocks     (무엇을 언제 freeze할지)
Layer B  PlaneSurfaceAtlas (u,v 셀 누적)   (어떤 평면에 어떤 점이 있는지)
Layer C  Polygon mesher                    (점 → quad mesh)
```

- Layer A·B는 기존 **평면 성장형 freeze** 이론 유지
- Layer C만 legacy grid ↔ adaptive quadtree 로 교체 (`use_adaptive_polygons`)

이 분리는 **관심사 분리(separation of concerns)**: segmentation stability와
mesh resolution을 독립적으로 튜닝할 수 있다.

---

## 4. Layer B — 관측 게이트 (Occupy threshold)

mesh가 **확인되지 않은 영역으로 확장**하지 않도록 coarse cell \( (i,j) \) 마다:

\[
N_{ij} \ge \max\left(N_{\min},\; \rho \cdot \frac{A_{\text{cell}}}{v^2}\right)
\]

- \( N_{ij} \): 셀에 누적된 점 수
- \( v \): voxel_size, \( \rho \): `min_cell_coverage` (기본 20%)
- \( A_{\text{cell}} = \text{cell\_size}^2 \)

TSDF 표면 추출은 복셀당 대략 1점을 가정하므로, 이는 **관측 밀도 기반 신뢰 영역**이다.
적응형 mesher도 **occupy 통과 coarse cell** 을 root로만 quadtree를 시작한다.

---

## 5. Layer C — Adaptive quadtree mesher

### 5.1 Root

각 occupied coarse cell의 **실제 점 범위** \( [min_u, max_u] \times [min_v, max_v] \) 를
quadtree root로 사용한다 (legacy per-cell quad와 동일한 footprint 원칙).

### 5.2 Refinement (4분할)

스택 기반 quadtree, 노드 \( [u_0,u_1] \times [v_0,v_1] \) 에 대해:

1. 박스 내 archive 점 수집
2. \( \text{RMSE} > \epsilon \) **또는** \( \max(u_1-u_0, v_1-v_0) > h_{\max} \) 이면 4분할
3. 더 이상 분할 불가(\( h < 2 h_{\min} \))이면 **리프 quad 1개** emit
4. 리프 footprint = 해당 노드 내 점들의 **실제 min/max (u,v)** (extrapolation 없음)

이는 **2D quadtree mesh refinement** (CFD/지형 DEM 분야의 error-based refinement)와
동일한 패밀리이다. Delaunay 전역 재구성 대비 **dirty surface만** 갱신 가능해
온라인 SLAM에 적합하다.

### 5.3 리프 상한

\( |\text{leaves}| \ge L_{\max} \) 이면 분할 중단 → **LOD cap / complexity bound**.
과분할 시 렌더·저장 비용 폭증을 막는다.

### 5.4 리프 → quad mesh

리프마다 4 vertex + 2 triangle, 색은 셀 평균색.
경계 edge는 이웃 리프가 없으면 `boundary_vertices`에 등록 → **SnapToNeighbors** 입력.

---

## 6. 경계 처리

### 6.1 Convex hull clip (선택, `poly_use_boundary_hull`)

occupied coarse cell 모서리를 2D 샘플로 모아 **convex hull** (monotone chain) 구성.
리프 중심이 hull 밖이면 제거.

- **이론**: convex hull은 concave scan 윤곽의 **보수적(안쪽) 근사**
- **장점**: 구현·갱신 비용 낮음, 온라인 적합
- **한계**: 오목한 경계(러그·가구 경계)는 과도하게 잘릴 수 있음 → 기본 OFF

계획상 alpha shape / concave hull은 **정확도↑, 비용↑** 후보로 문서화만 유지.

### 6.2 평면-평면 교선 스냅 (기존 유지)

이웃 표면 \( \pi_1, \pi_2 \) 의 교선 \( \ell = \pi_1 \cap \pi_2 \) 에 대해
경계 vertex를 스냅 → **piecewise-planar room model** 의 모서리 정합.

스냅 후 `RepositionVerticesOnPlane`으로 **자기 평면에 재투영** → source overlay와
coplanar 유지 (3D 교선으로 들어올려진 vertex 보정).

---

## 7. 채택하지 않은 방법과 이유

| 방법 | 이론적 장점 | 온라인 SLAM에서 배제/후순위 이유 |
|------|-------------|-----------------------------------|
| **PolyFit (Nan & Wonka)** | 전역 plane arrangement, watertight | 정수계획·전역 최적화, 프레임마다 재계산 부담 |
| **Kinetic Shape Reconstruction** | 대규모 실내 watertight | 구현·의존성 규모 |
| **2D Delaunay + edge split** | 곡률·비균일 밀도 | 전역 triangulation 갱신 비용 |
| **Poisson + 투영** | smooth surface | 평면 선명도 저하, TSDF와 중복 |
| **Alpha shape (concave hull)** | scan 윤곽 정확 | hull 재계산; convex hull로 1단계 대체 |

현재 구현은 **quadtree + 선택적 convex hull** 로 **온라인·증분·저비용**을 우선한다.

---

## 8. 품질 지표

| 지표 | 정의 | 용도 |
|------|------|------|
| **compare_rmse** | aligned source → mesh vertex 거리 RMS | freeze 품질, Info 패널 |
| **closure** | 스냅된 경계 vertex / 전체 경계 vertex | 모서리 봉합률 |
| **patch_count** | adaptive 리프 수 | 해상도·복잡도 |
| **mesh_mode** | `legacy_grid` / `adaptive_quadtree` | A/B·JSON |

---

## 9. 롤아웃과 실험 설계

- **`use_adaptive_polygons = false` (기본)**: legacy mesh 경로 100% 유지 → **대조군**
- **`true`**: 동일 freeze·동일 source archive, mesh 생성기만 교체 → **실험군**

회귀 가설:

- occupy gate 유지 → **footprint 확장 없음**
- adaptive ON → **격자 aliasing 감소**, patch_count 증가
- RMSE·closure → legacy 대비 **동등 또는 개선**

---

## 10. 한계 (의도된 근사)

1. **단일 평면 per surface** — 미세 경사·곡면은 평탄화 (객체는 marching cubes 경로)
2. **Open surface** — 문·창·미관측은 열린 경계 (watertight 아님)
3. **2D quadtree** — 경계가 계단형(staircase)일 수 있음 (hull clip으로 일부 완화)
4. **Convex hull clip** — 오목 경계에서 보수적

완전 폐곡면이 필요하면 **스캔 종료 후 PolyFit류 오프라인 후처리**가 이론적으로
적합한 2단계이다.

---

## 11. 한 줄 요약

> **건축 평면을 (u,v)에 올린 뒤, 관측 게이트를 통과한 영역만 quadtree로
> plane-RMSE·edge-length 기준 분할하고, 리프마다 quad를 만들어 open
> piecewise-planar mesh를 구성한다. freeze·스냅·coverage는 기존 이론을 유지하고,
> mesh 해상도만 적응형으로 바꾼다.**
