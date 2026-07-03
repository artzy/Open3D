# 평면 우선 Freeze와 성장형 메시 연결

Freeze 시 처음부터 폐곡면(얇은 박스)을 만들지 않고, 1차로 **평면(open
surface)** 을 freeze한 뒤 인접 평면을 연결해 메시를 키워가며 폐곡면에
점진적으로 접근하는 방식.

대상: `OnlineSLAMRGBD`, `OnlineSLAMRealSense`, `RealTimeSLAMRealSense`
(공통 파이프라인 `examples/cpp/ObjectMeshPipeline.h`).

## 배경

기존 구현은 평면 타일도 `CreateWallMesh`로 얇은 **박스(닫힌 폐곡면)** 를
타일마다 생성했다. Lounge 150초 기준 박스 32개가 따로 떠 있고, 서로 이어지지
않으며, 성장하지도 않았다.

## 방법 조사와 채택

| 방법 | 평가 |
|------|------|
| **점유 그리드 quad 메시 + 교선 스냅 (채택)** | 평면별 2D 셀 점유에서 셀당 quad 생성. 셀 추가만으로 메시가 자람. 이웃 평면과는 평면-평면 교선에 경계 정점을 스냅해 모서리 봉합. 온라인·저비용 |
| 2D concave hull (alpha shape) | 경계는 매끈하나 점 추가마다 전체 재계산, 병합 복잡 |
| PolyFit (Nan & Wonka 2017) | 평면 배열 + 정수계획법으로 완전 watertight. 전역 최적화라 온라인 성장 부적합 — **스캔 종료 후 후처리 후보** |
| Kinetic Shape Reconstruction | 대규모 실내에 강력하나 신규 의존성/구현 규모 초과 — 문서화만 |
| Poisson + 평면 투영 | 폐곡면은 얻지만 평면 선명도 저하, TSDF와 중복 |

## 구현 (`ObjectMeshPipeline.h`)

### 1단계: 평면 freeze = open quad 메시 (`PlaneSurfaceAtlas`)

- 타일이 freeze되면 점들을 평면에 투영해 **0.25 m 셀 점유 그리드**(tile_size/4)에
  누적 (평균 색 저장). 평면에서 `1.5 x surface_merge_dist`(3 cm) 이상 떨어진
  점은 다른 기하로 간주해 제외.
- **셀 커버리지 제약**: 셀은 기대 밀도(복셀당 1점 기준, 0.25 m 셀 ≈ 1800점)의
  `min_cell_coverage`(기본 20%) 이상 관측 포인트가 쌓여야 렌더링됨 —
  **확인된 클라우드 포인트가 있는 영역 안에서만 평면이 형성**되고, 산발적
  노이즈로 셀이 커지지 않는다. (`min_cell_points`=10은 절대 하한)
- **경계 quad 클리핑**: 셀마다 점의 (u, v) 범위를 기록하고, 경계 격자 정점을
  인접 점유 셀들의 점 범위 안으로 clamp — 부분만 관측된 경계 셀의 quad가
  셀 전체(0.25 m)로 튀어나오지 않고 실제 점 범위에서 끝난다. 내부 정점은
  격자 위치 그대로라 메시 연속성 유지.
- 메시 = 점유 셀당 quad(삼각형 2개), 인접 셀과 정점 공유 → **납작한 open
  메시**. `CreatePrimitiveMesh`의 평면 타입 박스 생성은 제거.
- 셀 좌표는 법선에서 유도한 정준(canonical) 평면 축으로 계산해 프레임 간
  재현 가능.

### 2단계: 동일 평면 병합 (성장)

- 새 타일이 기존 표면과 coplanar(법선 5° 이내, |Δd| ≤ 2 cm, 타입 동일)이면
  그 표면에 셀 병합 — 표면 개수는 유지되고 메시가 자란다.
- 평면 계수는 샘플 수 가중 평균으로 정련 (드리프트 억제). 셀 축은 생성 시
  고정해 셀 인덱스가 흔들리지 않음.
- 같은 id로 재방출된 메시는 소비자가 **교체**: OnlineSLAM은 `frozen_objects_`
  entry 교체 + 씬 `region_{id}` remove-then-add, RealTime은 id→mesh 맵으로
  `UpdateGeometry`.

### 3단계: 인접 평면 연결 (corner snap)

- 이웃 후보: 법선 사이 각 > 30°(coplanar 제외) + AABB가 스냅 거리(1.5 x cell)
  내 접근.
- 두 평면의 **교선**을 `[n1; n2; dir]` 3x3 시스템으로 풀고, 경계 정점(한
  셀에만 속한 격자 변의 정점) 중 교선까지 거리 ≤ 스냅 거리인 것을 교선 위로
  투영 — 벽-벽, 벽-바닥 모서리가 틈 없이 봉합된다.
- **스냅 게이팅**: 교선은 무한 직선이므로, 투영 지점이 **이웃 평면의 실제
  점유 셀(3x3 이웃) 근처일 때만** 스냅한다. 이웃 기하가 관측되지 않은
  구간으로 경계가 늘어나 평면이 커지는 것을 방지.
- 성장으로 이웃 관계가 바뀔 수 있으므로 dirty 표면의 이웃도 함께 재생성.
- **폐합도(closure)** = 스냅된 경계 정점 비율. Info 탭과 JSON에 표시.

### 건축 표면 필터 (가구 오분류 방지)

평면 패치가 무조건 wall/floor로 승격되면 소파 시트가 "floor", 등받이가
"wall"이 되어 납작하게 변형된다. 승격 조건을 추가:

- **수평 패치**: 기존 바닥/천장 높이 밴드(`floor_band` 0.15 m) 안에 있을 때만
  floor/ceiling. 힌트가 없으면 `min_patch_area_m2`(1.5 m²) 이상의 큰 패치만
  시드 가능. 소파 시트·테이블 상판은 거부.
- **수직 패치**: 실제 점 y span ≥ `min_wall_height`(1.2 m) **필수**.
  면적은 보조(≥ 0.25 × min_patch_area_m2)이며, 면적 단독으로 wall 승격
  불가 — 소파 옆면처럼 점 밀도만 큰 가구 패널을 차단.
- 거부된 패치의 점은 claim되지 않고 DBSCAN 객체 경로로 흘러간다.

### 객체는 실제 형상 유지

box/cylinder로 분류된 클러스터도 primitive 대신 **TSDF marching cubes
메시**를 우선 사용 (빈 메시일 때만 primitive 폴백). 소파는 스캔된 실제
형상 그대로 freeze된다. 타입 라벨은 GUI/JSON에 유지.

건축 필터는 DBSCAN 경로에도 적용된다: `ClassifyCluster`가 kWall로 재분류한
클러스터(소파 옆면·등받이 등)는 `PassesArchitecturalWallFilter`(y span
필수 + 보조 면적)를 통과해야 평면 atlas로 가고, 탈락하면 kGeneric으로
강등되어 실제 형상으로 freeze된다. y span은 `ComputeWorldYSpan`(점 범위),
면적은 `EstimateSurfaceArea`(점수 × voxel_size²)를 사용한다.

### 섬 영역 미평면화 해소 (frozen 블록 겹침 함정)

주변이 먼저 freeze된 "섬" 영역이 영원히 평면화되지 못하는 함정이 있었다:
등록 블록 키가 truncation 대역만큼 부풀려져 이웃 frozen 평면의 블록이 섬
가장자리까지 침범하고, 섬 후보는 frozen 겹침 30% 초과로 반복 폐기됐다
(진단 로그로 32~49% 겹침 반복 폐기 확인).

수정 (`RegionRegistry::UpdateBatch`):

- **core 블록 키 판정**: frozen 겹침·매칭 투표를 부풀림 없는 core 키(점이
  실제 들어 있는 블록, `CoreKeysFromPoints`)로 수행 — 이웃의 truncation
  침범 블록이 카운트되지 않음. 폐기 임계값 30% → 60%.
- **폐기 대신 축소 진행**: frozen region이 소유한 블록을 제외한 나머지
  키로 등록/freeze 진행 (경계 소유권은 먼저 얼어붙은 region 유지).

수정 후: 폐기는 core 기준 60~100% 겹침(진짜 재관측)만 발생, 이전에 32~41%
겹침으로 폐기되던 floor 후보들이 정상 freeze됨 (Lounge 90초에서 표면 7 →
10개, 주 바닥 옆 러그 높이의 floor 표면 2개 추가 확인).

## GUI / 저장

- Info 탭: `Surfaces (N): #id type area closure%` 목록 + frozen region 수
- JSON `objects/frozen_blocks.json`: 기존 `objects` 배열 유지 + `surfaces`
  배열 추가 `{id, type, plane, cell_size, cell_count, closure, connected}`
- 평면 메시 파일명 `surface_{id}.ply` (성장 시 갱신), 객체는 `object_{id}.ply`

## 테스트 (Lounge 150초, CUDA)

### 1차 (2026-07-02, 커버리지 제약 전)

- 표면 19개 (기존 박스 32개 대비 감소), 성장·연결 동작 확인
- 주 바닥 177셀 — 산발적 포인트로도 셀이 켜져 실제 관측 영역보다 큼

### 2차 (2026-07-03, 셀 커버리지 20% 적용)

- 표면 **10개**, 주 바닥 **81셀**(5.1 m²), 주 벽 37셀 — 메시가 확인된
  포인트 영역에 밀착
- 성장/폐합 유지: 바닥 closure 57%, 벽 55~100%, connected 목록 정상
- `surface_*.ply` 10개 + JSON `surfaces` 배열 정상
- CPU 45초 sanity PASS (1차에서 확인, 셀 로직은 디바이스 무관)

### 수동 테스트 (D415 실기)

1. 방 스캔 시 바닥+벽 2~3면이 연결된 open shell로 성장하는지 확인
2. Info 탭 closure %가 스캔 진행에 따라 오르는지 확인
3. 모서리(벽-벽, 벽-바닥)에 틈이 없는지 시각 확인

## 한계와 후속 후보

- 이 방식은 폐곡면에 **근접**할 뿐 완전한 watertight는 아님 (문/창/미관측
  영역은 열린 경계로 남음 — 의도된 동작).
- 완전 폐합이 필요하면 스캔 종료 후 PolyFit류 후처리(표면 plane + 경계
  입력)로 확장 가능 — 범위 외.
- 곡면(원기둥 등)은 여전히 primitive 박스/실린더로 처리.

## 조정 가능한 항목 (`SegmentationConfig`)

| 항목 | 기본값 | 의미 |
|------|--------|------|
| `min_cell_points` | 10 | 셀 점유 절대 하한 점수 |
| `min_cell_coverage` | 0.2 | 셀 렌더링에 필요한 기대 밀도 대비 커버리지 |
| `surface_merge_angle_deg` | 5.0 | coplanar 병합 법선 허용각 |
| `surface_merge_dist` | 0.02 m | coplanar 병합 offset 허용 |
| `snap_angle_deg` | 30.0 | 연결 대상 최소 평면 사이각 |
| `snap_dist_factor` | 1.5 | 스냅 거리 = factor x cell_size |
| `min_wall_height` | 1.2 m | wall 승격 최소 y span (필수) |
| `min_patch_area_m2` | 1.5 m² | floor/ceiling 시드 + wall 보조 면적 |
| `floor_band` | 0.15 m | 바닥/천장 높이 허용 밴드 |
