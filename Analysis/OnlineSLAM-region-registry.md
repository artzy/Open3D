# Auto-freeze 영역(면) 구획·인덱싱 관리

auto-freeze 대상 영역을 면 단위로 구획하고 인덱싱해 관리하는 기능.
대상: `OnlineSLAMRGBD`, `OnlineSLAMRealSense`, `RealTimeSLAMRealSense`
(공통 파이프라인 `examples/cpp/ObjectMeshPipeline.h`).

## 기존 한계

- DBSCAN만 사용 → 바닥과 벽이 맞닿으면 하나의 거대 클러스터가 되어 면 단위
  freeze 불가
- frozen 목록이 flat vector → "이 블록이 어느 region 소속인가" 조회 불가,
  중복 freeze 방지 없음, 긴 벽의 부분(타일) freeze 불가
- 클러스터 매칭이 centroid/OBB 근사 비교라 불안정

## 채택 방식과 알려진 대안

| 문제 | 채택 | 대안 (미채택 사유) |
|------|------|--------------------|
| 면 분리 | `DetectPlanarPatches` (Araujo & Oliveira 2020, Open3D 내장) | 반복 RANSAC(경계 없음 — 떨어진 동일 평면 병합됨), 커스텀 region growing(중복 구현), 시맨틱 ML(의존성) |
| 큰 면 세분 | 평면 좌표계 고정 타일 (기본 1.0 m, GUI 슬라이더) | 쿼드트리(인덱스 불안정, 과함) |
| 공간 인덱스 | TSDF 블록 키(Int32×3) → region ID 해시맵 | Octree(이중 자료구조), R-tree(신규 의존성), Morton 정렬(온라인 갱신 부적합) |
| 프레임 간 매칭 | 후보 블록 키의 다수결 투표 | centroid/extent 매칭(기존 방식, 근사 비교라 취약) |

블록 키 인덱스를 선택한 근거: TSDF 볼륨이 이미 16³ 복셀 블록으로 공간을
구획하므로 이를 그대로 재사용하면 O(1) 소속 조회, 결정적 재매칭, 중복 freeze
방지, `FreezeBlocks` 기존 흐름과의 일치를 모두 얻는다.

## 구현 (`ObjectMeshPipeline.h` 재구성)

### 구획 파이프라인 (`ProcessExtractedSurface`)

1. **평면 패치**: 추출 pcd(법선 포함)를 2.5 cm 다운샘플 후 `DetectPlanarPatches`
   → 패치 OBB. 법선으로 분류: |n·y| > 0.8이면 수평(centroid y로
   floor/ceiling — 월드 = 첫 카메라 프레임, y가 아래 방향), 아니면 wall.
2. **타일 분할**: 법선에서 유도한 정준(canonical) 평면 축 u, v로 점을 투영해
   `(i, j) = floor(u·p / tile), floor(v·p / tile)` 타일 버킷 생성.
   `min_tile_points`(1500) 미만 타일은 무시.
3. **잔여 점**: 패치에 속하지 않은 점만 기존 DBSCAN → box/cylinder/generic.

### RegionRegistry (스레드 안전)

```text
Region { id, type, state(kObserved→kStable→kFrozen), plane(n,d),
         tile(i,j), stable_frames, area_m2, block_keys }
인덱스: unordered_map<BlockKey, region_id>
```

- **매칭**: 후보의 블록 키로 다수결 투표. 과반이 기존 unfrozen region이면 동일
  region으로 갱신(stable_frames++), frozen region에 30% 이상 겹치면 폐기
  (중복 freeze 방지), 아니면 신규 region 등록.
- **freeze**: stable_frames ≥ stability_frames(5)에 도달하면 kStable →
  mesh 생성(`CreatePrimitiveMesh`: 면은 얇은 box, generic은 marching cubes) +
  `model.FreezeBlocks` → kFrozen.
- **소멸**: 재관측되지 않은 unfrozen region은 제거하고 블록 매핑 회수.
- 타일 경계에 걸친 블록은 최초 소속 region에 귀속 (인덱스 선점).

### GUI / 저장

- Info 탭에 region 목록: `#id type state tile(i,j) area` (frozen 우선 8줄)
- "Planar tiles" 토글, "Tile size" 슬라이더(0.5~2.0 m) 추가
- `objects/frozen_blocks.json` 확장 (기존 필드 유지, 하위 호환):

```json
{ "id": 0, "type": "floor", "state": "frozen",
  "plane": [-0.03, -0.999, -0.022, 0.404], "tile": [1, -1],
  "area_m2": 0.152, "mesh": "object_0.ply", "block_keys": [...] }
```

- 씬 지오메트리 이름 `region_{id}`로 통일

### 정지 상태 통합 스킵 (같이 반영)

카메라가 정지하면(이동 < 5 mm, 회전 < 0.3°/frame이 연속 30프레임) Integrate와
free-space carving을 건너뛴다 — weight가 이미 포화된 뒤의 통합은 GPU 낭비이고
경계 노이즈가 새 블록만 할당하기 때문. 추적·GUI는 그대로 유지되고 움직이면
즉시 재개. Info 탭에 "Integration idle (camera stationary)" 표시.
상수: `kStationaryTranslationMax/RotationMaxDeg/Frames` (`OnlineSLAMUtil.h`).

### 수정 중 발견한 버그

`GetUniqueBlockCoordinates(pcd)`에 CPU 포인트클라우드를 그대로 넘기면 CUDA
touch 커널이 host 메모리를 읽어 illegal memory access → 프로세스 크래시.
`CollectBlockKeys`에서 블록 크기의 절반으로 다운샘플 후 볼륨 디바이스로 옮겨
호출하도록 수정 (공유 frustum 해시맵 오버플로도 함께 방지).

## 테스트 (2026-07-02)

```powershell
cmake --build d:\study\Open3D\build --config Release --target OnlineSLAMRGBD OnlineSLAMRealSense RealTimeSLAMRealSense
cd d:\study\Open3D\build\bin\examples\Release
.\OnlineSLAMRGBD.exe --device CUDA:0
```

| 테스트 | 결과 |
|--------|------|
| CUDA 150초 Lounge — floor/wall 타일 region 분리·freeze | PASS (32 region frozen, floor/wall 타입·타일 인덱스 정상) |
| 중복 freeze 방지 (같은 타일 재동결 없음) | PASS — frozen 겹침 후보 폐기 동작 |
| JSON에 plane/tile/state/area 기록 | PASS (`objects/frozen_blocks.json`) |
| scene.ply / trajectory.log / object_*.ply 저장 | PASS |
| CPU 45초 sanity | 별도 실행 확인 |

### 알려진 동작

- 종료 시 세그먼테이션 워커가 진행 중인 패치 검출을 마칠 때까지 종료가 수십 초
  지연될 수 있음 (기존 DBSCAN 구조와 동일).

### 수동 테스트 (D415 실기)

1. 방을 천천히 스캔 → Info 탭에 wall/floor 타일 region이 observed → frozen으로
   전환되는지 확인
2. 정지 상태 30프레임 후 "Integration idle" 표시 + Active blocks 증가 정지 확인
3. 종료 후 `objects/frozen_blocks.json`의 plane/tile 값 확인

## 조정 가능한 항목

| 항목 | 위치 | 기본값 |
|------|------|--------|
| Planar tiles on/off | GUI 토글 | on |
| Tile size | GUI 슬라이더 | 1.0 m |
| min_tile_points | `SegmentationConfig` | 1500 |
| Stability frames | GUI 슬라이더 | 5 |
| 정지 게이트 | `kStationary*` 상수 | 5 mm / 0.3° / 30프레임 |
