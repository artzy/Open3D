# OnlineSLAMRealSense 동작 불량 원인 및 수정 (2025-06-16)

## 증상
- 프로그램 종료 문제는 해결됨 (1500+ 프레임 실행 확인)
- 3D 재구성/포인트클라우드가 제대로 표시되지 않음

## 로그 근거

### 1. Surface 업데이트 누락 (확정)
- idx 50: `after_extract` 성공, `gui_callback_end` 성공
- **idx 50에서 `before_update_geometry` 없음** — 첫 surface 갱신 실패
- 첫 `before_update_geometry`는 **idx 1000**에서 발생 (약 950프레임 지연)

**원인:** `is_scene_updated_` 공유 플래그 경쟁
- SLAM worker가 idx 50에서 extract 후 `is_scene_updated_=true` 설정
- GUI callback 큐에 idx 51, 52… 가 먼저 쌓임
- idx 51 callback이 플래그를 읽고 false로 클리어 → idx 50 callback은 surface 갱신 없이 종료

### 2. Zero-padding 부작용
- `PadPointCloudToCapacity`로 (0,0,0) 점 수십만 개 추가
- Filament가 전체 800000점 렌더 → 원점 주변 거대 blob, 실제 geometry 가림

### 3. Tracking 예외 (부분)
- idx 1124+ `Singular 6x6 linear system` 예외 다수
- `TrackFrameToModel` 예외가 프레임 전체 catch → integrate/synthesize 스킵

## 수정 내용

| 항목 | 변경 |
|------|------|
| Surface 플래그 | `update_surface_this_frame`을 lambda capture (프레임별 고정) |
| Point cloud 표시 | `UpdateGeometry` → `RemoveGeometry` + `AddGeometry` (가변 점 수) |
| Padding | `PadPointCloudToCapacity` 제거 |
| Placeholder | 800k 빈 placeholder 제거 |
| Tracking | `TrackFrameToModel` try/catch → 실패 시 이전 pose 유지, 루프 계속 |
| 기본값 | block_count 16384, estimated_points 1500000, update_interval 30 |

## 검증 결과 (2025-06-16 post-fix)

로그(`debug-a85f05.log`) 기준 **수정 성공 확인**:

| 항목 | 수정 전 | 수정 후 |
|------|---------|---------|
| 첫 surface 갱신 | idx 1000 (누락) | **idx 30** (`after_update_geometry` 성공) |
| 포인트 수 | 800000 (zero padding) | **53552** (실제 추출값) |
| 실행 시간 | ~10초 크래시 | **566프레임 (~19초)** 정상 종료 |
| 예외 | tracking/frame 다수 | **0건** |

디버그 instrumentation 제거 및 Release 재빌드 완료.
