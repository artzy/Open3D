# Region 미생성 원인 — DBSCAN eps vs downsample (2026-07-15)

## 증상

2분 테스트 (`realtime-slam-2min-blue-current-20260715.log`):

- filtered/queued **39회**
- frame 240만 `tracked 3` (split 성공)
- 이후 전부 `tracked 0` → Saved region **0**

## 원인

1. region 입력이 커지면 (`filtered` 평균 ~40만점) `DownsamplePointCloudIfNeeded`가 60k 이하로 맞추려고 **voxel을 키움**
2. DBSCAN eps는 계속 `4 * params.voxel_size` (~0.023 m) 고정
3. 다운샘플 간격 > eps → 거의 모든 점이 noise → 클러스터 0 → tracked 0

frame 240은 입력 ~12만점이라 다운샘플이 약해 eps와 맞아 `tracked 3`이 나왔고, 이후 맵이 커지며 깨짐.

## 수정

- `DownsamplePointCloudIfNeeded`가 사용한 voxel을 반환
- region worker에서 `dbscan_eps = max(기존, 4 * downsample_voxel)`
- 로그: `Region downsample: N -> M points (voxel=..., dbscan_eps=...)`

적용 파일: `ObjectMeshPipeline.h`, `RealTimeSLAMRealSense.cpp`, `OnlineSLAMUtil.h`
