# 구멍 난 폴리곤 Mesh 지연 (Defer-to-Last) 구현

날짜: 2026-07-14

## 목표

유효한 triangle이 있어도 readiness가 낮으면(구멍이 큰 mesh) TSDF block freeze/저장을 미루고, 완전한 region을 먼저 commit한다. holey region은 재스캔으로 품질이 오르거나 flush 조건(종료/시도 한도/남은 후보가 holey뿐)에서만 mesh화한다.

## 변경 파일

| 파일 | 내용 |
|------|------|
| `SLAM/cpp/ObjectMeshPipeline.h` | readiness 점수, MarkFreezeDeferred, ProcessPendingRegionCandidates, CommitRegionFreeze |
| `SLAM/cpp/RealTimeSLAMRealSense.cpp` | RegionWorker 정렬/defer/flush + CLI |
| `SLAM/cpp/OnlineSLAMUtil.h` | segmentation worker 동일 로직 + exit flush |
| `SLAM/cpp/OnlineSLAMRGBD.cpp` | CLI 옵션 |

## Readiness 신호

1. **yield** = mesh_vertices / surface_points (block_keys 범위 ExtractPointCloudIncluding)
2. **occupancy** = cluster voxel cell 점유율
3. **boundary_ratio** = open boundary edges / total edges

`score = 0.4*yield_term + 0.3*occupancy_term + 0.3*boundary_term` (wall은 boundary 가중치 0.15)

## CLI

- `--region_min_readiness F` (default 0.65)
- `--region_max_holey_defer N` (default 40)
- `--region_no_holey_defer`

## 검증 로그

- `Analysis/realtime-slam-holey-defer-test.log` — readiness=0.73으로 즉시 committed
- high-threshold 재테스트 결과는 동일 폴더의 `realtime-slam-holey-defer-highthresh.log` 참고

## 기대 로그

- `Region candidate N deferred (holey): readiness=...`
- `Region candidate N flushed on exit/stop: readiness=... -> committed`
- `Region worker: committed X, deferred Y holey, failed Z empty`
