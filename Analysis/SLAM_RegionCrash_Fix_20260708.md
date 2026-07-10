# SLAM Region 실행 중 종료(CUDA crash) 수정 — 2026-07-08

## 증상

`RealTimeSLAMRealSense.exe --regions --profile low` 실행 시 약 90~2350 프레임 부근에서 종료.

- Exit code: `-1073740791` (STATUS_STACK_BUFFER_OVERRUN / fast fail)
- stderr: `CUDA illegal memory access` (`MemoryManagerCUDA.cpp`)

## 재현 테스트 결과

| 테스트 | 조건 | 결과 |
|--------|------|------|
| 수정 전 | `--regions`, 130s | frame 93에서 crash (freeze mesh extract 시작 직후) |
| 수정 전 | `--profile low` only, 130s | frame 3831, 정상 |
| 수정 후 | `--regions`, 130s | frame 3828, 정상, region 0·1 저장 |

## 가설 검증 (debug-a85f05.log)

| ID | 가설 | 결과 | 근거 |
|----|------|------|------|
| A | region extract 200000 고정 버퍼 overflow | **부분 기여** | 이전 로그에 `Point cloud size larger than estimated` 대량 발생. `-1` 2-pass로 제거됨 |
| B | display extract budget 부족 | **기각(단독 원인 아님)** | budget 2.5M, 실제 포인트 ~27k 수준 |
| C | region freeze `ExtractTriangleMeshIncluding` CUDA crash | **확정** | pre-fix: `region_freeze start` 후 `done` 없이 crash. post-fix: `vertices:5960` 등 done 로그 |
| D | GUI/Filament 단독 | **기각** | regions 없이 130s+ 정상 |
| E | `UpdateFramePose` mutex 밖 호출 | **수정 적용, 보조** | integrate 블록 안으로 이동 (동시 freeze와 pose 경합 방지) |

## 근본 원인

`ApplyFreezeAndExtractMesh` → `ExtractTriangleMeshIncluding`이 **클러스터 블록만** 대상으로 marching cubes를 실행할 때, 이웃 블록이 `inverse_index_map`에 없어 CUDA 커널이 잘못된 인덱스로 mesh structure에 접근 → illegal memory access.

## 수정 (RealTimeSLAMRealSense)

1. **`ObjectMeshPipeline.h`**: `ExpandBlockKeysForMeshExtract()` — mesh 추출 전 블록 키 3×3×3 halo 확장
2. **`RealTimeSLAMRealSense.cpp`**: region point extract `-1` (2-pass), `UpdateFramePose`를 `model_mutex` 안 integrate와 묶음

## OnlineSLAMRGBD region polygon 이식 (2026-07-10)

RealTime과 동일 UX: `--regions`, `--region_interval`, `--region_stability`, `--region_min_points`, `--region_dir`.

### 변경 파일

| 파일 | 내용 |
|------|------|
| `ObjectMeshPipeline.h` | `BuildLiveRegionSegmentationConfig`, `RegionParams`, `SaveFrozenRegion`, `WriteRegionsJson`, voxel 기반 `DownsamplePointCloudIfNeeded` |
| `RealTimeSLAMRealSense.cpp` | 공용 config/저장 함수 사용 |
| `OnlineSLAMUtil.h` | `RegionSettings`, RegionWorker 패턴 `SegmentationWorker`, extract `-1`, mutex 내 `UpdateFramePose`, GUI regions 표시, 종료 시 `regions.json` |
| `OnlineSLAMRGBD.cpp` | CLI, `--regions` 시 `auto_freeze=1`, `min_points` 기본 2000 |

### file playback(lounge) 보정

데이터셋 재생은 초기 프레임부터 전체 장면이 빠르게 통합되어 RealTime 라이브 스캔과 extract 규모가 다름. 추가 조정:

- **`RegionExtractWeightThreshold`**: RealTime과 동일하게 weight ≥ 1.0 (GUI용 낮은 threshold와 분리)
- **voxel downsample**: region DBSCAN 입력을 deterministic하게 유지 (RandomDownSample은 클러스터 추적 불안정)
- **file playback matching 완화**: `centroid_match_eps`, `extent_iou_min`, `max_cluster_extent_m` 완화

### 검증 결과

| 테스트 | 조건 | 결과 |
|--------|------|------|
| regions on | `OnlineSLAMRGBD --default_dataset lounge --regions --region_interval 30 --region_stability 3`, 120s | `regions/region_0.ply` + `regions.json` 생성, CUDA error 없음 |
| regions off | `OnlineSLAMRGBD --default_dataset lounge`, 35s+ | 정상 실행, CUDA error 없음 |

로그: `Analysis/online_rgbd_regions_test6_out.txt`, `Analysis/online_rgbd_noregions_test_out.txt`

## 검증 명령 (PowerShell)

```powershell
cd d:\study\Open3D\SLAM\bin\Release

# RealTime
.\RealTimeSLAMRealSense.exe --regions --profile low --region_interval 30 --region_stability 3

# OnlineSLAMRGBD
.\OnlineSLAMRGBD.exe --default_dataset lounge --regions --region_interval 30 --region_stability 3
.\OnlineSLAMRGBD.exe --default_dataset lounge
```

130초(또는 lounge EOF) 이상 실행 후 `regions/region_*.ply` 생성 및 CUDA error 없음을 확인.

**상태:** RealTime 사용자 재현 확인 완료 (2026-07-08). OnlineSLAMRGBD lounge 테스트 완료 (2026-07-10).
