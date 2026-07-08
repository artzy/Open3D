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

## 수정

1. **`ObjectMeshPipeline.h`**: `ExpandBlockKeysForMeshExtract()` — mesh 추출 전 블록 키 3×3×3 halo 확장
2. **`RealTimeSLAMRealSense.cpp`**: region point extract `-1` (2-pass), `UpdateFramePose`를 `model_mutex` 안 integrate와 묶음

## 검증 명령 (PowerShell)

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\RealTimeSLAMRealSense.exe --regions --profile low --region_interval 30 --region_stability 3
```

130초 이상 실행 후 `regions/region_*.ply` 생성 및 CUDA error 없음을 확인.

**상태:** 사용자 재현 확인 완료 (2026-07-08). 디버그 instrumentation 제거됨.
