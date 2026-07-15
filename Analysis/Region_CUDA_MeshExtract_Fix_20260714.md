# Region freeze/extract CUDA 수정 및 2분 재테스트 — 2026-07-14

## 원인

`ExtractTriangleMeshIncluding/Excluding`이 include/exclude로 필터한 블록만
`inverse_index_map`에 넣고, marching cubes는 3×3×3 이웃 버퍼 인덱스로
`mesh_structure[inv[neighbor]]`에 접근 → 이웃이 맵에 없으면 잘못된 인덱스/손상 →
CUDA illegal memory access → `0xC0000409`.

부가:
- `RemeshCommittedRegion`은 halo 재확장 없이 extract
- void/holey/motion 게이트가 **extract 이후**라 정지 중에도 CUDA mesh extract 반복

## 수정

| 파일 | 내용 |
|------|------|
| `cpp/open3d/t/geometry/VoxelBlockGrid.cpp` | 이웃 buf index를 active set에 union 후 inverse_map 구축; iota는 active 길이 기준 |
| `SLAM/cpp/ObjectMeshPipeline.h` | remesh 전 1-block halo; stationary면 extract 스킵; extract start/done 로그 |
| `SLAM/cpp/RealTimeSLAMRealSense.cpp` | regions on 시 display extract 항상 2-pass (`-1`) |

## 2분 재테스트

- 결과: **STOPPED_AFTER_120S** (크래시 없음)
- exit/stderr: 정상 / 비어 있음
- frame ~3523, motion warmup 2 + stationary 232, **queued 0**
- 카메라가 거의 고정이라 mesh extract 경로는 이번 런에서 미실행

로그: `Analysis/realtime-slam-2min-cudafix-test.log`

## 추가 권장

카메라를 천천히 움직여 region commit이 나오는지 한 번 더 검증하면
CUDA mesh extract 경로까지 확인 가능.
