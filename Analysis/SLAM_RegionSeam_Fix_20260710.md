# Region 간 갭(seam) 수정 — 2026-07-10

## 증상

인접 region freeze 시 `region_N.ply` 메시 사이에 빈 공간(gap)이 생김.

## 원인

1. **Block key 수집 범위가 좁음** — DBSCAN 클러스터 포인트 기준 block만 freeze → 인접 region 경계 block이 빠짐
2. **1-block halo만 사용** — marching cubes용 3×3×3 확장만으로는 region 간 overlap 부족
3. **Sanitize bounds가 cluster AABB에 고정** — 경계 삼각형이 잘려 region mesh가 서로 맞닿지 않음

## 수정 (`ObjectMeshPipeline.h`)

| 항목 | 내용 |
|------|------|
| `ExpandBlockKeysByBlockRadius` | 가변 반경 block dilation (기본 seam radius ≥ 2) |
| `CollectNeighborBlockKeys` | 기존 frozen block 중 새 region raw key 근처 block을 mesh/freeze에 포함 |
| `BlockKeysWorldAABB` | sanitize bounds를 block grid AABB와 cluster bounds union으로 확장 |
| `bounds_margin` | 최소 1 block extent 보장 |

`ApplyFreezeAndExtractMesh` 흐름:

```
raw_keys → dilate(seam_radius) → merge adjacent frozen neighbors → freeze + extract → sanitize(확장 bounds)
```

## 검증

```powershell
RealTimeSLAMRealSense.exe --regions --profile low --region_interval 30 --region_stability 3
```

- 60s: region 0·1 저장, CUDA error 없음
- block 수 증가 (예: region 0 788→1143 blocks) — seam overlap 반영

로그: `Analysis/realtime_seam_fix_out.txt`
