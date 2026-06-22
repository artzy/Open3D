# RealSense SLAM VRAM/RAM 최적화 (2026-06-16)

## 참고 문서

- [RealSense with Open3D](https://www.open3d.org/docs/latest/tutorial/sensor/realsense.html)
- [Voxel Block Grid](https://www.open3d.org/docs/latest/tutorial/t_reconstruction_system/voxel_block_grid.html) — living room 기준 **50,000 blocks** 권장
- Python `default_config.yml` — block_count 40,000, depth_max 3.0

## VRAM 구성

| 구성요소 | 블록/버퍼당 | medium (40k) |
|---------|------------|--------------|
| TSDF 블록 (16³, tsdf+weight+color) | ~48 KB | ~1.9 GB |
| ExtractPointCloud 버퍼 | ~16 B/점 | ~96 MB (6M) |
| RGB-D 프레임 (GPU) | 해상도 의존 | ~2–6 MB |
| Raycast + Odometry | — | ~200–500 MB |

## 메모리 프로필 (`--profile`)

| 프로필 | block_count | estimated_points | depth_max | TSDF VRAM (추정) | 용도 |
|--------|-------------|------------------|-----------|------------------|------|
| **low** | 16,384 | 1,500,000 | 2.0 m | ~0.8 GB | 8GB GPU, 좁은 실내 |
| **medium** (기본) | 40,000 | 6,000,000 | 3.0 m | ~1.9 GB | RTX 3060 12GB, 일반 실내 |
| **high** | 50,000 | 8,000,000 | 5.0 m | ~2.4 GB | 넓은 실내/복도 |

## RealSense 캡처 최적화

튜토리얼 권장: 30fps 기준 **~33ms/프레임** 이내 처리.

- `examples/test_data/rs_slam_lowmem.json` — 540p 컬러, 480p 깊이, 30fps
- `--no-align` — 라이브 캡처 시 depth-to-color 정렬 생략 (처리 시간 절감)
- 고해상도(≥1024×768) 시 경고 로그 출력

## RealSense → SLAM 자동 힌트

`OnlineSLAMRealSense.cpp`가 메타데이터를 분석해 다음을 수행합니다.

| 조건 | 동작 |
|------|------|
| depth ≥ 1024×768 | 경고 + `--profile` 명시 시 `update_interval` +25 |
| L515 + SHORT_RANGE preset | `depth_max ≤ 3.0 m` 권장 로그 |
| bag (depth_scale=1000, ≤640×480) | `--profile low` 권장 로그 |

## VRAM 부족 시 조치 순서

1. `--profile low`
2. GUI에서 `depth_max` ↓
3. `voxel_size` ↑ (0.006 → 0.008)
4. `block_count` ↓
5. `estimated_points` ↓
6. `-c rs_slam_lowmem.json` + `--no-align`
7. `--device CPU:0` (최후 수단)

## 실행 예

```powershell
# 기본 (medium, RTX 3060)
OnlineSLAMRealSense.exe --device CUDA:0 --profile medium

# 저메모리 + 저해상도 캡처
OnlineSLAMRealSense.exe --profile low -c examples\test_data\rs_slam_lowmem.json --no-align

# Bag 재생
OnlineSLAMRealSense.exe --profile medium --use_bag_file record.bag
```

## 코드 변경 요약

- `OnlineSLAMRealSense.cpp`: `--profile`, `--no-align` CLI
- `OnlineSLAMUtil.h`: medium 기본값, `GetSlamProfile()`, VRAM 예산 로그, 디버그 로그 제거
- 95% hash fill 시 Integrate 스킵 (기존 유지)
