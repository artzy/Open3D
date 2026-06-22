# SLAM 성능 개선 (2026-06-16)

## 병목 요약

| 구간 | 비용 | 개선 |
|------|------|------|
| TrackFrameToModel | 매 프레임 GPU | `--perf fast` → odom 3/2/1 |
| Integrate + RayCast | 매 프레임 GPU | raycast color off (fast) |
| ExtractPointCloud | 주기적 spike | **async extract thread** |
| GPU→CPU + GUI | 매 프레임 | **gui_update_interval** (fast=3) |
| Filament PCD | extract마다 | **UpdateGeometry** in-place |

## Performance preset (`--perf`)

| preset | odom iter | GUI interval | raycast color | 용도 |
|--------|-----------|--------------|---------------|------|
| **fast** (기본) | 3/2/1 | 3 | off | 라이브 30fps |
| balanced | 4/2/1 | 2 | on | 균형 |
| quality | 6/3/1 | 1 | on | bag/품질 우선 |

`--profile`(VRAM)과 독립적으로 조합 가능.

## 실행 예

```powershell
# 권장: fast perf + medium VRAM
OnlineSLAMRealSense.exe --device CUDA:0 --profile medium --perf fast

# 라이브 RealSense
OnlineSLAMRealSense.exe --perf fast -c examples\test_data\rs_slam_lowmem.json --no-align

# 품질 우선 (bag)
OnlineSLAMRealSense.exe --profile medium --perf quality --use_bag_file record.bag
```

## GUI 슬라이더 (Starting settings)

- **Odom iter coarse/mid/fine** — RGB-D odometry 피라미드 반복 횟수
- **GUI interval** — 2D preview/Info 갱신 주기 (SLAM은 매 프레임)

## 추가 최적화 (코드)

- Async extract: SLAM worker는 flag만 set, 별도 스레드가 ExtractPointCloud
- Frame skip: 라이브에서 처리 >40ms 연속 3회 → 다음 capture 1프레임 skip
- depth_diff 2× 재시도: fitness < 0.08일 때만 (불필요한 2× odometry 방지)
- idx 3 early extract: Python dense_slam_gui와 동일

## 벤치마크 방법

1. Bag 재생으로 입력 고정
2. FPS 패널 (30프레임 rolling average) 비교
3. `--perf fast` vs `--perf quality` A/B
4. `nvidia-smi`로 VRAM 변화 없음 확인

## 트레이드오프

- fast + raycast color off → raycast 탭 컬러 없음 (depth만)
- GUI interval ↑ → 2D preview 업데이트 간격 증가 (3D surface는 extract 완료 시 갱신)
- odom iter ↓ → 급격한 카메라 움직임 시 tracking 실패 가능 → balanced/quality 사용
