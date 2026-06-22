# RealTimeSLAMRealSense 실내 방 재구성 품질

**날짜:** 2026-06-16

## 증상

`scene.ply` / 외부 메시 툴에서 방이 아닌 **떠다니는 조각·이중 벽·구멍** 형태로 보임.

## 주요 원인

1. **Pose drift 후 integrate** — LOST/f2f 구간 뒤 첫 Strong 프레임을 바로 TSDF에 넣으면 기존 모델과 어긋난 조각이 생김.
2. **depth_max 3m** — 실내 원거리 벽·천장이 잘리거나 프레임마다 다르게 잘림.
3. **Integrate 비율 낮음** — Strong tier만 integrate + 엄격 threshold → 스킵 프레임 많음.
4. **포인트만 저장** — `scene.ply`를 메시 툴에서 Poisson 등으로 변환 시 노이즈·구멍이 더 부각됨.

## 적용한 개선

| 항목 | 변경 |
|------|------|
| medium `depth_max` | 3m → **5m** |
| high `depth_max` | 5m → **8m** |
| odometry 반복 | medium **6/3/2**, high **8/4/2** |
| Strong threshold | fitness **0.12**, translation **0.15m** |
| LOST 후 integrate | **연속 Strong 2프레임** 후 재개 |
| 출력 | **`scene_mesh.ply`** (TSDF mesh) 추가 |
| CLI | `--depth_max`, `--voxel_size` |
| 종료 로그 | integrate 비율, 50% 미만 시 경고 |

## 권장 스캔 방법

1. **천천히** 이동 (~0.3 m/s), 프레임 간 **30% 이상 overlap**
2. 벽·바닥·천장이 프레임에 들어오게 **팔 길이 거리** 유지
3. 큰 방: `--profile high --depth_max 8`
4. 결과 확인: **`scene_mesh.ply`** 우선 (포인트→메시 변환보다 TSDF mesh가 낫음)
5. 콘솔 **integrated (%)** — 70% 이상이면 양호, 50% 미만이면 재스캔

## 실행 예

```powershell
cd d:\study\Open3D
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe --device CUDA:0 --profile high --depth_max 8
```

종료 후 `scene_mesh.ply`를 MeshLab / Open3D Viewer로 열어 확인.
