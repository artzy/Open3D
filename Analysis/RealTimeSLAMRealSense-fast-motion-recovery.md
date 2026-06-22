# RealTimeSLAMRealSense 급가속 Integrate 스킵 대응

**날짜:** 2026-06-16  
**대상:** [`examples/cpp/RealTimeSLAMRealSense.cpp`](../examples/cpp/RealTimeSLAMRealSense.cpp)

## 배경

급격한 카메라 움직임 시 frame-to-model odometry가 실패하면 Integrate가 스킵되어 TSDF/scene이 멈추고, raycast pose가 실제 카메라와 어긋나 트래킹 악순환이 발생한다.

## 정책 (사용자 선택 반영)

| Tier | 조건 | Pose | Integrate |
|------|------|------|-----------|
| **Strong** | fitness ≥ 0.15, translation < 0.12 m | model odometry | O |
| **Weak** | fitness ≥ 0.08, translation < 0.30 m | f2f 성공 시만 | X |
| **Outlier** | translation ≥ 0.50 m | 유지 | X |
| **Fail** | 그 외 | f2f 시도 후 성공 시만 | X |

- **Weak tier에서도 integrate 하지 않음** → ghosting 최소화 (구멍은 허용).
- model tracking Fail + fitness < 0.08 → `depth_diff × 2`로 **1회 재시도**.
- model Weak/Fail/Outlier(단, Outlier는 f2f 생략) → **frame-to-frame RGBD odometry**로 pose 브릿지 (integrate 없음).

## 구현 요약

1. `TrackingTier`, `ClassifyTracking()`, `SafeOdometryDepthDiff()`
2. `SlamWorker`: Strong만 `Integrate` + `trajectory.log` 기록
3. `consecutive_tracking_failures` — Strong 복귀 시 0, 5 초과 시 LOST 메시지
4. `RealTimeVisualizer::SetStatusTitle()` — 창 제목에 Frame/blocks/LOST 표시
5. 종료 로그: strong pose 수, f2f bridge 성공 횟수

## 자동 회귀 테스트 (2026-06-16)

환경: D415, CPU:0, profile low, 10초

```
cmake --build build --config Release --target RealTimeSLAMRealSense
cd d:\study\Open3D
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe --device CPU:0 --profile low --no-align
```

결과:

- 빌드 성공
- frame 0~27 **tier strong** (정지/저속 장면)
- `SLAM frame N | hash blocks M | tier ...` 로그 정상
- empty bbox 경고 없음 (blank-screen fix 유지)

## 수동 검증 시나리오 (권장)

1. **정상 스캔** 5초 — hash blocks 증가, 3D scene 갱신
2. **급격히 흔들기** 2초 — `Tracking weak/fail/outlier` 경고, blocks 정지, 창 제목 `LOST N — move slowly back`
3. **천천히 이전 영역 복귀** — tier `strong` 재개, blocks 재증가
4. **ESC 종료** — `trajectory.log`에 strong pose만, LOST 구간 pose 중복 없음

## 한계

- LOST 구간 geometry는 **영구 구멍** (integrate 안 함).
- f2f pose drift 가능 → Strong 복귀 전까지 TSDF 오염은 없음.
- f2f는 model tracking 실패 시에만 실행 (fps 영향 제한).

## 실행 예

```powershell
cd d:\study\Open3D
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe --device CUDA:0 --profile medium
```

급가속 후: **천천히** 이미 스캔한 영역으로 돌아가면 reconstruction이 재개된다.
