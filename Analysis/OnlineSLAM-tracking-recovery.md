# OnlineSLAM Tracking Outlier 복구 방법론

**날짜:** 2026-07-07  
**대상:** [`examples/cpp/OnlineSLAMUtil.h`](../examples/cpp/OnlineSLAMUtil.h),  
[`examples/cpp/SlamTrackingRecovery.h`](../examples/cpp/SlamTrackingRecovery.h)

## 배경

급격한 카메라 움직임·텍스처 부족 등으로 model tracking 이 Outlier/Fail 이면 integration 이
중단되고 `Tracking outlier ... Skipping integration.` 경고가 **프레임마다** 출력된다.
이 경고는 **억제하지 않는다** — 문제가 지속되는 동안 계속 표시되어야 한다.

문제는 integration 중단만으로는 부족하고, **포즈가 고정**되면 raycast 모델이 카메라를
따라가지 못해 Strong 재획득이 어려워진다는 점이다.

## Tracking tier (OnlineSLAM)

| Tier | 조건 (요약) | Integrate | Pose 갱신 |
|------|-------------|-----------|-----------|
| **Strong** | fitness ≥ Min fitness, trans < 0.12 m, rot < 8° | O (relocalizing 중 X) | model odometry |
| **Weak** | fitness ≥ 0.08, trans < 0.30 m | X | f2f 성공 시만 |
| **Outlier** | trans ≥ 0.50 m 또는 rot ≥ 25° | X | relocalizing 중 f2f(엄격)만 |
| **Fail** | 그 외 | X | f2f 성공 시만 |

공유 헬퍼: [`SlamTrackingRecovery.h`](../examples/cpp/SlamTrackingRecovery.h)

## 복구 방법론 (3단)

### 1) 즉시 조작 (사용자)

경고가 연속될 때:

1. **천천히** 이미 스캔한 영역(벽·가구 등 구조/텍스처)으로 되돌아간다.
2. Info 탭에서 `Relocalizing (n/5 strong frames)` 진행을 확인한다.
3. 급격히 흔들리는 중이면 **Pause** → 안정 후 **Resume**.
4. stuck 시 슬라이더: `Depth diff` ↑ (0.07→0.10), `Min fitness` ↓ (0.25→0.20).
5. 복구 후 겹친 벽이 보이면 **Clean ghosts** + 해당 구역 재스캔.

### 2) 자동 pose 브릿지 (코드)

model tracking 이 Strong 이 아닐 때:

- **Fail / Weak**: frame-to-frame RGBD odometry(f2f)로 pose 만 갱신 (integrate 없음).
- **Outlier**: 평소 f2f 생략. **Relocalizing 모드**에서만 f2f 허용 — trans < 0.12 m,
  rot < 8° 등 Strong 수준의 엄격한 상한.
- f2f 성공: `LogDebug`. tier 가 Outlier/Fail 이면 **기존 LogWarning 유지**.

Relocalization 게이트 (고스트 방지, 변경 없음):

- 연속 3프레임 비Strong → Relocalizing 진입, integration 일시 중단.
- 연속 5프레임 Strong → `Relocalized at frame N. Resuming integration.`

### 3) 수동 복구 GUI

Settings 패널 **Recover tracking** 버튼:

1. `T_frame_to_model` → 마지막 Strong 포즈(`last_strong_pose_`)로 롤백.
2. Relocalizing 모드 진입, integration 중단 유지.
3. 이후 1) 또는 2)와 동일하게 Strong 5연속 후 통합 재개.

Info 탭 추가 표시:

- `Tracking: tier | fitness | trans | rot`
- `F2F pose bridges: N`
- Outlier/Fail 시 복구 안내 문구

## 구현 파일

| 파일 | 역할 |
|------|------|
| `SlamTrackingRecovery.h` | tier 분류, f2f 수용 조건, motion 측정 |
| `OnlineSLAMUtil.h` | f2f 브릿지, Recover 버튼, Info 안내 |
| `RealTimeSLAMRealSense.cpp` | 공유 헤더 사용 (동작 동일) |

## 빌드

```powershell
cmake --build d:\study\Open3D\build --config Release --target OnlineSLAMRGBD OnlineSLAMRealSense RealTimeSLAMRealSense
```

2026-07-07 Release 빌드: 세 타겟 모두 **성공**.

## 수동 검증 시나리오 (권장)

1. **정상 스캔** 5초 — Strong, hash blocks 증가.
2. **급격히 흔들기** 2초 — Outlier 경고 **매 프레임**, integration 정지.
3. **천천히 스캔 영역 복귀** — f2f 로 raycast 추종, Strong 5연속 후 integration 재개.
4. **stuck 시 Recover tracking** — pose 롤백 후 3과 동일.
5. ESC 종료 — ghost wall 없음; 필요 시 Clean ghosts.

## RealTimeSLAMRealSense 와의 관계

RealTime 예제는 이미 f2f 브릿지를 사용한다  
([`Analysis/RealTimeSLAMRealSense-fast-motion-recovery.md`](RealTimeSLAMRealSense-fast-motion-recovery.md)).
OnlineSLAM 은 rotation-aware Strong 판정 + relocalization 게이트(5 Strong)가 추가되어
더 보수적이며, 이번 변경으로 **f2f 브릿지**를 동일하게 갖추었다.

## 한계

- LOST 구간 geometry 는 **구멍**으로 남을 수 있다 (integrate 안 함 — 의도).
- f2f drift 가능 → Strong 복귀 전까지 TSDF 오염은 없음.
- Recover 롤백은 **마지막 Strong 포즈**까지만 (그 이후 카메라 이동은 버려짐).

## 관련 문서

- [`OnlineSLAM-ghost-surface-fix.md`](OnlineSLAM-ghost-surface-fix.md) — relocalization 게이트 원칙
- [`RealTimeSLAMRealSense-fast-motion-recovery.md`](RealTimeSLAMRealSense-fast-motion-recovery.md) — f2f 정책 참고
