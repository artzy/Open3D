# SLAM Global Relocalization (방안 C)

## 개요

`RealTimeSLAMRealSense`에 keyframe DB 기반 global relocalization을 추가했다. tracking lost 이후 FPFH+RANSAC/FGR global registration과 MultiScaleICP 정제로 pose를 복구하고, multi-hypothesis tracking으로 model tracking seed를 선택한다.

## 변경 파일

| 파일 | 내용 |
|------|------|
| `SLAM/cpp/Relocalization.h` | KeyframeDatabase, GlobalRelocalizer, MultiHypothesisTracker, depth histogram |
| `SLAM/cpp/RealTimeSLAMRealSense.cpp` | SLAM 루프 통합, CLI, recovery gate |
| `SLAM/cpp/RealTimeSLAMUtil.h` | reloc status, keyframe marker toggle |
| `SLAM/RealTimeSLAMRealSense.vcxproj` | header 등록 |

## 아키텍처

1. **Keyframe capture**: integrate 성공 시 `MaybeAddKeyframe()` — RGBD → downsampled world point cloud + legacy FPFH + depth histogram
2. **Lost detection**: 기존 `consecutive_tracking_failures > 5` 유지
3. **Global reloc** (15프레임마다, lost 중): spatial + histogram 후보 선별 → RANSAC/FGR → ICP → verify gate
4. **Multi-hypothesis**: lost 진입 시 `last_stable`, `current`, global 성공 시 `global` seed 유지 → model retry 프레임에서 best fitness 선택
5. **Recovery**: global 성공 후 `verify_strong_streak=3` strong 연속 시 integration 재개

## CLI 옵션

```
--global_reloc 0|1              (default: 1)
--keyframe_interval 30
--keyframe_max 48
--reloc_method ransac|fgr
--reloc_retry_interval 15
--reloc_candidate_radius 2.5
--reloc_min_fitness 0.25
--reloc_self_test              (자동 lost/global-reloc 테스트 후 종료)
```

## GUI

- **Reloc detail**: keyframe 수, global reloc 결과/거부 사유
- **Keyframe markers**: 청록색 sphere로 keyframe 위치 표시 (토글)

## 검증 게이트 (duplicate scan 방지)

- ICP `EvaluateRegistration` fitness ≥ `min_fitness`
- Information matrix ratio ≥ 0.20
- `last_stable` 대비 translation jump ≤ 1.0 m
- RANSAC identity transform 거부 (`trace == 4.0`)
- integration 재개 전 strong streak 3프레임

## 빌드·실행 테스트 (2026-07-08)

| 항목 | 결과 |
|------|------|
| Release 빌드 | 성공 |
| `-l` RealSense D415 (314522061035) | 감지됨 |
| `--help` global reloc 옵션 | 확인 |
| Live 실행 (~445 frames, `--profile low`) | **성공** (크래시 없음) |
| Keyframe 누적 | #0 frame 0, #1 frame 319, #2 frame 399, #3 frame 417 |
| Tracking tier | 전 구간 `strong` (lost 미발생) |
| Global reloc 시도 | lost 미발생으로 미실행 |
| GUI (Filament OpenGL) | 정상 기동 |

### 테스트 중 발견·수정한 버그

- **증상**: frame 0 integrate 직후 `0xC0000409` 크래시
- **원인**: `ComputeDepthHistogram()`이 Float32 depth tensor에 `GetDataPtr<double>()` 호출
- **수정**: UInt16 / Float32 / Float64 분기 처리 (`Relocalization.h`)

로그: [`Analysis/global_reloc_live_test_out.txt`](global_reloc_live_test_out.txt)

### `--reloc_self_test` 자동 lost 시나리오 (2026-07-08)

| 항목 | 결과 |
|------|------|
| 실행 | `RealTimeSLAMRealSense.exe --reloc_self_test --profile low --global_reloc 1` |
| 모드 | headless (GUI 생략) |
| warmup | 35프레임 후 drift 주입 (0.20 m, 15°) |
| Global reloc | frame 35, keyframe 0, icp fitness 1.000 |
| Recovery | frame 37 `Tracking restabilized`, integration 재개 |
| PASS 조건 | global reloc OK + pose 오차 ≤ 0.15 m/12° + tracking restabilized |
| 결과 | **RELOC_SELF_TEST: PASS**, exit code **0** |

로그: [`Analysis/global_reloc_self_test_out.txt`](global_reloc_self_test_out.txt)

재실행: `SLAM/run_reloc_self_test.ps1`

#### 수정 이력 (self-test/recovery 버그)

| 문제 | 수정 |
|------|------|
| global reloc만 PASS, integration 미재개 | self-test가 recovery 완료까지 대기 |
| `Skipped -1 frames in update T!` | `Model::UpdateFramePose` 동일 frame 재설정 허용 |
| lost 후 tracking 실패 | pose 갱신 직후 `SynthesizeModelFrame` 추가 |
| hypothesis probe 부작용 | probe 후 pose 복원 |

## 수동 테스트 시나리오

1. **정상 스캔 + keyframe 누적**: 30초 이상 천천히 스캔 → 로그 `Keyframe added #N` 확인, GUI `Keyframes X/48`
2. **Lost 유도**: 급격한 pan/tilt → `RELOCALIZING` 상태, keyframe marker 토글 ON
3. **복귀**: 스캔 영역 근처(20~50cm) 복귀 → `Global relocalization accepted` 로그, `GLOBAL RECOVERING` → integration 재개
4. **오매칭 방지**: 다른 방향/미스캔 영역 → `Global relocalization rejected` 만 출력, integrate 미재개
5. **Duplicate 회귀**: 단일 물체 스캔 후 mesh/pcd에 이중 구조 없음 확인

## 알려진 제한

- RANSAC/FGR는 legacy CPU — lost 중 0.5~2s/시도 가능
- tensor RANSAC 미지원 → legacy hybrid 필수
- RealSense live 카메라 없이 GUI end-to-end 테스트는 환경 의존

## 로그 키워드

- `Keyframe added #`
- `Global relocalization accepted`
- `Global relocalization rejected`
- `Hypothesis '...' selected`
- `Tracking restabilized`
