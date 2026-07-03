# OnlineSLAM 고스트 표면(벽 중복 캡처) 방지 및 삭제

같은 벽/공간이 2~3겹으로 캡처되는 "고스트 표면" 현상의 원인과 수정 내용.
대상: `OnlineSLAMRGBD`, `OnlineSLAMRealSense` (공통 헤더
`examples/cpp/OnlineSLAMUtil.h`).

## 증상

- 스캔 중 카메라를 빠르게 돌리거나 텍스처 없는 벽에 가까이 가면 추적(odometry)이
  어긋난 채 TSDF 통합이 계속되어 같은 벽이 offset 된 위치에 여러 겹 생김.
- 한 번 생긴 고스트는 기존 코드에서는 지울 방법이 없었음 (Integrate 커널이
  free-space 관측을 버리기 때문).

## 원인 (코드 확인)

1. **Weak 티어(fitness 0.08~0.15) 프레임을 "이전 포즈"로 통합** — 카메라는
   움직였는데 옛 위치에 깊이가 쌓여 벽이 겹침 (가장 큰 원인).
2. **추적 손실 후 복구 게이트 없음** — 실패 동안 실제 이동량이 누적된 채,
   fitness가 한 번만 통과하면 어긋난 포즈로 즉시 통합 재개.
3. **회전량 미검사** — translation만 검사하고 회전 점프는 통과.
4. `kPoseFitnessMin = 0.15`로 관대함.
5. Integrate 커널은 `sdf < -sdf_trunc`(확실한 빈 공간 관측)를 그냥 버림 —
   "여기에 벽이 없다"는 증거가 반영되지 않아 고스트가 영구히 남음.

## 방지 수정 (`OnlineSLAMUtil.h`)

| 항목 | 변경 |
|------|------|
| Weak 티어 | 포즈/통합 모두 스킵 (기존: 이전 포즈로 통합) |
| 회전량 검사 | Strong: < 8 deg/frame, Outlier: >= 25 deg/frame |
| fitness 기준 | 0.15 → 0.25 (GUI "Min fitness" 슬라이더로 조정 가능) |
| 재정렬 게이트 | 연속 3프레임 비Strong → Relocalizing 모드 진입. 연속 5프레임 Strong일 때만 통합 재개. 재정렬 중 Strong 프레임은 포즈만 갱신(레이캐스트 추종), 통합은 중지 |
| GUI 안내 | Info 탭에 "Relocalizing: move slowly back to the scanned area (n/5 strong frames)" 표시 |

핵심 원칙: **불확실한 포즈로는 절대 통합하지 않는다.** 프레임 몇 장을
건너뛰는 것이 겹친 벽보다 낫다.

## 삭제 수정 (이미 생긴 고스트 제거)

### 1. Free-space carving (자동)

`CarveFreeSpace()` — Strong 추적 프레임에서 주기적으로(CUDA 30프레임,
CPU 90프레임마다) 실행:

1. 활성 블록 중심을 카메라에 투영해 시야 내 블록만 선별 (블록 프리필터)
2. 해당 블록의 복셀 중심을 depth 이미지에 투영
3. 관측 깊이보다 `3 x sdf_trunc`(~0.14 m) 이상 **앞**(확실한 빈 공간)에 있고
   weight > 0인 복셀의 weight를 매회 절반으로 감쇠
4. weight가 추출/레이캐스트 임계값(3.0) 밑으로 내려가면 화면과 저장 결과에서
   자동으로 사라짐

→ **올바른 위치에서 그 공간을 다시 비추면 고스트 벽이 수 초 내 소거됨.**
텐서 연산만 사용 (CPU+CUDA 동작, 코어 라이브러리 수정 없음). 마진을 sdf_trunc의
3배로 보수적으로 잡아 얇은 실제 구조물(문틀 등)은 깎지 않음. 실제 표면은
carving 사이에 매 프레임 +1 weight로 재통합되므로 임계값 위에서 평형 유지.

측정: 1패스 60~130 ms (CUDA, medium 프로파일, Lounge) → 30프레임 간격으로 분산.

### 2. GUI "Clean ghosts" 버튼 (수동)

`CleanLowWeightVoxels()` — 버튼 클릭 시:

1. weight < "Min weight" 슬라이더 값(기본 3)인 복셀의 tsdf/weight를 0으로 리셋
2. 완전히 비게 된 블록은 `HashMap::Erase`로 제거 → **해시 용량 회수**
   (hash_near_full로 통합이 멈추는 문제도 완화)
3. surface 재추출 트리거

블록은 zero 후 Erase되므로 해시 슬롯이 재사용돼도 이전 데이터 오염 없음
(weight 0이면 다음 Integrate가 처음부터 기록).
8192블록 단위 청크 처리로 임시 메모리 제한.

### 3. 저장 시 weight 필터 연동

종료 시 `scene.ply` 추출의 weight_threshold가 "Min weight" 슬라이더 값과 연동
(기존 고정 3.0) → 저신뢰 잔여물을 최종 산출물에서 배제.

## 테스트 (2026-07-02)

```powershell
cmake --build d:\study\Open3D\build --config Release --target OnlineSLAMRGBD OnlineSLAMRealSense
cd d:\study\Open3D\build\bin\examples\Release
.\OnlineSLAMRGBD.exe --device CUDA:0
```

| 테스트 | 결과 |
|--------|------|
| CUDA 120초 Lounge 회귀 (프로세스 1개 유지) | PASS — 추적 실패/재정렬 진입 0회, fitness ~0.8 유지 |
| CUDA 정상 시퀀스에서 과도한 통합 스킵 없음 | PASS — Weak/Fail 경고 없음 |
| Free-space carving 동작 | PASS — 패스당 20만~170만 복셀 감쇠, 60~130 ms |
| CPU 45초 sanity | 로그 확인 (터미널 503681) |
| scene.ply / trajectory.log 저장 | PASS (첫 실행에서 39 MB / 38 KB 생성 확인) |

### 수동 테스트 (D415 실기 필요)

1. **재정렬 게이트**: 카메라를 빠르게 돌려 추적을 잃은 뒤 Info 탭에
   "Relocalizing (n/5 strong frames)" 표시 확인 → 스캔한 영역으로 천천히
   복귀 → "Relocalized" 로그 후 통합 재개, 벽 중복 없어야 함.
2. **Carving**: 일부러 고스트를 만든 뒤 올바른 위치에서 그 공간을 다시 비추면
   수 초 내 고스트가 사라지는지 확인.
3. **Clean ghosts 버튼**: 클릭 시 저weight 잔여물이 즉시 사라지고
   "erased N empty blocks" 로그 확인.

## 촬영 가이드 (코드 외 대책)

- 회전은 천천히 (특히 벽만 보이는 구간)
- 텍스처/기하가 풍부한 영역을 시야에 유지, 평평한 벽 단독 촬영 회피
- `--perf quality` (odom 6/3/1)로 빠른 회전에 대한 추적 강건성 향상
- `depth_max`는 실내 3 m 유지 (원거리 노이즈 배제)

## 조정 가능한 상수 (`OnlineSLAMUtil.h` 상단)

| 상수 | 기본값 | 의미 |
|------|--------|------|
| `kPoseFitnessMin` | 0.25 | Min fitness 슬라이더 기본값 |
| `kPoseRotationMaxDeg` | 8.0 | Strong 허용 회전량 (deg/frame) |
| `kOutlierRotationDeg` | 25.0 | Outlier 판정 회전량 |
| `kRelocalizeAfterFailures` | 3 | 재정렬 진입 실패 프레임 수 |
| `kRelocalizeStrongFrames` | 5 | 통합 재개에 필요한 연속 Strong 수 |
| `kCarveIntervalCuda/Cpu` | 30 / 90 | carving 실행 간격 (프레임) |
| `kCarveMarginTruncFactor` | 3.0 | carving 마진 (x sdf_trunc) |
