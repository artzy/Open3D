# RealTimeSLAMRealSense 중복 스캔 원인 및 수정

## 현상

스캔 중 하나의 물체가 서로 떨어진 두 위치에 누적되어 보였다. 이는 같은 표면이 잘못된 카메라 pose로 TSDF에 다시 integration될 때 발생한다.

## 원인

`RealTimeSLAMRealSense.cpp`의 tracking 실패 처리에서 frame-to-model tracking이 weak/fail인 경우에도 frame-to-frame odometry 결과를 `T_frame_to_model`에 누적했다.

이 방식은 순간적인 tracking 실패 동안 pose를 계속 움직일 수 있지만, feature가 부족하거나 겹침이 낮은 장면에서는 frame-to-frame drift가 누적된다. 이후 drift된 pose가 다시 strong tracking으로 판정되면, 같은 물체가 새로운 위치에 integration되어 중복 스캔처럼 보인다.

기존 strong 판정도 translation과 fitness만 보았고, 회전량은 확인하지 않았다. 따라서 큰 회전 오차가 있는 pose가 strong으로 통과할 여지가 있었다.

## 수정

- 기존처럼 모든 tracking 실패마다 frame-to-frame odometry를 바로 integration 후보로 쓰지 않도록 했다.
- tracking 판정에 rotation angle 검사를 추가했다.
- strong 판정 기준을 더 보수적으로 조정했다.
  - fitness: `0.12 -> 0.15`
  - translation: `0.15m -> 0.12m`
  - rotation: `10 deg` 제한 추가
- tracking lost 이후 integration 재개 조건을 강화했다.
  - strong streak: `2 -> 5`
  - recovery fitness: `0.20` 이상
  - recovery translation: `0.06m` 미만
  - recovery rotation: `5 deg` 미만
- recovery 후보 pose는 조건을 만족하기 전까지 실제 model pose에 반영하지 않는다.
- tracking lost가 길어지면 오래된 raycast pose에 고착되지 않도록 frame-to-frame bridge를 복구 전용으로 다시 사용한다.
  - bridge는 pose와 raycast 기준만 현재 카메라 근처로 따라가게 한다.
  - bridge 중에는 TSDF integration을 하지 않는다.
  - frame-to-model tracking이 다시 안정 조건을 만족해야 integration을 재개한다.
- lost 상태에서는 매 프레임 model tracking을 시도하지 않고 10프레임마다 재시도한다. 반복적인 singular solve 로그와 CPU/GPU 낭비를 줄인다.
- tracking lost가 감지되면 마지막 안정 integration 카메라 pose를 주황색 카메라 프러스텀(`lost_camera`)으로 표시한다.
  - 사용자는 이 주황색 카메라 위치/방향 근처로 돌아가면 재획득을 쉽게 시작할 수 있다.
  - 안정 tracking이 복구되고 integration이 재개되면 marker를 자동으로 숨긴다.
- tracking lost 중 GUI 좌측 패널에 마지막 안정 pose와 현재 추정 pose의 차이를 표시한다.
  - `dX`, `dY`, `dZ`: 마지막 안정 pose 기준 위치 차이(m)
  - `dist`: 위치 차이 크기(m)
  - `rot`: 전체 회전 오차(deg)
  - `rX`, `rY`, `rZ`: 축별 회전 오차(deg)
- hash full 상태에서는 기존 요구대로 pose tracking은 계속하고 integration만 중단한다.

## 검증

- Release x64 빌드 성공.
- `--help` 실행 정상.
- IDE linter 진단: 오류 없음.

## 참고

이번 수정은 잘못된 pose가 TSDF에 들어가는 것을 막기 위한 보수적 조치다. 스캔 중 빠른 움직임이나 낮은 overlap이 있으면 integration이 더 자주 멈출 수 있다. lost 상태에서는 `RELOCALIZING` 상태와 `f2f` streak이 표시되며, 뷰어의 주황색 카메라 프러스텀 근처로 천천히 돌아와 안정적인 frame-to-model tracking을 여러 프레임 유지하면 integration이 재개된다.
