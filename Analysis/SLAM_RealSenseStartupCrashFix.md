# RealTimeSLAMRealSense 0xc0000409 시작 오류 수정

## 현상

옵션 없이 `RealTimeSLAMRealSense.exe`를 실행하면 아래 로그 이후 `0xc0000409`로 종료됐다.

```text
No options given — starting live SLAM with default settings ...
Compute device: CUDA:0
SLAM profile: medium ...
```

## 원인

두 가지 문제가 겹쳤다.

1. 기본 config 경로가 `examples/test_data/rs_d415_slam.json` 상대 경로 하나만 확인했다.
   - Visual Studio 실행 작업 디렉터리는 `SLAM/bin/Release`라서 이 경로를 찾지 못했다.
   - config를 못 찾으면 RealSense 기본 설정(`RS2_FORMAT_ANY`, `0,0`, `fps=0`)으로 초기화되어 장치/드라이버 조합에 취약했다.
2. 일반 실행 경로에서 `rs.ListDevices()`를 먼저 호출했다.
   - librealsense 장치 query가 실패하면 native abort로 이어질 수 있어 실제 SLAM 시작 전 종료될 수 있었다.
3. `RealSenseSensor::InitSensor()` 내부는 실패 시 `LogError`를 던진다.
   - 호출부에서 잡지 않아 no device / camera busy / invalid config 상황이 0xc0000409처럼 보였다.

## 수정

- 기본 D415 config 탐색 후보를 추가했다.
  - `examples/test_data/rs_d415_slam.json`
  - `../examples/test_data/rs_d415_slam.json`
  - `../../examples/test_data/rs_d415_slam.json`
  - `../../../examples/test_data/rs_d415_slam.json`
  - `../../../../examples/test_data/rs_d415_slam.json`
- config를 못 찾으면 안전하게 오류 메시지와 함께 `return 1`.
- 일반 실행 경로에서 `rs.ListDevices()` 호출 제거.
- `InitSensor()`와 `StartCapture()`를 `try/catch`로 감싸 장치 없음/점유/설정 오류를 정상 종료로 처리.

## 검증

- Release x64 빌드 성공.
- `SLAM/bin/Release`에서 옵션 없이 실행 시 기본 config를 `../../../examples/test_data/rs_d415_slam.json`로 찾음.
- 현재 테스트 환경에서는 RealSense 장치가 연결되지 않은 상태로 확인됨.
- 이전처럼 `0xc0000409`가 아니라 아래 메시지 후 정상 오류 코드 `1`로 종료됨.

```text
RealSense startup failed: ... No device connected
EXIT:1
```

## 사용 메모

카메라가 연결되어 있는데 같은 메시지가 나오면 Intel RealSense Viewer, 이전 `RealTimeSLAMRealSense.exe`, 다른 캡처 프로그램이 카메라를 점유 중인지 확인한다.
