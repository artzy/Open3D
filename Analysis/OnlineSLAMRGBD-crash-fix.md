# OnlineSLAMRGBD 크래시 수정 (0xC0000005)

## 증상

`OnlineSLAMRGBD.exe --device CUDA:0` 실행 시:

```
SLAM hash capacity: 0/40000 blocks.
```

로그 이후 **exit code 0xC0000005** (액세스 위반) 또는 GUI 없이 멈춘 것처럼 보임.

## 원인

1. **SLAM 모델을 GUI 메인 스레드에서 생성**하고 Integrate/Track는 **워커 스레드**에서 실행.
2. **`app.Run()` 전에** SLAM 워커가 Filament(OpenGL)와 동시에 동작.
3. **CUDA `raycast_frame`에 CPU 텐서**를 `SetData("color", ...)` — CUDA 사용 시 시작 직후 AV.
4. **Trajectory LineSet** GUI/워커 데이터 레이스.
5. **GUI 미리보기** — CUDA depth/color에 `ColorizeDepth`/`ToLegacy`를 직접 호출 → frame 0 부근 크래시.

**「프로그램 2개」처럼 보이는 경우:** 크래시 시 Windows가 `WerFault.exe` 등 WER 헬퍼를 띄워 작업 관리자에 `OnlineSLAMRGBD.exe`가 2개처럼 보일 수 있음. **정상적인 이중 실행이 아님.** (자동 테스트에서 CUDA/CPU를 동시 실행해도 2개로 보임)

## 수정 (`OnlineSLAMUtil.h`, `OnlineSLAMRGBD.cpp`)

| 항목 | 변경 |
|------|------|
| SLAM 모델 초기화 | `InitSlamModel()` — SLAM 워커 스레드에서 1회 생성 |
| 워커 시작 | `SetOnTickEvent` — GUI 첫 tick 이후 SLAM 스레드 시작 |
| GUI/SLAM 직렬화 | `PostGuiTask` + `WaitForPendingGui()` |
| raycast placeholder | `Zeros(..., device_)` — **CPU:0 사용 금지** |
| Surface 표시 | legacy `PointCloud` + `Open3DScene::AddGeometry` |
| GUI 이미지 | depth/color `To(CPU:0)` 후 ColorizeDepth / ToLegacy |
| Trajectory | GUI 전 `LineSet` 스냅샷 |
| Resume 토글 | SLAM 워커 준비 전 중복 `StartSlam` 방지 |
| 이미지 로드 | `CreateImageFromFile` null 체크 |

## 테스트 (2026-06-16)

```powershell
cmake --build d:\study\Open3D\build --config Release --target OnlineSLAMRGBD
cd d:\study\Open3D\build\bin\examples\Release
.\OnlineSLAMRGBD.exe --device CUDA:0
```

- **단일 인스턴스** 90초: `OnlineSLAMRGBD` 프로세스 1개 유지, `WerFault` 없음 — **PASS**
- CPU 30초 — **PASS**

## 빌드 (필수)

수정 반영 후 **반드시 재빌드**:

```powershell
cmake --build d:\study\Open3D\build --config Release --target OnlineSLAMRGBD
```

## 실행

```powershell
cd d:\study\Open3D\build\bin\examples\Release
.\OnlineSLAMRGBD.exe --device CUDA:0
```

- GUI 창이 뜨고 Input images / 3D view가 갱신되면 정상.
- Lounge 3000프레임은 수 분 소요.
- 종료 시 `scene.ply`, `trajectory.log` 저장.

`OnlineSLAMRealSense`도 동일 헤더를 사용하므로 함께 재빌드 권장.
