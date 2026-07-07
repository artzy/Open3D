# SLAM 뷰 뒤집힘 수정 (SLAM/cpp)

## 원인

1. **좌표계 불일치**: SLAM/RGB-D 월드는 카메라 관례(+Y 아래). `SceneWidget::SetupCamera`는 내부에서 `GoToCameraPreset(PLUS_Z)`를 호출해 OpenGL 기본(+Y 위) 시점으로 리셋한다.
2. **초기화 플래그 오류**: GUI 초기화 시 placeholder bbox로 `ConfigureSlamCameraView`만 호출하고 `camera_view_initialized_ = true`로 설정해, 실제 포인트 클라우드가 올 때 1회 피팅(LookAt 포함)이 실행되지 않았다.
3. **평면 변환 시 재뒤집힘**: 표면 갱신마다 `SetupCamera`를 다시 호출하면 +Y up으로 돌아가 뷰가 뒤집힌다. (이전 `camera_fitted_` 로직과 동일 이슈)

## 수정 (`SLAM/cpp/OnlineSLAMUtil.h`)

- `ConfigureSlamCameraView`: `SetupCamera` 직후 `LookAt(..., up=(0,-1,0))` 적용
- 시작 placeholder fit은 방향만 맞추고 `camera_view_initialized_`는 설정하지 않음
- `UpdateCameraCenterOfRotation`: 최초 실제 geometry bbox에서 1회만 `ConfigureSlamCameraView`, 이후 `SetCenterOfRotation`만

## 검증

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\OnlineSLAMRGBD.exe --device CUDA:0 --default_dataset lounge
```

- 시작 직후 방이 똑바로 보이는지
- auto-freeze로 포인트→평면 mesh 전환 후에도 뷰가 뒤집히지 않는지 (10~15초)
