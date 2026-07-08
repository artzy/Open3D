# SLAM 뷰 뒤집힘 수정 (SLAM/cpp)

## 원인

1. **좌표계 불일치**: SLAM/RGB-D 월드는 카메라 관례(+Y 아래). `SceneWidget::SetupCamera`는 내부에서 `GoToCameraPreset(PLUS_Z)`를 호출해 OpenGL 기본(+Y 위) 시점으로 리셋한다.
2. **Legacy Visualizer**: `ViewControl::Reset()`도 +Y up / +Z front. `AddGeometry(..., true)`가 자동으로 `ResetViewPoint()`를 호출한다.
3. **평면 변환 시 재뒤집힘**: 표면 갱신마다 `SetupCamera`/`ResetViewPoint`를 다시 호출하면 +Y up으로 돌아간다.

## 수정

### OnlineSLAMUtil.h
- `ConfigureSlamCameraView`: `SetupCamera` 직후 `LookAt(..., up=(0,-1,0))`
- `UpdateCameraCenterOfRotation`: 최초 1회만 카메라 피팅

### RealTimeSLAMRealSense.cpp
- `ApplySlamViewOrientation`: `SetUp(0,-1,0)`, `SetFront(0,0,-1)`
- `ConfigureSlamViewOnce`: 최초 1회만 `ResetViewPoint` + SLAM up
- `AddGeometry(..., false)`로 자동 view reset 방지, 이후 `UpdateGeometry`만 사용

## 검증

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\OnlineSLAMRGBD.exe --device CUDA:0 --default_dataset lounge
```

- 시작 직후 방이 똑바로 보이는지
- auto-freeze로 포인트→평면 mesh 전환 후에도 뷰가 뒤집히지 않는지 (10~15초)
