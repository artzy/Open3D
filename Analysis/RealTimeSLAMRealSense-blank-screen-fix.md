# RealTimeSLAMRealSense 빈 화면 문제

**날짜:** 2026-06-16

## 증상

`RealTimeSLAMRealSense.exe` 실행 시 3D 창이 뜨지만 포인트가 보이지 않거나, exe 더블클릭 시 아무 창도 안 뜸.

## 원인

### 1. 초기 ExtractPointCloud가 0포인트

- frame 0에서 `ExtractPointCloud(3.0, ...)` 호출
- TSDF voxel weight는 integrate 1회당 ~1 → threshold 3.0 미만이라 **표면 0개**
- 로그: `The number of points is 0 when creating axis-aligned bounding box.`
- Visualizer가 **빈 PointCloud**로 `AddGeometry` → bbox/카메라 무효 → 회색 빈 화면

### 2. 인자 없이 실행

- `argc <= 1`이면 도움말만 출력하고 **Visualizer 창을 만들지 않음**
- Explorer에서 exe 더블클릭 시 콘솔만 잠깐 보이고 종료

## 수정 (`RealTimeSLAMRealSense.cpp`)

1. **`ExtractWeightThreshold(frame_id)`** — `max(1.0, min(frame_id, 3.0))`로 초기 프레임도 extract 가능
2. **`ShouldRefreshDisplay`** — frame 0, 3, 이후 `update_interval`마다 갱신 (OnlineSLAM과 동일 패턴)
3. **빈 point cloud는 GUI에 전달하지 않음** (`!pcd->IsEmpty()`)
4. **첫 유효 geometry 추가 시 `ResetViewPoint(true)`**
5. **인자 없으면 기본 설정으로 SLAM 시작** (`--help`만 도움말)

## 실행 방법

```powershell
cd d:\study\Open3D
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe
# 또는
.\build\bin\examples\Release\RealTimeSLAMRealSense.exe --device CUDA:0 --profile medium
```

작업 디렉터리는 `examples/test_data/rs_d415_slam.json`을 찾을 수 있는 `d:\study\Open3D` 권장.

## 검증

CPU 프로필 low 8초 실행 후 frame 0에서 **empty bbox 경고 사라짐** 확인.
