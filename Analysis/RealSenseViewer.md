# RealSenseViewer C++ 예제

## 개요

`examples/cpp/RealSenseViewer.cpp`는 RealSense RGB-D **단순 시각화** 전용 예제입니다. SLAM·녹화·재구성 없이 라이브/ bag 영상만 표시합니다.

Python `examples/python/io/realsense_io.py`의 C++ 대응 예제입니다.

## 모드

| `--mode` | 설명 |
|----------|------|
| `pointcloud` (기본) | 컬러 3D 포인트클라우드 1창 |
| `rgbd` | 컬러·깊이 2D 이미지 2창 |

## 실행

```powershell
cd d:\study\Open3D\build\bin\examples\Release

# D415 라이브 포인트클라우드
.\RealSenseViewer.exe -c d:\study\Open3D\examples\test_data\rs_d415_slam.json

# 2D 컬러/깊이
.\RealSenseViewer.exe --mode rgbd

# bag 재생
.\RealSenseViewer.exe --input capture.bag

# 샘플 bag
.\RealSenseViewer.exe --default_dataset jack_jack
```

**조작:** ESC 종료, bag 재생 시 SPACE 일시정지/재개

## 예제 비교

| 예제 | SLAM | 녹화 | 시각화 |
|------|------|------|--------|
| **RealSenseViewer** | ✗ | ✗ | ✓ |
| `RealSenseRecorder` | ✗ | ✓ | 2D만 |
| `RealTimeSLAMRealSense` | ✓ | ✓ | 3D+SLAM |
