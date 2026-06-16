# Open3D CUDA 빌드 기록 (2026-06-15)

## 환경

| 항목 | 값 |
|------|-----|
| CUDA Toolkit | 12.6.85 (winget `Nvidia.CUDA` 12.6) |
| GPU | NVIDIA GeForce RTX 3060 (compute capability 8.6) |
| 드라이버 지원 CUDA | 13.1 |
| MSVC | 19.44.35227.0 (VS 2022) |
| CMake | 4.1.0-rc4 |
| Python | 3.11.9 |

## CMake 옵션

```powershell
cmake -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_BUILD_TYPE=Release `
  -DBUILD_CUDA_MODULE=ON `
  -DCUDAToolkit_ROOT="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6" `
  -DCMAKE_CUDA_COMPILER="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/bin/nvcc.exe" `
  -DCMAKE_CUDA_ARCHITECTURES=86 `
  -DPython3_ROOT="C:/Users/PM/AppData/Local/Programs/Python/Python311"
```

- `CMAKE_CUDA_ARCHITECTURES=86`: CMake GPU 자동 감지 실패 시 RTX 3060용 명시
- 기존 CPU 빌드 설정 유지: `BUILD_SHARED_LIBS=OFF`, `BUILD_PYTHON_MODULE=OFF`, `BUILD_GUI=ON`

## 빌드

```powershell
$cuda = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6"
$env:CUDA_PATH = $cuda
$env:CUDA_PATH_V12_6 = $cuda
$env:Path = "$cuda\bin;$env:Path"
cd d:\study\Open3D\build
cmake --build . --parallel --config Release
```

## 산출물

- `build/lib/Release/Open3D.lib` — CUDA 포함 정적 라이브러리 (~474 MB)
- `build/Open3D/Release/Open3DViewer.exe` — GUI 뷰어
- `build/bin/examples/Release/OnlineSLAMRealSense.exe` — CUDA 예제

## 이슈 및 해결

1. **CUDA Toolkit 미설치** → winget으로 CUDA 12.6 설치
2. **MSBuild `CudaToolkitDir` 비어 있음** → `CUDA_PATH` 환경 변수 및 `CUDAToolkit_ROOT` 명시
3. **`CUDA_ARCHITECTURES=native` GPU 미감지** → `CMAKE_CUDA_ARCHITECTURES=86` 설정
4. **LNK1104 OnlineSLAMRealSense.exe** → 실행 중 프로세스 종료 후 재빌드

## 재빌드 시 참고

CUDA 경로를 PATH에 넣은 뒤 configure/build:

```powershell
$cuda = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6"
$env:CUDA_PATH = $cuda
$env:CUDA_PATH_V12_6 = $cuda
$env:Path = "$cuda\bin;$env:Path"
```

Python 모듈이 필요하면 `-DBUILD_PYTHON_MODULE=ON` 추가 후 `install-pip-package` 타겟 빌드.
