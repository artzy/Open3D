# SLAM 프로젝트 런타임 리소스 분석

## 분석 대상

- `SLAM/OnlineSLAMRGBD.vcxproj`
- `SLAM/OnlineSLAMRealSense.vcxproj`
- `SLAM/Open3DExample.props`
- `examples/cpp/OnlineSLAMRGBD.cpp`, `OnlineSLAMRealSense.cpp`

## 결론

두 예제 모두 `gui::Application::Initialize()`를 호출하며, Open3D GUI(Filament) 렌더링에 **64개의 GUI 리소스**가 필요합니다.

### 리소스 탐색 순서 (`Application.cpp` → `FindResourcePath`)

1. 환경 변수 `OPEN3D_RESOURCE_PATH`
2. 실행 파일 기준 상대 경로:
   - `{exe_dir}/resources`
   - `{exe_dir}/../resources`
   - `{exe_dir}/../../resources` ← **SLAM 레이아웃에서 사용**
   - `{exe_dir}/share/resources`
   - `{exe_dir}/share/Open3D/resources`

### SLAM 디렉터리 레이아웃

```
exe: SLAM/bin/Release/OnlineSLAMRGBD.exe
     → ../../resources = SLAM/resources  ✓
```

## 필요 리소스 종류

| 종류 | 예시 | 용도 |
|------|------|------|
| Filament material | `ui_blit.filamat`, `defaultLit.filamat` | GUI/3D 렌더링 |
| IBL / Skybox | `default_ibl.ktx`, `park_skybox.ktx` | 조명·배경 |
| 폰트 | `Roboto-Medium.ttf`, `RobotoMono-Medium.ttf` | ImGui UI |
| 텍스처 | `defaultTexture.png`, `defaultGradient.png` | 기본 재질 |
| Gaussian splat | `gaussian_splat/*.comp`, `*.spv` | (옵션) 스플at |
| WebRTC HTML | `html/*` | WebRTC 예제용 (링크됨, SLAM에서는 미사용) |

필수 검증 파일: `ui_blit.filamat` (`Application::Initialize`에서 존재 확인)

## 런타임 DLL (정적 링크 외)

| DLL | 출처 | PostBuild |
|-----|------|-----------|
| `tbb12.dll` / `tbb12_debug.dll` | `build/msvc_*_release/` | vcxproj PostBuild |
| `zlib1.dll` | vcpkg | vcxproj PostBuild |

CUDA (`cudart64_*.dll`)는 CUDA Toolkit PATH 또는 시스템 PATH에 의존 (정적 링크 아님).

## 복사 위치

- **마스터**: `SLAM/resources/` (64 files, `build/bin/resources`와 동기)
- **동기화 스크립트**: `SLAM/copy_resources.ps1`
- **빌드 시 자동 동기화**: vcxproj PostBuild → `robocopy $(BuildRoot)\bin\resources → $(ProjectDir)resources`

## 데이터셋 (별도)

RGB-D 파일·RealSense JSON 설정은 Open3D 리소스가 아님. 실행 시 인자로 지정:

- RGBD: `--dataset_path examples/test_data/RGBD`
- RealSense: `-c examples/test_data/rs_slam_lowmem.json`

## Open3D 재빌드 후

```powershell
python SLAM/generate_props.py   # 링크 설정 변경 시
powershell -File SLAM/copy_resources.ps1   # 리소스만 갱신
# 또는 SLAM 솔루션 빌드 (PostBuild가 자동 동기화)
```
