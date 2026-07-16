# iPhone 14 Pro RealTimeSLAM 연동 구현 보고 (2026-07-16)

## 목표

RealSense 기반 `RealTimeSLAMRealSense`는 그대로 두고, iPhone 14 Pro(LiDAR + ARKit VIO)를 USB로 받아 Windows CUDA SLAM을 돌리는 **별도 실행 파일/프로젝트**를 만든다.

## 구현 요약

| 항목 | 상태 | 경로 |
|------|------|------|
| 원본 보존 | 완료 | `SLAM/cpp/RealTimeSLAMRealSense.cpp` 미수정 |
| 신규 소스 | 완료 | `SLAM/cpp/IPhoneSLAMRealSense.cpp` |
| Record3D 어댑터 | 완료 | `SLAM/cpp/IPhoneCapture.h` |
| 신규 VS 프로젝트 | 완료 | `SLAM/IPhoneSLAMRealSense.vcxproj` (+ filters), `slam.sln` 등록 |
| record3d 3rdparty | 완료 | `3rdparty/record3d` 클론, `/MT`로 `record3d_cpp` Release 빌드 |
| 빌드 | 완료 | `SLAM/bin/Release/IPhoneSLAMRealSense.exe` |

## 아키텍처

- iPhone: Record3D 앱 → USB(usbmuxd)로 RGB + float32 깊이(m) + ARKit 포즈
- PC: `IPhoneCapture` → RGB를 깊이 해상도로 다운스케일, float(m)→uint16(mm), `depth_scale=1000`
- ARKit(-Z) → OpenCV(+Z): `diag(1,-1,-1)` 변환
- `--pose_source`:
  - `fused`(기본): ARKit 프레임 델타를 odometry prior로 주입, 로스트 시 ARKit 절대 포즈 리시드
  - `arkit`: ARKit 포즈만 신뢰(odometry 스킵)
  - `odom`: ARKit 무시(기존 RGB-D 트래킹)

## 실행 방법

1. **Apple Mobile Device Support** 또는 iTunes 설치 (usbmuxd 소켓).
2. iPhone USB 연결 → 잠금 해제 → 이 컴퓨터를 신뢰.
3. App Store에서 **Record3D** 설치 → USB Streaming 활성화 → Record 시작.
4. PC에서:

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\IPhoneSLAMRealSense.exe --device CUDA:0 --profile iphone --pose_source fused
# 또는 ARKit만:
.\IPhoneSLAMRealSense.exe --pose_source arkit --regions
```

record3d 재빌드(Open3D `/MT`와 맞춤):

```powershell
cd d:\study\Open3D\SLAM
.\build_record3d.ps1
```

## 검증 결과 (2026-07-16)

### 빌드/CLI

- `--help` 정상 출력 (iPhone/pose_source 옵션 확인).
- `--pose_source bad` → 에러로 거부.
- Release 링크 성공 (`IPhoneSLAMRealSense.exe`).

### 라이브 실행 테스트 (약 30초)

**1차 (버그):** float32 depth + uint8 color → Integrate 거부  
`Unsupported input data type combination. Expected (float, float) or (uint16, uint8)`

**수정:** Record3D float(m) → uint16(mm), `depth_scale=1000` (`IPhoneCapture.h`)

**2차 (성공):** CUDA:0, `--profile iphone --pose_source fused`

| 항목 | 결과 |
|------|------|
| USB 연결 | Connected to Record3D USB stream |
| 해상도/내참 | 192x256, fx=fy≈180.9, depth_scale=1000 |
| ARKit | pose calibration locked |
| Integrate dtype 오류 | 0 |
| SLAM frame 로그 | 1674 |
| tier strong | 1673 |
| Tracking lost | 0 |
| ARKit reseed | 0 |
| hash 성장 | frame0: 0 → frame1: 1043 → frame54: ~2960 (계속 증가) |

로그: `Analysis/iphone_run2_out.txt`, `Analysis/iphone_run2_err.txt`

### 2분 SLAM 라이브 테스트 (2026-07-16 13:37)

명령: `IPhoneSLAMRealSense.exe --device CUDA:0 --profile iphone --pose_source fused --regions`  
로그: `Analysis/iphone_slam_2min_20260716_133718.out.txt` / `.err.txt`

| 항목 | 결과 |
|------|------|
| 연결 | iPhone udid 인식, Record3D USB Connected, ARKit calib OK |
| 입력 | 192x256, depth_scale=1000 |
| 프레임 로그 | 2425 |
| tier strong / weak / fail | 1795 / 599 / 24 |
| Tracking lost | 6 |
| restabilized | 7 |
| ARKit reseed | 38 |
| keyframes | 18 |
| max hash blocks | 18363 / 40000 |
| dtype 오류 | 0 |
| Region | mesh extract 진행(후보 0/2/3/5, 최대 ~346k verts). 강제 종료로 저장 미완료 |

강제 Kill로 GUI 종료 저장(`scene.ply`)은 이번 런에 새로 생성되지 않음. 트래킹·맵 통합은 정상 동작 확인.

## 리스크 / 후속

- Record3D USB Streaming은 유료 인앱일 수 있음.
- LiDAR 깊이 해상도(~192x256)로 region mesh 품질이 D415보다 낮을 수 있음.
- JPEGDecoder stb / Open3D GLTF 심볼 중복(LNK4006) — `/FORCE:multiple`로 링크됨.
- `Apple Mobile Device Service`가 Stopped여도 USB 장치 인식이 되면 동작함(이번 테스트). 서비스 기동은 관리자 권한 필요.
- Debug 구성 빌드 시 record3d도 `MultiThreadedDebug` 재빌드 필요.

## 변경 파일 목록

- `SLAM/cpp/IPhoneSLAMRealSense.cpp` (신규)
- `SLAM/cpp/IPhoneCapture.h` (신규)
- `SLAM/IPhoneSLAMRealSense.vcxproj` (신규)
- `SLAM/IPhoneSLAMRealSense.vcxproj.filters` (신규)
- `SLAM/slam.sln` (프로젝트 추가)
- `SLAM/build_record3d.ps1` (신규)
- `3rdparty/record3d/` (클론 + 빌드 산출물)
