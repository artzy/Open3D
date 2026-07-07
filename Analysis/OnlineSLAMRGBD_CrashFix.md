# OnlineSLAMRGBD 크래시 (0xC0000005) 분석 및 수정

## 증상

`OnlineSLAMRGBD.exe --device CUDA:0 --default_dataset lounge` 실행 시
SLAM 초기화 로그 직후 ~5–18초 만에 **0xC0000005 (Access Violation)** 종료.

CMake 빌드 exe에서도 동일 (SLAM.sln 배포 문제 아님).

## 원인

1. **파일 재생 속도**: lounge(3000프레임)를 디스크 I/O 한도까지 연속 처리 → 짧은 시간에 다수의 `ExtractPointCloud` + GUI 갱신.
2. **대용량 표면 추출**: TSDF에서 **60만~90만 점** 추출 후 Filament `AddGeometry` / `UpdateGeometry` 부담.
3. **auto_freeze + DBSCAN**: 기본 `auto_freeze=1`로 추출마다 **DBSCAN** 실행 → 대형 포인트클라우드에서 불안정.
4. **Filament 제약**: `UpdateGeometry`는 기존 버텍스 수 **이하**만 갱신 가능. 점 수 증가 시 `RemoveGeometry`+`AddGeometry` 필요 (별도 수정).

## 수정 (`OnlineSLAMUtil.h`, `OnlineSLAMRGBD.cpp`)

| 변경 | 내용 |
|------|------|
| 파일 재생 기본값 | `exit_on_empty_frame_` 시 `auto_freeze=0`, `gui_update_interval=3`, `update_interval=100` |
| 프레임 페이싱 | 파일/bag 모드에서 프레임당 ~33ms sleep (~30fps) |
| 표면 다운샘플 | 추출 후 `surface_.pcd`를 최대 **150k** 점으로 축소 후 GUI 반영 |
| 세그mentation 상한 | `auto_freeze` 사용 시 DBSCAN 입력 **200k** 점 상한 |
| Filament 갱신 | 점 수 **증가** 시 항상 geometry 재생성 |

## 검증

```powershell
cd d:\study\Open3D\SLAM\bin\Release
.\OnlineSLAMRGBD.exe --device CUDA:0 --default_dataset lounge
```

- **60초 이상** 안정 실행 확인 (2026-07-07).

## 참고

- UI에서 **Auto freeze**를 다시 켤 수 있음 (실시간 RealSense에는 유용).
- `'Point cloud size larger than estimated'` 경고는 `estimated_points` 슬라이더를 올리면 완화 (Extract 버퍼 크기).
