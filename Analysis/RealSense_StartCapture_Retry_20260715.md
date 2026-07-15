# RealSense StartCapture 재시도 수정 — 2026-07-15

## 원인

`StartCapture()`가 HRESULT `0x800703e3`(I/O 취소)로 실패하면
`LogError` 예외 → 프로세스 exit 1. 직전 프로세스 해제/USB 재협상 직후에 흔함.

## 수정

| 파일 | 내용 |
|------|------|
| `cpp/open3d/t/io/sensor/realsense/RealSenseSensor.cpp` | 실패 시 `LogWarning` + `return false` (예외로 즉시 중단하지 않음) |
| `SLAM/cpp/RealTimeSLAMRealSense.cpp` | sensor 재생성 + 최대 5회 / 1.5초 간격 재시도 |

## 기대 로그

```
Retrying RealSense open in 1500 ms (attempt 2/5)...
RealSense capture started on attempt 2/5.
```

또는 1회에 성공하면 재시도 로그 없이 기존처럼 `Capture started...`.
