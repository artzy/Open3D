# iPhone Wi-Fi SLAM 재시도 (2026-07-16 13:52)

## 발견

1. Record3D **Wi-Fi Streaming**이 `http://172.20.10.1` 에서 일시적으로 활성임을 확인.
   - `/metadata` → 200 (K, originalSize 720x960)
   - `/getOffer` → 200 (WebRTC offer)
2. Python `aiortc`로 answer POST `/answer` → 200, **video track 수신**까지 성공.
3. 그러나 `track.recv()`로 **디코딩된 프레임을 받지 못함** (Windows aiortc/PyAV H.264 디코드 이슈 가능).
4. 이후 iPhone 쪽 스트림이 꺼지며 `/metadata` 타임아웃 → **현재 Wi-Fi Streaming 비활성**.

## 현재 링크

- `이더넷 2` = Apple Mobile Device Ethernet → USB 테더 유지.
- C++ `IPhoneSLAMRealSense` USB 경로는 Record3D가 Wi-Fi Streaming 모드일 때 프레임이 안 옴(정상).

## 테스트 산출물

- 스크립트: `SLAM/tools/record3d_wifi_slam_test.py`
- HTML 데모 서버: `python -m http.server 8765` in `3rdparty/record3d-wifi-demo` (브라우저 검증용)
- 이 문서

## 다시 테스트하려면 (아이폰)

1. Record3D → Settings → Live RGBD → **Wi-Fi**
2. Record 탭에서 빨간 버튼 → "**Started, Waiting for Connection**"
3. Device Address에 `172.20.10.1` 또는 Wi-Fi IP 표시 확인
4. PC에서:
   - 브라우저: http://127.0.0.1:8765/ 에 `172.20.10.1` 입력
   - 또는 `python SLAM/tools/record3d_wifi_slam_test.py --host 172.20.10.1 --seconds 40`

## 한계

- Wi-Fi Streaming은 **ARKit 포즈 없음** → odometry만.
- 정확도/지연은 USB Streaming 대비 불리 (공식 권장도 USB).
- SLAM(C++ CUDA)에 붙이려면 WebRTC 수신 어댑터를 `IPhoneCapture`에 추가해야 함.
