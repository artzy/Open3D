# iPhone Wi-Fi SLAM 연결 시도 (2026-07-16)

## 현재 PC–iPhone 연결 상태

| 항목 | 값 |
|------|-----|
| 어댑터 | `이더넷 2` = **Apple Mobile Device Ethernet** |
| PC IP | 172.20.10.8 |
| 게이트웨이 | 172.20.10.1 (아이폰) |
| PnP | Apple iPhone / Apple Mobile Device USB = OK |

→ 지금은 **Wi-Fi가 아니라 USB 케이블(테더 포함)** 연결입니다.

## Record3D Wi-Fi Streaming (WebRTC) 스캔

- LAN `192.168.0.0/24` 에서 `http://IP/metadata` 스캔 → **0건**
- 핫스팟 `172.20.10.1` `/metadata`, `/getOffer` → **타임아웃**

→ Record3D 앱에서 **Wi-Fi Streaming**이 켜져 있지 않거나, 유료 Extension / 같은 Wi-Fi 미연결.

## 기술 제약 (현재 `IPhoneSLAMRealSense`)

1. 공식 `record3d` C++ 라이브러리는 **USB(usbmuxd) 전용**. WebRTC Wi-Fi 수신은 미구현.
2. Record3D 공식 Wi-Fi Streaming은 **ARKit 포즈를 보내지 않음** (lossy HSV depth만). 트래킹 로스트 개선 목적과 충돌.
3. Wi-Fi에서도 포즈를 유지하려면:
   - Record3D: **USB Streaming** 유지
   - iTunes: **Wi-Fi로 이 iPhone과 연결** 활성화
   - USB 케이블 분리 → 무선 usbmuxd 터널

## 준비되면 실행할 테스트

스크립트: [`SLAM/test_iphone_wifi_usbmuxd.ps1`](../SLAM/test_iphone_wifi_usbmuxd.ps1)

```powershell
cd d:\study\Open3D\SLAM
# 1) iTunes에서 Wi-Fi 연결 체크
# 2) USB 케이블 뽑기
# 3) Record3D USB Streaming + Record
.\test_iphone_wifi_usbmuxd.ps1 -Seconds 90 -PoseSource fused
```

케이블을 뽑으면 스크립트가 USB 소멸을 감지한 뒤 SLAM을 돌립니다.

## 후속 (선택)

Record3D **Wi-Fi Streaming(WebRTC)** 을 SLAM에 붙이려면 WebRTC 수신 + HSV depth 복원 어댑터가 새로 필요하며, 포즈 없이 `--pose_source odom`만 가능합니다.
