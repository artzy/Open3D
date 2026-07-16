#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record3D Wi-Fi (WebRTC) RGBD receive + short Open3D TSDF SLAM test.

Record3D Wi-Fi Streaming encodes depth in the left half (HSV hue) and color in
the right half. ARKit pose is NOT available on Wi-Fi — this test uses RGB-D
odometry only (--pose_source odom equivalent).

Usage:
  python SLAM/tools/record3d_wifi_slam_test.py --host 172.20.10.1 --seconds 45
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

import aiohttp
import av
import cv2
import numpy as np
import open3d as o3d
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole


def decode_rgbd_frame(frame_bgr: np.ndarray, depth_max_m: float = 3.0):
    """Split side-by-side RGBD video into color (uint8) and depth (uint16 mm)."""
    h, w = frame_bgr.shape[:2]
    half = w // 2
    depth_bgr = frame_bgr[:, :half]
    color_bgr = frame_bgr[:, half:]
    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

    hsv = cv2.cvtColor(depth_bgr, cv2.COLOR_BGR2HSV)
    # OpenCV H is 0..179; Record3D demo uses Hue in 0..1 after convert.
    hue01 = hsv[:, :, 0].astype(np.float32) / 179.0
    depth_m = (depth_max_m * hue01).astype(np.float32)
    depth_mm = np.clip(np.rint(depth_m * 1000.0), 0, 65535).astype(np.uint16)
    return color_rgb, depth_mm


def parse_intrinsics(meta: dict, color_w: int, color_h: int):
    k = meta["K"]
    # Column-major 3x3: [fx,0,cx, 0,fy,cy, 0,0,1] laid out as
    # [fx,0,0, 0,fy,0, cx,cy,1] in Record3D metadata samples.
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[6]), float(k[7])
    orig = meta.get("originalSize", [color_w, color_h])
    ow, oh = int(orig[0]), int(orig[1])
    if ow > 0 and oh > 0 and (ow != color_w or oh != color_h):
        sx = color_w / float(ow)
        sy = color_h / float(oh)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    intrinsic = o3d.core.Tensor(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=o3d.core.Dtype.Float64,
    )
    return intrinsic, fx, fy, cx, cy


class FrameSink:
    def __init__(self):
        self.lock = asyncio.Lock()
        self.latest = None
        self.count = 0

    async def on_frame(self, img_bgr: np.ndarray):
        async with self.lock:
            self.latest = img_bgr
            self.count += 1

    async def take(self):
        async with self.lock:
            img = self.latest
            self.latest = None
            return img


async def consume_track(track, sink: FrameSink, stop_event: asyncio.Event):
    err_count = 0
    while not stop_event.is_set():
        try:
            frame = await asyncio.wait_for(track.recv(), timeout=3.0)
        except asyncio.TimeoutError:
            continue
        except Exception as e:
            err_count += 1
            if err_count <= 5:
                print(f"[WiFi] track.recv error: {type(e).__name__}: {e}")
            await asyncio.sleep(0.05)
            if err_count > 30:
                break
            continue
        try:
            img = frame.to_ndarray(format="bgr24")
            await sink.on_frame(img)
        except Exception as e:
            err_count += 1
            if err_count <= 5:
                print(f"[WiFi] frame decode error: {type(e).__name__}: {e}")
            await asyncio.sleep(0.01)


async def run_wifi_slam(host: str, seconds: float, depth_max: float, out_dir: Path):
    base = host if host.startswith("http") else f"http://{host}"
    base = base.rstrip("/")

    async with aiohttp.ClientSession() as session:
        async with session.get(f"{base}/metadata", timeout=5) as resp:
            meta = await resp.json()
        print(f"[WiFi] metadata={json.dumps(meta)}")

        async with session.get(f"{base}/getOffer", timeout=5) as resp:
            if resp.status != 200:
                raise RuntimeError(f"getOffer HTTP {resp.status}")
            offer = await resp.json()

        pc = RTCPeerConnection()
        sink = FrameSink()
        stop_event = asyncio.Event()
        consumer_task = None
        blackhole = MediaBlackhole()

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"[WiFi] connectionState={pc.connectionState}")

        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"[WiFi] iceConnectionState={pc.iceConnectionState}")

        @pc.on("track")
        def on_track(track):
            nonlocal consumer_task
            print(f"[WiFi] track received kind={track.kind}")
            if track.kind == "video":
                consumer_task = asyncio.create_task(
                    consume_track(track, sink, stop_event)
                )
            else:
                asyncio.ensure_future(blackhole.addTrack(track))

        await pc.setRemoteDescription(
            RTCSessionDescription(sdp=offer["sdp"], type=offer.get("type", "offer"))
        )
        # Prefer VP8 — aiortc/PyAV often fails on Record3D's H264 High profile
        # and the phone then closes the PeerConnection.
        try:
            from aiortc import RTCRtpSender

            caps = RTCRtpSender.getCapabilities("video")
            prefer = [c for c in caps.codecs if c.mimeType.lower() == "video/vp8"]
            if prefer:
                for t in pc.getTransceivers():
                    if t.kind == "video":
                        t.setCodecPreferences(prefer)
                        print("[WiFi] codec preference: VP8")
        except Exception as e:
            print(f"[WiFi] codec preference skipped: {e}")

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # Wait for ICE gathering (aiortc usually completes quickly).
        for _ in range(100):
            if pc.iceGatheringState == "complete":
                break
            await asyncio.sleep(0.05)
        print(f"[WiFi] local SDP ready iceGathering={pc.iceGatheringState}")

        payload = {"type": "answer", "data": pc.localDescription.sdp}
        # Demo HTML posts to /answer; README mentions /sendAnswer — try both.
        posted = False
        for path in ("/answer", "/sendAnswer"):
            try:
                async with session.post(
                    f"{base}{path}",
                    json=payload,
                    timeout=5,
                ) as resp:
                    print(f"[WiFi] POST {path} -> {resp.status}")
                    if resp.status < 400:
                        posted = True
                        break
            except Exception as e:
                print(f"[WiFi] POST {path} failed: {e}")
        if not posted:
            raise RuntimeError("Failed to POST WebRTC answer to Record3D")

        # Wait until ICE/peer is connected (or timeout).
        t_conn = time.time()
        while time.time() - t_conn < 10.0:
            if pc.iceConnectionState in ("connected", "completed"):
                break
            if pc.connectionState == "connected":
                break
            await asyncio.sleep(0.1)
        print(
            f"[WiFi] after ICE wait: ice={pc.iceConnectionState} "
            f"conn={pc.connectionState}"
        )

        # Wait for first frame
        t0 = time.time()
        first = None
        while time.time() - t0 < 20.0:
            first = await sink.take()
            if first is not None:
                break
            await asyncio.sleep(0.05)
        if first is None:
            stop_event.set()
            await pc.close()
            raise RuntimeError(
                "No video frame received over WebRTC "
                f"(video_rx={sink.count}, ice={pc.iceConnectionState})"
            )

        color, depth = decode_rgbd_frame(first, depth_max)
        h, w = depth.shape
        intrinsic, fx, fy, cx, cy = parse_intrinsics(meta, w, h)
        print(
            f"[WiFi] first frame color={color.shape} depth={depth.shape} "
            f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
        )

        device = o3d.core.Device("CPU:0")
        print("[WiFi] using CPU:0 (pip Open3D wheel is typically CPU-only)")

        depth_scale = 1000.0
        voxel_size = 0.007
        block_count = 20000
        model = o3d.t.pipelines.slam.Model(
            voxel_size, 16, min(block_count, 20000),
            o3d.core.Tensor.eye(4, dtype=o3d.core.Dtype.Float64),
            device,
        )
        input_frame = o3d.t.pipelines.slam.Frame(h, w, intrinsic, device)
        raycast_frame = o3d.t.pipelines.slam.Frame(h, w, intrinsic, device)
        T = o3d.core.Tensor.eye(4, dtype=o3d.core.Dtype.Float64)

        def to_rgbd(c_rgb, d_mm):
            c = o3d.t.geometry.Image(o3d.core.Tensor(c_rgb))
            d = o3d.t.geometry.Image(o3d.core.Tensor(d_mm))
            return o3d.t.geometry.RGBDImage(c, d)

        rgbd0 = to_rgbd(color, depth).to(device)
        input_frame.set_data_from_image("depth", rgbd0.depth)
        input_frame.set_data_from_image("color", rgbd0.color)
        model.integrate(input_frame, depth_scale, depth_max, 8.0)
        model.update_frame_pose(0, T)

        stats = {
            "frames": 1,
            "integrated": 1,
            "track_ok": 0,
            "track_fail": 0,
            "max_hash": int(model.get_hashmap().size()),
        }
        prev = rgbd0
        frame_id = 1
        deadline = time.time() + seconds
        print(f"[WiFi] SLAM running for {seconds:.0f}s...")

        while time.time() < deadline:
            img = await sink.take()
            if img is None:
                await asyncio.sleep(0.01)
                continue
            color, depth = decode_rgbd_frame(img, depth_max)
            if depth.shape[0] != h or depth.shape[1] != w:
                continue
            rgbd = to_rgbd(color, depth).to(device)
            input_frame.set_data_from_image("depth", rgbd.depth)
            input_frame.set_data_from_image("color", rgbd.color)
            stats["frames"] += 1

            try:
                model.update_frame_pose(frame_id, T)
                model.synthesize_model_frame(
                    raycast_frame, depth_scale, 0.1, depth_max, 8.0, False
                )
                result = model.track_frame_to_model(
                    input_frame,
                    raycast_frame,
                    depth_scale,
                    depth_max,
                    0.10,
                    o3d.t.pipelines.odometry.Method.PointToPlane,
                    [6, 3, 2],
                )
                fitness = float(result.fitness)
                if fitness >= 0.15:
                    T = T @ result.transformation.cpu()
                    model.update_frame_pose(frame_id, T)
                    model.integrate(input_frame, depth_scale, depth_max, 8.0)
                    stats["integrated"] += 1
                    stats["track_ok"] += 1
                else:
                    stats["track_fail"] += 1
            except Exception:
                stats["track_fail"] += 1

            stats["max_hash"] = max(
                stats["max_hash"], int(model.get_hashmap().size())
            )
            if stats["frames"] % 30 == 0:
                print(
                    f"[WiFi] frames={stats['frames']} integrated={stats['integrated']} "
                    f"ok={stats['track_ok']} fail={stats['track_fail']} "
                    f"hash={stats['max_hash']} video_rx={sink.count}"
                )
            prev = rgbd
            frame_id += 1

        stop_event.set()
        if consumer_task:
            consumer_task.cancel()
        await pc.close()

        # Save a quick point extract if possible
        out_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = out_dir / "wifi_slam_points.ply"
        try:
            pcd = model.extract_pointcloud(3.0, stats["max_hash"] * 1600)
            o3d.t.io.write_point_cloud(str(pcd_path), pcd)
            print(f"[WiFi] wrote {pcd_path}")
        except Exception as e:
            print(f"[WiFi] pointcloud extract skipped: {e}")

        report = {
            "host": base,
            "metadata": meta,
            "depth_max_m": depth_max,
            "device": str(device),
            "stats": stats,
            "video_frames_received": sink.count,
            "note": "Wi-Fi Streaming has no ARKit pose; RGB-D odometry only.",
        }
        report_path = out_dir / "wifi_slam_report.json"
        report_path.write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        print(f"[WiFi] report {report_path}")
        print(f"[WiFi] DONE stats={stats}")
        return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.20.10.1")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--depth-max", type=float, default=3.0)
    ap.add_argument(
        "--out-dir",
        default=str(Path("Analysis") / "iphone_wifi_webrtc_slam"),
    )
    args = ap.parse_args()
    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        # Prefer repo Analysis/
        repo = Path(__file__).resolve().parents[2]
        out_dir = repo / "Analysis" / "iphone_wifi_webrtc_slam"

    try:
        asyncio.run(
            run_wifi_slam(args.host, args.seconds, args.depth_max, out_dir)
        )
    except Exception as e:
        print(f"[WiFi] FAILED: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
