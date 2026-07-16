#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture Record3D Wi-Fi stream via Chromium (Playwright) and run Open3D SLAM.

Chrome handles H264 from Record3D; aiortc on Windows often drops the media.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from playwright.sync_api import sync_playwright


def decode_rgbd(frame_bgr: np.ndarray, depth_max_m: float = 3.0):
    h, w = frame_bgr.shape[:2]
    half = w // 2
    depth_bgr = frame_bgr[:, :half]
    color_bgr = frame_bgr[:, half:]
    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(depth_bgr, cv2.COLOR_BGR2HSV)
    hue01 = hsv[:, :, 0].astype(np.float32) / 179.0
    depth_m = depth_max_m * hue01
    depth_mm = np.clip(np.rint(depth_m * 1000.0), 0, 65535).astype(np.uint16)
    return color_rgb, depth_mm


def parse_K(meta: dict, w: int, h: int):
    k = meta["K"]
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[6]), float(k[7])
    ow, oh = meta.get("originalSize", [w, h])
    ow, oh = int(ow), int(oh)
    if ow > 0 and oh > 0 and (ow != w or oh != h):
        fx *= w / ow
        fy *= h / oh
        cx *= w / ow
        cy *= h / oh
    return o3d.core.Tensor(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=o3d.core.Dtype.Float64,
    ), fx, fy, cx, cy


GRAB_JS = """
(maxW, jpegQ) => {
  const v = document.querySelector('video');
  if (!v || v.videoWidth < 2 || v.videoHeight < 2) return null;
  const c = document.createElement('canvas');
  c.width = v.videoWidth; c.height = v.videoHeight;
  c.getContext('2d').drawImage(v, 0, 0);
  let dw = c.width, dh = c.height;
  // maxW<=0 => keep native side-by-side resolution
  if (maxW > 0 && dw > maxW) {
    dh = Math.round(dh * maxW / dw);
    dw = maxW;
  }
  const c2 = document.createElement('canvas');
  c2.width = dw; c2.height = dh;
  c2.getContext('2d').drawImage(c, 0, 0, dw, dh);
  const q = Math.min(Math.max(jpegQ || 0.92, 0.5), 1.0);
  return {
    native_w: c.width, native_h: c.height,
    w: dw, h: dh,
    jpeg: c2.toDataURL('image/jpeg', q)
  };
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.20.10.1")
    ap.add_argument("--seconds", type=float, default=40)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-extract", action="store_true")
    ap.add_argument("--depth-max", type=float, default=3.0)
    ap.add_argument(
        "--max-xfer-w",
        type=int,
        default=1280,
        help="Max WebRTC canvas width (side-by-side). 0=native.",
    )
    ap.add_argument("--slam-width", type=int, default=640)
    ap.add_argument("--slam-height", type=int, default=480)
    ap.add_argument(
        "--slam-max-h",
        type=int,
        default=0,
        help="Deprecated; use --slam-height.",
    )
    ap.add_argument("--jpeg-q", type=float, default=0.85)
    ap.add_argument("--voxel", type=float, default=0.012)
    ap.add_argument("--min-native-w", type=int, default=800)
    ap.add_argument("--demo-url", default="http://127.0.0.1:8765/")
    args = ap.parse_args()

    out_dir = Path(__file__).resolve().parents[2] / "Analysis" / "iphone_wifi_webrtc_slam"
    out_dir.mkdir(parents=True, exist_ok=True)

    import urllib.request

    meta = json.load(urllib.request.urlopen(f"http://{args.host}/metadata", timeout=5))
    print(f"[WiFi-Chrome] metadata={meta}", flush=True)
    ow, oh = meta.get("originalSize", [0, 0])
    target_native_w = int(args.min_native_w)
    print(
        f"[WiFi-Chrome] meta original={ow}x{oh} target native_w>={target_native_w} "
        f"(xfer_w={args.max_xfer_w or 'native'}, "
        f"slam={args.slam_width}x{args.slam_height})",
        flush=True,
    )

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        page.goto(args.demo_url, wait_until="domcontentloaded")
        page.fill("#remote-address", args.host)
        page.click("input[type=submit]")
        print("[WiFi-Chrome] clicked start streaming", flush=True)

        def to_bgr(payload):
            import base64

            b64 = payload["jpeg"].split(",", 1)[1]
            arr = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return img

        def grab():
            return page.evaluate(GRAB_JS, [args.max_xfer_w, args.jpeg_q])

        # Wait for highest attainable WebRTC resolution (plateau).
        t0 = time.time()
        first = None
        best = None
        plateau = 0
        while time.time() - t0 < 40:
            first = grab()
            if not first:
                time.sleep(0.3)
                continue
            nw = int(first.get("native_w", 0))
            if best is None or nw > int(best.get("native_w", 0)):
                best = first
                plateau = 0
                print(
                    f"[WiFi-Chrome] video native {first['native_w']}x{first['native_h']} "
                    f"xfer {first['w']}x{first['h']}",
                    flush=True,
                )
            else:
                plateau += 1
            if nw >= target_native_w:
                print(f"[WiFi-Chrome] reached target native_w={nw}", flush=True)
                break
            # Stable at best for ~2s after at least 800px wide
            if plateau >= 6 and nw >= 800:
                print(
                    f"[WiFi-Chrome] plateau at native {best['native_w']}x{best['native_h']}",
                    flush=True,
                )
                break
            time.sleep(0.35)
        first = best or first
        if not first or int(first.get("w", 0)) < 200:
            browser.close()
            raise SystemExit("No video frames from Chromium WebRTC")

        # One more grab at best settings
        first = grab() or first
        frame0 = to_bgr(first)
        sample = out_dir / "wifi_vga_sample.jpg"
        cv2.imwrite(str(sample), frame0)
        print(
            f"[WiFi-Chrome] sample {frame0.shape[1]}x{frame0.shape[0]} -> {sample}",
            flush=True,
        )

        color, depth = decode_rgbd(frame0, args.depth_max)
        sw, sh = int(args.slam_width), int(args.slam_height)
        if sw > 0 and sh > 0 and (color.shape[1] != sw or color.shape[0] != sh):
            color = cv2.resize(color, (sw, sh), interpolation=cv2.INTER_AREA)
            depth = cv2.resize(depth, (sw, sh), interpolation=cv2.INTER_NEAREST)
        elif args.slam_max_h > 0 and color.shape[0] > args.slam_max_h:
            scale = args.slam_max_h / float(color.shape[0])
            nw = max(32, int(round(color.shape[1] * scale)))
            nh = args.slam_max_h
            color = cv2.resize(color, (nw, nh), interpolation=cv2.INTER_AREA)
            depth = cv2.resize(depth, (nw, nh), interpolation=cv2.INTER_NEAREST)
        h, w = depth.shape
        intrinsic, fx, fy, cx, cy = parse_K(meta, w, h)
        print(
            f"[WiFi-Chrome] SLAM res {w}x{h} fx={fx:.1f} fy={fy:.1f} "
            f"cx={cx:.1f} cy={cy:.1f} voxel={args.voxel}",
            flush=True,
        )

        try:
            device = o3d.core.Device("CUDA:0")
            _ = o3d.core.Tensor([1.0], device=device)
            print("[WiFi-Chrome] device=CUDA:0", flush=True)
        except Exception:
            device = o3d.core.Device("CPU:0")
            print("[WiFi-Chrome] device=CPU:0", flush=True)

        depth_scale = 1000.0
        block_count = 20000 if max(h, w) >= 720 else 8000
        model = o3d.t.pipelines.slam.Model(
            args.voxel,
            16,
            block_count,
            o3d.core.Tensor.eye(4, dtype=o3d.core.Dtype.Float64),
            device,
        )
        input_frame = o3d.t.pipelines.slam.Frame(h, w, intrinsic, device)
        raycast_frame = o3d.t.pipelines.slam.Frame(h, w, intrinsic, device)
        T = o3d.core.Tensor.eye(4, dtype=o3d.core.Dtype.Float64)

        def push(c_rgb, d_mm):
            c_rgb = np.ascontiguousarray(c_rgb, dtype=np.uint8)
            d_mm = np.ascontiguousarray(d_mm, dtype=np.uint16)
            color_t = o3d.t.geometry.Image(o3d.core.Tensor(c_rgb))
            depth_t = o3d.t.geometry.Image(o3d.core.Tensor(d_mm))
            input_frame.set_data_from_image("depth", depth_t)
            input_frame.set_data_from_image("color", color_t)

        push(color, depth)
        model.integrate(input_frame, depth_scale, args.depth_max, 8.0)
        model.update_frame_pose(0, T)
        model.synthesize_model_frame(
            raycast_frame, depth_scale, 0.1, args.depth_max, 8.0, False
        )

        stats = {
            "frames": 1,
            "integrated": 1,
            "track_ok": 0,
            "track_fail": 0,
            "max_hash": int(model.get_hashmap().size()),
            "video_wh": [int(first["w"]), int(first["h"])],
            "native_wh": [int(first.get("native_w", 0)), int(first.get("native_h", 0))],
            "slam_wh": [w, h],
            "voxel": args.voxel,
        }
        frame_id = 1
        deadline = time.time() + args.seconds
        max_frames = args.max_frames if args.max_frames > 0 else max(
            int(args.seconds * 3), 20
        )
        print(
            f"[WiFi-Chrome] SLAM for {args.seconds:.0f}s "
            f"(max_frames={max_frames})...",
            flush=True,
        )

        while time.time() < deadline and stats["frames"] < max_frames:
            t_frame = time.time()
            payload = grab()
            if not payload:
                time.sleep(0.05)
                continue
            bgr = to_bgr(payload)
            color, depth = decode_rgbd(bgr, args.depth_max)
            if color.shape[0] != h or color.shape[1] != w:
                color = cv2.resize(color, (w, h), interpolation=cv2.INTER_AREA)
                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
            push(color, depth)
            stats["frames"] += 1
            try:
                result = model.track_frame_to_model(
                    input_frame,
                    raycast_frame,
                    depth_scale,
                    args.depth_max,
                    0.07,
                )
                if float(result.fitness) >= 0.15:
                    T = T @ result.transformation.cpu()
                    model.update_frame_pose(frame_id, T)
                    model.integrate(input_frame, depth_scale, args.depth_max, 8.0)
                    model.synthesize_model_frame(
                        raycast_frame, depth_scale, 0.1, args.depth_max, 8.0, False
                    )
                    stats["integrated"] += 1
                    stats["track_ok"] += 1
                else:
                    stats["track_fail"] += 1
            except Exception as e:
                stats["track_fail"] += 1
                if stats["track_fail"] <= 3:
                    print(f"[WiFi-Chrome] track err: {e}", flush=True)
            stats["max_hash"] = max(
                stats["max_hash"], int(model.get_hashmap().size())
            )
            dt = time.time() - t_frame
            if stats["frames"] <= 5 or stats["frames"] % 5 == 0:
                print(
                    f"[WiFi-Chrome] frames={stats['frames']} "
                    f"int={stats['integrated']} ok={stats['track_ok']} "
                    f"fail={stats['track_fail']} hash={stats['max_hash']} "
                    f"dt={dt:.2f}s",
                    flush=True,
                )
            frame_id += 1

        browser.close()
        print("[WiFi-Chrome] browser closed", flush=True)

    if args.no_extract:
        print("[WiFi-Chrome] extract skipped (--no-extract)", flush=True)
    else:
        try:
            npts = min(max(stats["max_hash"] * 200, 20000), 250000)
            print(f"[WiFi-Chrome] extract_pointcloud n={npts}...", flush=True)
            pcd = model.extract_pointcloud(3.0, npts)
            ply = out_dir / "wifi_chrome_slam_vga_points.ply"
            o3d.t.io.write_point_cloud(str(ply), pcd)
            print(f"[WiFi-Chrome] wrote {ply}", flush=True)
            stats["ply"] = str(ply)
        except Exception as e:
            print(f"[WiFi-Chrome] extract skipped: {e}", flush=True)

    report = {
        "host": args.host,
        "method": "playwright-chromium + Open3D SLAM (640x480 Wi-Fi)",
        "metadata": meta,
        "stats": stats,
        "note": "No ARKit pose on Wi-Fi Streaming; RGB-D odometry only.",
    }
    report_path = out_dir / "wifi_chrome_slam_vga_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"[WiFi-Chrome] report {report_path}", flush=True)
    print(f"[WiFi-Chrome] DONE {stats}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
