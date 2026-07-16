#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Quick Chromium grab-rate smoke test for Record3D Wi-Fi."""

from __future__ import annotations

import argparse
import base64
import json
import time
import urllib.request

import cv2
import numpy as np
from playwright.sync_api import sync_playwright

JS = """
() => {
  const v = document.querySelector('video');
  if (!v || v.videoWidth < 2) return null;
  const maxW = 640;
  let dw = v.videoWidth, dh = v.videoHeight;
  if (dw > maxW) { dh = Math.round(dh * maxW / dw); dw = maxW; }
  const c = document.createElement('canvas');
  c.width = dw; c.height = dh;
  c.getContext('2d').drawImage(v, 0, 0, dw, dh);
  return {
    nw: v.videoWidth, nh: v.videoHeight, w: dw, h: dh,
    jpeg: c.toDataURL('image/jpeg', 0.6)
  };
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.110")
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--demo-url", default="http://127.0.0.1:8765/")
    args = ap.parse_args()

    meta = json.load(urllib.request.urlopen(f"http://{args.host}/metadata", timeout=5))
    print("meta", meta, flush=True)

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        page.goto(args.demo_url, wait_until="domcontentloaded")
        page.fill("#remote-address", args.host)
        page.click("input[type=submit]")
        t0 = time.time()
        n = 0
        best = 0
        while time.time() - t0 < args.seconds and n < 40:
            payload = page.evaluate(JS)
            if not payload:
                time.sleep(0.15)
                continue
            best = max(best, int(payload["nw"]))
            b64 = payload["jpeg"].split(",", 1)[1]
            arr = np.frombuffer(base64.b64decode(b64), np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            n += 1
            if n <= 3 or n % 5 == 0:
                shape = None if img is None else img.shape
                print(
                    f"grab {n}: native={payload['nw']}x{payload['nh']} "
                    f"xfer={payload['w']}x{payload['h']} img={shape} "
                    f"dt={time.time()-t0:.1f}s",
                    flush=True,
                )
        browser.close()
        print(f"DONE grabs={n} best_native_w={best}", flush=True)
    return 0 if n > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
