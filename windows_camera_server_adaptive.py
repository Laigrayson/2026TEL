#!/usr/bin/env python3
"""
Windows 鏡頭串流 - 弱網路自適應版
原理:手機每次只拿「最新一張」畫面,網路卡就自動降畫質/縮小,不會累積延遲。

安裝: python -m pip install opencv-python flask
執行: python windows_camera_server_adaptive.py
"""

import socket
import threading
import time

import cv2
from flask import Flask, Response, abort, render_template_string, request
from werkzeug.serving import WSGIRequestHandler

# ================= 可調整設定 =================
CAMERA_INDEX = 1        # Windows 的鏡頭索引，若開不到可改為 0
CAP_WIDTH = 640         # 鏡頭擷取解析度 (再由程式縮小)
CAP_HEIGHT = 480
CAP_FPS = 30
PORT = 5000
ROTATE_180 = True       # 鏡頭倒裝時設為 True,畫面會旋轉 180 度

# 畫質等級: (輸出寬度, JPEG 品質),由低到高,網頁會自動切換
LEVELS = [
    (160, 30),
    (240, 40),
    (320, 50),
    (480, 60),
    (640, 70),
]
START_LEVEL = 2
# ============================================

app = Flask(__name__)


class Camera:
    def __init__(self):
        self.cap = self._open_capture()

        self.img = None
        self.frame_id = 0
        self.cache = {}                 # (frame_id, level) -> jpeg bytes
        self.cond = threading.Condition()
        threading.Thread(target=self._update, daemon=True).start()

    @staticmethod
    def _open_capture():
        for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF):
            cap = cv2.VideoCapture(CAMERA_INDEX, backend)
            if not cap.isOpened():
                cap.release()
                continue

            if backend == cv2.CAP_DSHOW:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAP_FPS)

            ok, _ = cap.read()
            if ok:
                print(f"鏡頭索引 {CAMERA_INDEX}，擷取後端 {cap.getBackendName()}")
                return cap
            cap.release()

        raise RuntimeError(
            f"無法從 Windows 鏡頭索引 {CAMERA_INDEX} 讀取影像。"
            "請確認相機權限及裝置是否被占用，或試著將 CAMERA_INDEX 改為 0。"
        )

    def _update(self):
        while True:
            ok, img = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            if ROTATE_180:
                img = cv2.rotate(img, cv2.ROTATE_180)
            with self.cond:
                self.img = img
                self.frame_id += 1
                self.cache = {}         # 新畫面就清掉舊快取
                self.cond.notify_all()

    def get(self, level, last_id):
        """等到有比 last_id 更新的畫面,回傳該等級的 JPEG。"""
        with self.cond:
            self.cond.wait_for(
                lambda: self.img is not None and self.frame_id != last_id,
                timeout=1.0,
            )
            if self.img is None:
                return None, 0
            fid = self.frame_id
            key = (fid, level)
            if key in self.cache:
                return self.cache[key], fid
            img = self.img

        width, quality = LEVELS[level]
        h, w = img.shape[:2]
        if width < w:
            img = cv2.resize(
                img, (width, int(h * width / w)), interpolation=cv2.INTER_AREA
            )
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None, 0
        data = buf.tobytes()
        with self.cond:
            if self.frame_id == fid:
                self.cache[key] = data
        return data, fid


camera = Camera()

PAGE = """
<!DOCTYPE html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Windows 即時畫面</title>
  <style>
    body{margin:0;background:#000;color:#fff;font-family:-apple-system,sans-serif}
    #wrap{position:relative;max-width:960px;margin:0 auto}
    img{width:100%;height:auto;display:block;background:#111;min-height:200px}
    #stat{position:absolute;left:8px;top:8px;font-size:12px;background:#0008;
          padding:3px 7px;border-radius:6px}
    #crosshair {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 36px;
      height: 36px;
      transform: translate(-50%, -50%);
      pointer-events: none;
      filter: drop-shadow(0 0 1px black) drop-shadow(0 0 1px black);
    }
    #crosshair::before,
    #crosshair::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      background: #ff2020;
    }
    #crosshair::before { width: 36px; height: 4px; }
    #crosshair::after { width: 4px; height: 36px; }
  </style>
</head>
<body>
<div id="wrap">
  <img id="v" alt="live">
  <div id="crosshair" aria-hidden="true"></div>
  <div id="stat">連線中…</div>
</div>
<script>
const LEVELS = {{ n_levels }};
let level = {{ start_level }};
let lastId = 0, ema = 100, stable = 0, frames = 0, lastT = performance.now();
const img = document.getElementById('v'), stat = document.getElementById('stat');

function adapt(rtt) {
  ema = ema * 0.7 + rtt * 0.3;
  if (ema > 180 && level > 0) { level--; stable = 0; ema = 100; }
  else if (ema < 70 && level < LEVELS - 1) {
    if (++stable >= 20) { level++; stable = 0; ema = 100; }
  } else stable = 0;
}

async function loop() {
  const t0 = performance.now();
  try {
    const r = await fetch(`/frame?l=${level}&last=${lastId}`, {cache: 'no-store'});
    if (r.status === 200) {
      const id = +r.headers.get('X-Frame-Id');
      if (id <= lastId) { return loop(); }   // 重複或過期的畫面,直接丟掉
      lastId = id;
      const blob = await r.blob();
      const url = URL.createObjectURL(blob);
      img.onload = () => URL.revokeObjectURL(url);
      img.src = url;
      adapt(performance.now() - t0);
      frames++;
    }
  } catch (e) {
    await new Promise(r => setTimeout(r, 300));
  }
  const now = performance.now();
  if (now - lastT > 1000) {
    stat.textContent = `${frames} fps · 等級 ${level + 1}/${LEVELS} · ${Math.round(ema)}ms`;
    frames = 0; lastT = now;
  }
  loop();
}
// 同時跑 3 條請求,錯開啟動,用來對抗高延遲網路
for (let i = 0; i < 3; i++) setTimeout(loop, i * 120);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(
        PAGE, n_levels=len(LEVELS), start_level=START_LEVEL
    )


@app.route("/frame")
def frame():
    try:
        level = max(0, min(len(LEVELS) - 1, int(request.args.get("l", START_LEVEL))))
        last_id = int(request.args.get("last", 0))
    except ValueError:
        abort(400)
    data, fid = camera.get(level, last_id)
    if data is None:
        return Response(status=204)
    resp = Response(data, mimetype="image/jpeg")
    resp.headers["X-Frame-Id"] = str(fid)
    resp.headers["Cache-Control"] = "no-store"
    return resp


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


if __name__ == "__main__":
    WSGIRequestHandler.protocol_version = "HTTP/1.1"   # keep-alive,省掉重複連線
    print(f"\n✅ 啟動!iPhone Safari 開啟: http://{get_ip()}:{PORT}\n")
    app.run(host="0.0.0.0", port=PORT, threaded=True)
