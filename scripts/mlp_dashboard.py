#!/usr/bin/env python3
"""MLP 위험 판단을 브라우저에서 실시간으로 본다.

/argos/fire_detection 을 구독해서 SSE 로 흘려보내고 한 장짜리 페이지를 낸다.
원본(new_main_robot_map.py)이 쓰는 MLP 는 특징 8 개를 받는다:

    fire_conf, smoke_conf, cigarette_conf, spark_conf,
    temperature, gas, temp_change, gas_change

이 중 temp_change / gas_change 는 토픽에 실리지 않는다.  원본과 같은 2 초
창으로 여기서 다시 계산한다 -- 그래서 값이 원본과 완전히 같지는 않을 수
있다 (표본 시점이 다르다).  판정 자체는 원본이 낸 mlp_prob 를 그대로 쓴다.
"""

import collections
import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

PORT = 8088
TOPIC = "/argos/fire_detection"
THRESHOLD = 0.70          # 원본 FIRE_PROB_THRESHOLD
CHANGE_WINDOW = 2.0       # 원본 CHANGE_WINDOW_SECONDS
CLASSES = ["fire", "smoke", "cigarette_butt", "spark"]

_clients: list[queue.Queue] = []
_clients_lock = threading.Lock()
_latest = {"raw": None}


class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("mlp_dashboard")
        self.history: collections.deque = collections.deque()
        # 인지 노드가 BEST_EFFORT 로 낸다.  BEST_EFFORT + VOLATILE 은
        # 양쪽 어느 조합의 발행자와도 붙는 가장 관대한 구독이다.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(String, TOPIC, self._on, qos)
        self.get_logger().info(f"{TOPIC} 구독, http://0.0.0.0:{PORT}")

    def _on(self, message: String) -> None:
        try:
            d = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return

        stamp = float(d.get("stamp", 0.0))
        sensor = d.get("sensor") or {}
        temp = float(sensor.get("temp", 0.0))
        gas = float(sensor.get("gas", 0.0))

        # 원본과 같은 2 초 창으로 변화량을 다시 만든다.
        self.history.append((stamp, temp, gas))
        while self.history and stamp - self.history[0][0] > CHANGE_WINDOW:
            self.history.popleft()
        if len(self.history) >= 2:
            _, t0, g0 = self.history[0]
            d_temp, d_gas = temp - t0, gas - g0
        else:
            d_temp = d_gas = 0.0

        confs = d.get("confs") or {}
        payload = {
            "seq": d.get("seq"),
            "stamp": stamp,
            "prob": float(d.get("mlp_prob", 0.0)),
            "danger": bool(d.get("mlp_danger", False)),
            "duration": float(d.get("danger_duration", 0.0)),
            "alert_expected": bool(d.get("alert_expected", False)),
            "sensor_ok": bool(sensor.get("ok", False)),
            "temp": temp,
            "gas": gas,
            "d_temp": d_temp,
            "d_gas": d_gas,
            "confs": {c: float(confs.get(c, 0.0)) for c in CLASSES},
            "boxes": d.get("boxes") or {},
            "telegram": d.get("telegram") or {},
        }

        line = "data: " + json.dumps(payload) + "\n\n"
        _latest["raw"] = line
        with _clients_lock:
            dead = []
            for q in _clients:
                try:
                    q.put_nowait(line)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                _clients.remove(q)


PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARGOS MLP 판단</title>
<style>
:root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--fg:#e6edf3;--dim:#8b949e;
      --ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
.wrap{max-width:1100px;margin:0 auto;padding:20px}
h1{font-size:16px;margin:0 0 4px;font-weight:600}
.sub{color:var(--dim);font-size:12px;margin-bottom:18px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
@media(max-width:820px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);
      border-radius:8px;padding:16px}
.card h2{font-size:11px;letter-spacing:.08em;text-transform:uppercase;
         color:var(--dim);margin:0 0 12px;font-weight:600}
.big{font-size:60px;font-weight:700;line-height:1;font-variant-numeric:tabular-nums}
.verdict{display:inline-block;padding:4px 12px;border-radius:999px;
         font-size:12px;font-weight:700;margin-top:10px}
.safe{background:rgba(63,185,80,.15);color:var(--ok);border:1px solid var(--ok)}
.dang{background:rgba(248,81,73,.15);color:var(--bad);border:1px solid var(--bad)}
.bar{position:relative;height:22px;background:#010409;border-radius:4px;
     margin-top:14px;overflow:hidden;border:1px solid var(--line)}
.fill{height:100%;width:0;background:var(--acc);transition:width .12s linear}
.fill.hot{background:var(--bad)}
.mark{position:absolute;top:-3px;bottom:-3px;left:70%;width:2px;background:var(--warn)}
.marklab{color:var(--warn);font-size:11px;margin-top:5px}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
td{padding:5px 0;border-bottom:1px solid var(--line)}
td:last-child{text-align:right;font-weight:600}
tr:last-child td{border-bottom:none}
.k{color:var(--dim);font-weight:400}
.spark{width:100%;height:70px;display:block}
.pill{font-size:11px;padding:2px 8px;border-radius:4px;background:#010409;
      border:1px solid var(--line);color:var(--dim)}
.up{color:var(--bad)}.dn{color:var(--acc)}.zero{color:var(--dim)}
.off{opacity:.4}
</style></head><body><div class="wrap">
<h1>ARGOS &mdash; MLP 위험 판단</h1>
<div class="sub">
  특징 8개 &rarr; MLP &rarr; 위험 확률.  임계값 <b>0.70</b> 이상이 위험.
  <span id="conn" class="pill">연결 중</span>
</div>

<div class="grid">
  <div class="card">
    <h2>위험 확률 (mlp_prob)</h2>
    <div class="big" id="prob">&mdash;</div>
    <div id="verdict" class="verdict safe">대기</div>
    <div class="bar"><div class="fill" id="fill"></div><div class="mark"></div></div>
    <div class="marklab">&#9650; 임계값 0.70</div>
    <table style="margin-top:14px">
      <tr><td class="k">위험 지속</td><td id="dur">&mdash;</td></tr>
      <tr><td class="k">알림 예정</td><td id="alert">&mdash;</td></tr>
      <tr><td class="k">프레임</td><td id="seq">&mdash;</td></tr>
    </table>
  </div>

  <div class="card">
    <h2>확률 추이 (최근 200 프레임)</h2>
    <canvas class="spark" id="spark" width="520" height="70"></canvas>
    <h2 style="margin-top:18px">텔레그램</h2>
    <table>
      <tr><td class="k">상태</td><td id="tg">&mdash;</td></tr>
      <tr><td class="k">보낸 횟수</td><td id="tgseq">&mdash;</td></tr>
      <tr><td class="k">사진</td><td id="tgphoto">&mdash;</td></tr>
    </table>
  </div>

  <div class="card">
    <h2>MLP 입력 &mdash; YOLO 신뢰도</h2>
    <table id="confs"></table>
  </div>

  <div class="card">
    <h2>MLP 입력 &mdash; 센서</h2>
    <table>
      <tr><td class="k">temperature</td><td id="temp">&mdash;</td></tr>
      <tr><td class="k">gas</td><td id="gas">&mdash;</td></tr>
      <tr><td class="k">temp_change <span style="font-weight:400">(2s)</span></td>
          <td id="dtemp">&mdash;</td></tr>
      <tr><td class="k">gas_change <span style="font-weight:400">(2s)</span></td>
          <td id="dgas">&mdash;</td></tr>
      <tr><td class="k">센서 연결</td><td id="sok">&mdash;</td></tr>
    </table>
  </div>
</div>
</div>
<script>
const CLASSES = ["fire","smoke","cigarette_butt","spark"];
const hist = [];
const $ = (id) => document.getElementById(id);

function sign(v, digits, cls) {
  const s = (v >= 0 ? "+" : "") + v.toFixed(digits);
  const k = Math.abs(v) < 1e-9 ? "zero" : (v > 0 ? "up" : "dn");
  return '<span class="' + k + '">' + s + '</span>';
}

function drawSpark() {
  const c = $("spark"), g = c.getContext("2d");
  const w = c.width, h = c.height;
  g.clearRect(0, 0, w, h);
  // 임계선
  g.strokeStyle = "#d29922"; g.setLineDash([3, 3]); g.beginPath();
  const ty = h - 0.70 * h;
  g.moveTo(0, ty); g.lineTo(w, ty); g.stroke(); g.setLineDash([]);
  if (hist.length < 2) return;
  g.beginPath();
  hist.forEach((v, i) => {
    const x = i / (hist.length - 1) * w;
    const y = h - Math.max(0, Math.min(1, v)) * h;
    i ? g.lineTo(x, y) : g.moveTo(x, y);
  });
  g.strokeStyle = hist[hist.length - 1] >= 0.70 ? "#f85149" : "#58a6ff";
  g.lineWidth = 2; g.stroke();
}

function render(d) {
  $("prob").textContent = d.prob.toFixed(3);
  const hot = d.prob >= 0.70;
  $("prob").style.color = hot ? "var(--bad)" : "var(--fg)";
  const v = $("verdict");
  v.className = "verdict " + (d.danger ? "dang" : "safe");
  v.textContent = d.danger ? "위험 판정" : "정상";
  const f = $("fill");
  f.style.width = Math.max(0, Math.min(1, d.prob)) * 100 + "%";
  f.className = "fill" + (hot ? " hot" : "");

  $("dur").textContent = d.duration.toFixed(1) + " s";
  $("alert").textContent = d.alert_expected ? "예" : "아니오";
  $("seq").textContent = d.seq;

  const rows = CLASSES.map((c) => {
    const val = d.confs[c] || 0;
    const box = d.boxes && d.boxes[c] ? ' <span class="pill">box</span>' : "";
    const col = val > 0 ? "" : ' class="off"';
    return "<tr" + col + '><td class="k">' + c + box + "</td><td>"
         + val.toFixed(3) + "</td></tr>";
  });
  $("confs").innerHTML = rows.join("");

  $("temp").textContent = d.temp.toFixed(1);
  $("gas").textContent = d.gas.toFixed(0);
  $("dtemp").innerHTML = sign(d.d_temp, 2);
  $("dgas").innerHTML = sign(d.d_gas, 0);
  $("sok").innerHTML = d.sensor_ok
      ? '<span style="color:var(--ok)">연결됨</span>'
      : '<span style="color:var(--bad)">끊김</span>';

  const t = d.telegram || {};
  $("tg").textContent = t.state || "—";
  $("tgseq").textContent = t.seq === undefined ? "—" : t.seq;
  $("tgphoto").textContent = t.photos_ok === undefined ? "—" : t.photos_ok;

  hist.push(d.prob);
  if (hist.length > 200) hist.shift();
  drawSpark();
}

const es = new EventSource("/stream");
es.onopen = () => {
  $("conn").textContent = "연결됨";
  $("conn").style.color = "var(--ok)";
};
es.onerror = () => {
  $("conn").textContent = "끊김 — 재연결 중";
  $("conn").style.color = "var(--bad)";
};
es.onmessage = (e) => { try { render(JSON.parse(e.data)); } catch (_) {} };
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_a):        # 접속 로그로 콘솔을 더럽히지 않는다
        pass

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            q: queue.Queue = queue.Queue(maxsize=64)
            with _clients_lock:
                _clients.append(q)
            if _latest["raw"]:
                try:
                    self.wfile.write(_latest["raw"].encode())
                    self.wfile.flush()
                except OSError:
                    pass
            try:
                while True:
                    try:
                        self.wfile.write(q.get(timeout=15).encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with _clients_lock:
                    if q in _clients:
                        _clients.remove(q)
            return

        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    rclpy.init()
    node = Bridge()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
