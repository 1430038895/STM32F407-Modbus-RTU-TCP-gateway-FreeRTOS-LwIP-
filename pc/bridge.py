# 444.py —— 桥（长连接 + 自动重连）+ 自带网页。直接运行本文件即可。
import asyncio, json, socket, struct, threading, http.server
import websockets

MCU = "192.168.77.100"
DUMP_PORT = 5000
MB_PORT = 502
WS_PORT = 8000
HTTP_PORT = 8001
POLL_MS = 300

PAGE = """<!doctype html>
<html lang="zh"><head><meta charset="utf-8"><title>Modbus 网关</title>
<style>
 body{margin:0;font-family:"Microsoft YaHei",Consolas,monospace;background:#101418;color:#efefef;}
 .wrap{display:flex;height:100vh;}
 .list{width:200px;background:#1b2026;border-right:1px solid #2a2f36;overflow:auto;}
 .list div{padding:13px 14px;border-bottom:1px solid #2a2f36;cursor:pointer;color:#aaa;}
 .list div.sel{background:#1e4976;color:#fff;border-left:4px solid #42a5f5;}
 .data{flex:1;padding:16px 22px;overflow:auto;}
 h2{color:#42a5f5;font-size:16px;margin:0 0 14px;}
 table{width:100%;border-collapse:collapse;}
 td{padding:7px 8px;border-bottom:1px solid #2a2f36;}
 td.n{color:#aaa;width:46%;}
 input{width:150px;background:#1b2026;border:1px solid #2a2f36;color:#4caf50;
       font-size:15px;padding:5px 8px;border-radius:6px;}
 input:focus{outline:none;border-color:#42a5f5;color:#fff;}
</style></head>
<body>
<div class="wrap">
  <div class="list" id="list"></div>
  <div class="data"><h2 id="title">连接中…</h2><table id="tbl"></table></div>
</div>
<script>
var ZH = {"baud":"波特率","station":"站点号","decimals":"小数位","K":"K值",
 "flow_total":"累积流量","flow_total_pulse":"累积脉冲","flow_temp":"临时流量","flow_temp_pulse":"临时脉冲",
 "flow_instant":"瞬时流量","flow_instant_pulse":"瞬时脉冲","ma_4ma":"4mA下限","ma_20ma":"20mA上限",
 "press_A":"压力A","press_B":"压力B","pressure_kpa":"压力kPa","wind_speed":"风速","wind_dir":"风向",
 "air_temp":"气温","humidity":"湿度","pressure":"气压","rain_min":"分钟雨量","rain_hour":"小时雨量",
 "rain_day":"天雨量","rain_total":"累计雨量","dev_addr":"设备地址","temp":"温度","temp_c":"温度℃",
 "pm1":"PM1","pm25":"PM2.5","pm10":"PM10","co2":"CO2","hcho":"甲醛","voc":"VOC","ntc_res":"NTC电阻"};

var pts = [], sel = null, lastKeys = "", lastSel = null, sock = null;

function connect(){
  var url = "ws://" + (location.hostname || "127.0.0.1") + ":8000";
  sock = new WebSocket(url);
  sock.onmessage = function(e){ try{ pts = JSON.parse(e.data); render(); }catch(_){} };
  sock.onclose = function(){
    document.getElementById('title').textContent = "连接断开，重试中…";
    setTimeout(connect, 1000);
  };
}
connect();

function units(){
  var u = {};
  pts.forEach(function(p){ (u[p.unit] = u[p.unit] || {unit:p.unit, dev:p.dev, pts:[]}).pts.push(p); });
  return u;
}
function doWrite(inp, p){
  if(sock && sock.readyState === 1)
    sock.send(JSON.stringify({type:'write', idx:+p.idx, unit:+p.unit, reg:+p.reg, k:+p.k, d:+p.d, value:inp.value}));
}
function render(){
  var u = units();
  var ks = Object.keys(u).sort(function(a,b){ return a-b; });
  var keys = ks.join(",");
  if(sel === null && ks.length) sel = +ks[0];

  var ul = document.getElementById('list');
  if(keys !== lastKeys){
    lastKeys = keys; ul.innerHTML = '';
    ks.forEach(function(k){
      var d = document.createElement('div');
      d.textContent = u[k].dev; d.dataset.u = k;
      d.onclick = function(){ sel = +k; lastSel = null; render(); };
      ul.appendChild(d);
    });
  }
  Array.prototype.forEach.call(ul.children, function(d){
    d.className = (+d.dataset.u === sel) ? 'sel' : '';
  });

  var cur = u[sel]; if(!cur) return;
  document.getElementById('title').textContent = cur.dev + "  (unit " + sel + ")";
  var tbl = document.getElementById('tbl');

  var sig = sel + "|" + cur.pts.map(function(p){ return p.idx; }).join(",");
  if(sig !== lastSel){
    lastSel = sig; tbl.innerHTML = '';
    cur.pts.forEach(function(p){
      var tr = document.createElement('tr');
      var td1 = document.createElement('td'); td1.className = 'n';
      td1.textContent = ZH[p.point] || p.point;
      var td2 = document.createElement('td');
      var inp = document.createElement('input'); inp.value = p.val;
      inp.onchange = function(){ doWrite(inp, p); };
      inp.onkeydown = function(ev){ if(ev.key === 'Enter'){ doWrite(inp, p); inp.blur(); } };
      td2.appendChild(inp); tr.appendChild(td1); tr.appendChild(td2); tbl.appendChild(tr);
    });
  } else {
    var ins = tbl.querySelectorAll('input');
    Array.prototype.forEach.call(ins, function(inp, i){
      if(document.activeElement !== inp && cur.pts[i] && inp.value !== cur.pts[i].val) inp.value = cur.pts[i].val;
    });
  }
}
</script>
</body></html>
"""

def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        c = sock.recv(n - len(buf))
        if not c:
            raise ConnectionError("closed")
        buf += c
    return buf

def decode_val(v):
    if v.startswith("F32:") and len(v) >= 12:
        try:
            return "%.3f" % struct.unpack(">f", bytes.fromhex(v[4:12]))[0]
        except Exception:
            return v
    return v

def parse(line):
    f = line.split(",")
    p = {"idx": int(f[0]), "dev": f[1], "unit": int(f[2]), "point": f[3], "val": decode_val(f[4])}
    for kv in f[5:]:
        if "=" in kv:
            k, v = kv.split("=", 1)
            p[k] = v
    return p

class Dump:
    def __init__(self):
        self.s = None
        self.n = 0
    def _connect(self):
        self.s = socket.create_connection((MCU, DUMP_PORT), timeout=2)
        self.s.settimeout(2)
        self.n = recv_exact(self.s, 1)[0]
    def read(self):
        for attempt in range(2):
            try:
                if self.s is None:
                    self._connect()
                pts = []
                for d in range(self.n):
                    self.s.sendall(bytes([d]))
                    m = int.from_bytes(recv_exact(self.s, 4), "big")
                    txt = recv_exact(self.s, m).decode("utf-8", "replace") if m else ""
                    for ln in txt.splitlines():
                        if ln.strip():
                            pts.append(parse(ln))
                return pts
            except Exception:
                try:
                    self.s.close()
                except Exception:
                    pass
                self.s = None
                if attempt == 1:
                    raise

class Mb:
    def __init__(self):
        self.s = None
        self.tid = 0
    def write(self, unit, reg, regs):
        try:
            if self.s is None:
                self.s = socket.create_connection((MCU, MB_PORT), timeout=2)
                self.s.settimeout(2)
            self.tid = (self.tid + 1) & 0xFFFF
            qty = len(regs)
            pdu = struct.pack(">BHHB", 0x10, reg, qty, qty * 2) + b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
            mbap = struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, unit)
            self.s.sendall(mbap + pdu)
            resp = self.s.recv(256)
            return len(resp) >= 9 and not (resp[7] & 0x80)
        except Exception:
            try:
                self.s.close()
            except Exception:
                pass
            self.s = None
            raise

DUMP = Dump()
MB = Mb()

def encode(kind, dec, value):
    if kind == 4:
        b = struct.pack(">f", float(value))
        return [int.from_bytes(b[0:2], "big"), int.from_bytes(b[2:4], "big")]
    if kind == 3:
        sc = int(round(float(value) * 100))
        return [((sc // 100) >> 16) & 0xFFFF, (sc // 100) & 0xFFFF, sc % 100]
    sc = int(round(float(value) * (10 ** dec)))
    if kind in (0, 1):
        return [sc & 0xFFFF]
    if kind == 2:
        return [(sc >> 16) & 0xFFFF, sc & 0xFFFF]
    return []

class _Html(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        data = PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
    def log_message(self, *a):
        pass

def start_http():
    http.server.HTTPServer(("0.0.0.0", HTTP_PORT), _Html).serve_forever()

CLIENTS = set()
LATEST = []

async def handler(ws):
    print("== client connected")
    CLIENTS.add(ws)
    try:
        if LATEST:
            await ws.send(json.dumps(LATEST[0]))
        async for msg in ws:
            c = json.loads(msg)
            if c.get("type") == "write":
                loop = asyncio.get_event_loop()
                try:
                    ok = await loop.run_in_executor(None, lambda: MB.write(
                        int(c["unit"]), int(c["reg"]),
                        encode(int(c["k"]), int(c["d"]), c["value"])))
                    print("== WRITE unit=%s reg=%s value=%s -> %s" % (c["unit"], c["reg"], c["value"], ok))
                except Exception as e:
                    print("== WRITE ERR:", e)
    finally:
        CLIENTS.discard(ws)
        print("== client gone")

async def reader():
    loop = asyncio.get_event_loop()
    while True:
        try:
            pts = await loop.run_in_executor(None, DUMP.read)
            LATEST[:] = [pts]
            msg = json.dumps(pts)
            for ws in list(CLIENTS):
                try:
                    await ws.send(msg)
                except Exception:
                    pass
        except Exception as e:
            print("!! read err:", e)
        await asyncio.sleep(POLL_MS / 1000)

async def main():
    threading.Thread(target=start_http, daemon=True).start()
    print("网页打开:  http://127.0.0.1:%d" % HTTP_PORT)
    print("WebSocket: ws://0.0.0.0:%d  (长连接 读 %s:%d 写 %s:%d)" % (WS_PORT, MCU, DUMP_PORT, MCU, MB_PORT))
    async with websockets.serve(handler, "0.0.0.0", WS_PORT):
        await reader()

asyncio.run(main())
