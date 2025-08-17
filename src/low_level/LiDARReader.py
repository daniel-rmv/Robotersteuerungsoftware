#!/usr/bin/env python3
# MS200 LiDAR Treiber + einfacher HL-Wrapper (LidarHL)
import threading, serial, glob, os, time

class MS200Driver:
    def __init__(self, port=None, baud=230400, timeout=0.05, offset_mm=42.0):
        self.port = port or self._find_port()
        if not self.port:
            raise RuntimeError("Kein serieller Port für LiDAR gefunden.")
        self.baud = baud
        self.timeout = timeout
        self.offset_mm = float(offset_mm)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._points = []
        self._ser = None
        self._th = None

    def _find_port(self):
        for p in glob.glob("/dev/serial/by-id/*"):
            return os.path.realpath(p)
        for pat in ("/dev/ttyUSB*","/dev/ttyACM*","/dev/ttyAMA0","/dev/ttyS0"):
            hit = glob.glob(pat)
            if hit: return hit[0]
        return None

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n and not self._stop.is_set():
            ch = self._ser.read(n - len(buf))
            if not ch: return None
            buf.extend(ch)
        return bytes(buf)

    def _sync(self):
        while not self._stop.is_set():
            b = self._ser.read(1)
            if not b: return False
            if b and b[0] == 0x54:
                return True
        return False

    def _parse(self, rest: bytes):
        if not rest or len(rest) < 5: return None
        cnt = rest[0] & 0x1F
        if cnt == 0 or cnt > 40: return None
        sa = int.from_bytes(rest[3:5], "little")/100.0
        off = 5
        dist=[]; inten=[]
        need = off + cnt*3 + 2 + 2 + 1
        if len(rest) < need: return None
        for _ in range(cnt):
            d = int.from_bytes(rest[off:off+2], "little")
            i = rest[off+2]
            dist.append(d); inten.append(i); off += 3
        ea = int.from_bytes(rest[off:off+2], "little")/100.0
        if cnt > 1:
            diff = ea - sa
            if diff < -180: diff += 360
            elif diff > 180: diff -= 360
            step = diff/(cnt-1)
        else:
            step = 0.0
        pts=[]
        for i,(d) in enumerate(dist):
            if d == 0 or d > 12000: continue
            ang = (sa + i*step) % 360
            v = d - self.offset_mm
            if v <= 0: continue
            pts.append((ang, v, inten[i]))
        return pts

    def _run(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        try:
            # stabilisieren
            self._ser.dtr = False
            self._ser.rts = False
            time.sleep(0.05)
            self._ser.reset_input_buffer()

            tmp=[]; last=None
            while not self._stop.is_set():
                if not self._sync(): continue
                head = self._read_exact(5)
                if head is None or len(head) < 5:
                    continue
                cnt = head[0] & 0x1F
                body = self._read_exact(cnt*3 + 2 + 2 + 1)
                if body is None:
                    continue
                pkt = head + body
                pts = self._parse(pkt)
                if not pts:
                    continue
                for a,d,i in pts:
                    if last is not None and a < 10.0 and last > 350.0:
                        with self._lock:
                            self._points = tmp
                        tmp=[]
                    tmp.append((a,d,i)); last = a
        finally:
            try: self._ser.close()
            except: pass

    def start(self):
        if getattr(self,'_th',None) and self._th.is_alive(): return
        self._stop.clear()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()
        if getattr(self,'_th',None):
            self._th.join(timeout=1.0)

    def get_points(self):
        with self._lock:
            return list(self._points)

# -------- High-Level Winkelfunktionen (nur lesen, keine Hardwarelogik) -----
class LidarHL:
    """0°=rechts, 270°=vorne, 180°=links"""
    def __init__(self, drv: MS200Driver):
        self.drv = drv

    @staticmethod
    def _angdiff(a,b):
        return abs((a - b + 180) % 360 - 180)

    def distance_at(self, ang_deg, window=5.0, mode="median"):
        pts = self.drv.get_points()
        if not pts: return None
        vals = [d for (a,d,i) in pts if self._angdiff(a, ang_deg) <= window]
        if not vals: return None
        vals.sort()
        if mode == "min": return vals[0]
        n=len(vals); m=n//2
        return vals[m] if n%2 else 0.5*(vals[m-1] + vals[m])