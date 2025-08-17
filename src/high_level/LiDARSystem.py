#!/usr/bin/env python3
# High-Level LiDARSystem – nutzt low_level.MS200Driver und bietet bequeme Mess-Helper
from low_level.LiDARReader import MS200Driver
import math
import threading

def _angdiff(a, b):
    """kleinste Winkeldifferenz in Grad (−180..+180, Betrag)"""
    return abs((a - b + 180.0) % 360.0 - 180.0)

class LiDARSystem:
    """
    Kann mit existierendem Driver betrieben werden, oder erstellt selbst einen.
    Angles (Grad):
      - 0   = rechts
      - 180 = links
      - 270 = vorne
    """
    def __init__(self, drv: MS200Driver = None, offset_mm: float = 42.0):
        # Wenn ein Driver übergeben wurde -> verwenden, sonst neuen bauen
        self._drv = drv if drv is not None else MS200Driver(offset_mm=offset_mm)
        self._lock = threading.Lock()

    # ---------- Roh-Zugriff ----------
    def get_points(self):
        # Liste von (angle_deg, distance_mm, intensity)
        return self._drv.get_points()

    # ---------- Mess-Helfer ----------
    def _distance_nearest(self, target_deg: float, window_deg: float = 1.0):
        """
        'Exakt' einen Strahl nahe target_deg: wählt den Messpunkt mit der kleinsten Winkeldiff
        innerhalb eines Fensters (± window/2). Gibt Distanz in mm zurück oder None.
        """
        pts = self.get_points()
        if not pts:
            return None
        half = window_deg * 0.5
        best = None
        best_dang = None
        for a, d, i in pts:
            dang = _angdiff(a, target_deg)
            if dang <= half:
                if best is None or dang < best_dang:
                    best = d
                    best_dang = dang
        return best

    def _distance_window(self, target_deg: float, span_deg: float = 10.0, mode: str = "median"):
        """
        Fenster-Auswertung um target_deg (± span/2). mode: 'median' oder 'min'
        """
        pts = self.get_points()
        if not pts:
            return None
        half = span_deg * 0.5
        vals = [d for (a, d, i) in pts if _angdiff(a, target_deg) <= half]
        if not vals:
            return None
        vals.sort()
        if mode == "min":
            return vals[0]
        n = len(vals)
        m = n // 2
        return vals[m] if (n % 2 == 1) else 0.5 * (vals[m - 1] + vals[m])

    # ---------- „exakte“ Einzelstrahlen (du wolltest 270 / 180 / 0 exakt) ----------
    def front_distance_exact(self):
        return self._distance_nearest(270.0, window_deg=1.0)

    def left_distance_exact(self):
        return self._distance_nearest(180.0, window_deg=1.0)

    def right_distance_exact(self):
        return self._distance_nearest(0.0, window_deg=1.0)

    # ---------- Fenster-Varianten (falls du 'window' Mode nutzt) ----------
    def front_distance_mm(self, span_deg=10.0, mode="median"):
        return self._distance_window(270.0, span_deg=span_deg, mode=mode)

    def left_distance_mm(self, span_deg=10.0, mode="median"):
        return self._distance_window(180.0, span_deg=span_deg, mode=mode)

    def right_distance_mm(self, span_deg=10.0, mode="median"):
        return self._distance_window(0.0, span_deg=span_deg, mode=mode)

    # ---------- Orientierung (für Wand-Parallelität) ----------
    # Links-seitig: 168° (hinten), 192° (vorne) – kleine Fenster, 'min' ist robust gegen Ausreißer
    def left_back_mm(self, span_deg=6.0):
        return self._distance_window(168.0, span_deg=span_deg, mode="min")

    def left_front_mm(self, span_deg=6.0):
        return self._distance_window(192.0, span_deg=span_deg, mode="min")

    # Rechts-seitig: 12° (hinten), 348° (vorne)
    def right_back_mm(self, span_deg=6.0):
        return self._distance_window(12.0, span_deg=span_deg, mode="min")

    def right_front_mm(self, span_deg=6.0):
        return self._distance_window(348.0, span_deg=span_deg, mode="min")