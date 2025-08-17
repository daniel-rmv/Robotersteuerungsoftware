#!/usr/bin/env python3
# High-Level Navigation – nutzt MotorSystem + LiDARSystem
import time, json, os

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
RESET  = "\033[0m"

def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

class NavigationSystem:
    def __init__(self, motor_sys, lidar_sys,
                 left_target_mm=300.0, front_stop_mm=150.0, forward_mm_s=60.0,
                 left_mode="single", front_mode="single",
                 # Kalibrierung / Drehprofil:
                 calib_file=".turn_calib.json",
                 yaw_fast=16, yaw_slow=6, ratio_fast=0.80,
                 brake_opp=8, brake_time=0.06):
        """
        left_mode/front_mode: "single" (exakter Strahl) oder "window"
        Die Drehung nutzt .turn_calib.json, wenn vorhanden (deg/s & ggf. brake_deg).
        """
        self.motors = motor_sys
        self.lidar  = lidar_sys

        self.left_target = float(left_target_mm)
        self.front_stop  = float(front_stop_mm)
        self.forward     = float(forward_mm_s)
        self.left_mode   = left_mode
        self.front_mode  = front_mode

        # Regler (300 mm exakt)
        self.Kp_err     = 0.08
        self.Kp_orient  = 0.06
        self.MAX_YAW    = 20
        self.MIN_YAW    = 2
        self.TOL_MM     = 5.0

        # Safety
        self.GUARD_EXTRA = 80.0
        self.MIN_FWD     = 0.3
        self.MAX_SLOWERR = 250.0

        # Drehung – Standardprofil, wird durch JSON überschrieben falls vorhanden
        self.calib_file  = calib_file
        self.yaw_fast    = int(yaw_fast)
        self.yaw_slow    = int(yaw_slow)
        self.ratio_fast  = float(ratio_fast)
        self.brake_opp   = int(brake_opp)
        self.brake_time  = float(brake_time)

    # ---------- LiDAR Helpers ----------
    def _read_left(self):
        if self.left_mode == "single":
            return self.lidar.left_distance_exact()
        else:
            return self.lidar.left_distance_mm(span_deg=10.0)

    def _read_right(self):
        if self.left_mode == "single":
            return self.lidar.right_distance_exact()
        else:
            return self.lidar.right_distance_mm(span_deg=10.0)

    def _read_front(self):
        if self.front_mode == "single":
            return self.lidar.front_distance_exact()
        else:
            return self.lidar.front_distance_mm(span_deg=10.0)

    # Orientierung (kleine Winkelkorrektur)
    def _yaw_from_orientation_left(self):
        db = self.lidar.left_back_mm(span_deg=6.0)   # 168°
        df = self.lidar.left_front_mm(span_deg=6.0)  # 192°
        if db is None or df is None: return 0
        diff = df - db
        yaw = int(diff * self.Kp_orient)  # vorne weiter weg -> links drehen (+)
        return _clamp(yaw, -self.MAX_YAW, self.MAX_YAW)

    def _yaw_from_orientation_right(self):
        db = self.lidar.right_back_mm(span_deg=6.0)   # 12°
        df = self.lidar.right_front_mm(span_deg=6.0)  # 348°
        if db is None or df is None: return 0
        diff = df - db
        yaw = -int(diff * self.Kp_orient)  # Spiegelung
        return _clamp(yaw, -self.MAX_YAW, self.MAX_YAW)

    # Distanz-Fehler zu Yaw
    def _yaw_from_error_left(self, err_mm):
        if abs(err_mm) <= self.TOL_MM: return 0
        mag = int(abs(err_mm) * self.Kp_err)
        mag = _clamp(mag, self.MIN_YAW, self.MAX_YAW)
        return +mag if err_mm > 0 else -mag

    def _yaw_from_error_right(self, err_mm):
        if abs(err_mm) <= self.TOL_MM: return 0
        mag = int(abs(err_mm) * self.Kp_err)
        mag = _clamp(mag, self.MIN_YAW, self.MAX_YAW)
        return -mag if err_mm > 0 else +mag

    # Speed-Reduktion bei großem Fehler
    def _fwd_scale_from_error(self, err_mm):
        e = min(abs(err_mm), self.MAX_SLOWERR)
        drop = 0.7 * (e / self.MAX_SLOWERR)
        return max(self.MIN_FWD, 1.0 - drop)

    # ---------- Folgen LINKS bis STOP ----------
    def _follow_left_until_stop(self):
        print(YELLOW + "[MODE] follow LEFT wall then STOP at front" + RESET)
        while True:
            df = self._read_front()
            dl = self._read_left()
            if df is None or dl is None:
                print(RED + "[WARN] LiDAR liefert keine Werte"); time.sleep(0.05); continue

            print(BLUE + f"[SENSOR] left={dl:.0f}mm  front={df:.0f}mm" + RESET)

            if df <= self.front_stop:
                print(RED + f"[STOP] front {df:.0f}mm <= {self.front_stop:.0f}mm" + RESET)
                self.motors.stop()
                return

            err = dl - self.left_target
            yaw = _clamp(self._yaw_from_error_left(err) + self._yaw_from_orientation_left(),
                         -self.MAX_YAW, self.MAX_YAW)

            # harte Guard: nicht in Wand ziehen
            if dl < (self.left_target - self.GUARD_EXTRA):
                force = max(self.MIN_YAW, int((self.left_target - dl) * 0.1))
                yaw = -abs(force)

            fwd = self.forward * self._fwd_scale_from_error(err)
            if yaw == 0:
                print(GREEN + f"[GO] v={fwd:.0f} mm/s" + RESET)
            else:
                print(YELLOW + f"[ADJUST] yaw {yaw:+d}, v={fwd:.0f}" + RESET)
            self.motors.drive(forward_mm_s=fwd, yaw_pulses=yaw)
            time.sleep(0.05)

    # ---------- Folgen RECHTS bis STOP ----------
    def _follow_right_until_stop(self):
        print(YELLOW + "[MODE] follow RIGHT wall then STOP at front" + RESET)
        while True:
            df = self._read_front()
            dr = self._read_right()
            if df is None or dr is None:
                print(RED + "[WARN] LiDAR liefert keine Werte"); time.sleep(0.05); continue

            print(BLUE + f"[SENSOR] right={dr:.0f}mm  front={df:.0f}mm" + RESET)

            if df <= self.front_stop:
                print(RED + f"[STOP] front {df:.0f}mm <= {self.front_stop:.0f}mm" + RESET)
                self.motors.stop()
                return

            err = dr - self.left_target  # gleicher target Wert, aber rechte Seite
            yaw = _clamp(self._yaw_from_error_right(err) + self._yaw_from_orientation_right(),
                         -self.MAX_YAW, self.MAX_YAW)

            if dr < (self.left_target - self.GUARD_EXTRA):
                force = max(self.MIN_YAW, int((self.left_target - dr) * 0.1))
                yaw = +abs(force)  # von rechts weg -> links drehen (+)

            fwd = self.forward * self._fwd_scale_from_error(err)
            if yaw == 0:
                print(GREEN + f"[GO] v={fwd:.0f} mm/s" + RESET)
            else:
                print(YELLOW + f"[ADJUST] yaw {yaw:+d}, v={fwd:.0f}" + RESET)
            self.motors.drive(forward_mm_s=fwd, yaw_pulses=yaw)
            time.sleep(0.05)

    # ---------- kalibrierte 180°-Links-Drehung (2 Stufen + optional Brake) ----------
    def _load_turn_json(self):
        if not self.calib_file or not os.path.exists(self.calib_file):
            return None
        try:
            with open(self.calib_file, "r") as f:
                return json.load(f)
        except Exception:
            return None

    def rotate_left_180_calibrated(self, target_deg=180.0):
        """
        Nutzt .turn_calib.json falls vorhanden:
          - dps für yaw_fast und yaw_slow
          - ggf. gespeicherte Brake-Kompensation
        Sonst fallback: einfache Zeiten aus ratio_fast und Standard-dps (grobe Defaults).
        """
        data = self._load_turn_json()
        dps_fast = dps_slow = None
        brake_deg = 0.0

        if data and "deg_per_s" in data and "left" in data["deg_per_s"]:
            dps_fast = float(data["deg_per_s"]["left"].get(str(abs(self.yaw_fast)), 0.0) or 0.0)
            dps_slow = float(data["deg_per_s"]["left"].get(str(abs(self.yaw_slow)), 0.0) or 0.0)
        if data and "brake" in data and "left" in data["brake"]:
            # Key wie "opp8_t0.06"
            bkeys = list(data["brake"]["left"].keys())
            for k in bkeys:
                try:
                    if f"opp{abs(self.brake_opp)}" in k and f"t{self.brake_time:.2f}" in k:
                        brake_deg = float(data["brake"]["left"][k])
                        break
                except Exception:
                    pass

        # Fallbacks, falls nicht gemessen:
        if not dps_fast or dps_fast <= 0:
            dps_fast = 25.0  # konservativer Default, bitte messen
        if not dps_slow or dps_slow <= 0:
            dps_slow = 9.0   # konservativer Default, bitte messen

        # Ziel inkl. Bremskompensation (dein letzter Brake-Test = 0°, also neutral)
        target_eff = float(target_deg) + float(brake_deg)

        fast_deg = target_eff * self.ratio_fast
        slow_deg = target_eff - fast_deg
        t_fast = fast_deg / dps_fast
        t_slow = slow_deg / dps_slow

        print(YELLOW + f"[TURN-EXACT] target={target_deg:.1f}°, brake_comp={brake_deg:.1f}° "
              f"(eff={target_eff:.1f}°), ratio_fast={self.ratio_fast:.2f}" + RESET)
        print(f"   FAST: yaw={self.yaw_fast}  dps≈{dps_fast:.1f}  t≈{t_fast:.3f}s  (≈{fast_deg:.1f}°)")
        print(f"   SLOW: yaw={self.yaw_slow}  dps≈{dps_slow:.1f}  t≈{t_slow:.3f}s  (≈{slow_deg:.1f}°)")
        print(f"   BRAKE: opp={self.brake_opp} t={self.brake_time:.2f}s")

        # Phase FAST
        t_end = time.time() + t_fast
        while time.time() < t_end:
            self.motors.yaw_spin(+abs(self.yaw_fast))
            time.sleep(0.01)

        # Phase SLOW
        t_end = time.time() + t_slow
        while time.time() < t_end:
            self.motors.yaw_spin(+abs(self.yaw_slow))
            time.sleep(0.01)

        # Gegenbremse
        if self.brake_opp != 0 and self.brake_time > 0:
            self.motors.brake_yaw(opposite_pulses=+abs(self.brake_opp), duration=self.brake_time)
        self.motors.stop()

    # ---------- Öffentliche Sequenz ----------
    def straight_leftwall_stop(self):
        """
        1) Links-Wand folgen bis Front-Stop.
        2) 180° LINKS drehen (kalibriert).
        3) Rechts-Wand folgen bis Front-Stop.
        """
        self._follow_left_until_stop()
        self.rotate_left_180_calibrated(target_deg=180.0)
        self._follow_right_until_stop()