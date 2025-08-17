#!/usr/bin/env python3
# High-Level Motor-Service – wrappt low_level.MotorController
from low_level.MotorController import MotorController
import time

class MotorSystem:
    def __init__(self, ramp_step=3, g_vy=0.25):
        self._ll = MotorController(ramp_step=ramp_step, g_vy=g_vy)

    # --------- Driving APIs ----------
    def drive(self, forward_mm_s: float = 0.0, yaw_pulses: int = 0):
        """Vorwärts + Yaw (kein Strafe)."""
        base = self._ll.base_mag_from_speed(forward_mm_s)
        self._ll.command(base_fwd=base, strafe=0, yaw=int(yaw_pulses))

    def drive_full(self, forward_mm_s: float = 0.0, strafe_pulses: int = 0, yaw_pulses: int = 0):
        """Volle Kontrolle: Vortrieb, Strafe, Yaw gleichzeitig."""
        base = self._ll.base_mag_from_speed(forward_mm_s)
        self._ll.command(base_fwd=base, strafe=int(strafe_pulses), yaw=int(yaw_pulses))

    def drive_forward(self, speed_mm_s: float):
        self.drive(forward_mm_s=speed_mm_s, yaw_pulses=0)

    def rotate_left(self, yaw_pulses=12):
        self._ll.command(base_fwd=0, strafe=0, yaw=+abs(int(yaw_pulses)))

    def rotate_right(self, yaw_pulses=12):
        self._ll.command(base_fwd=0, strafe=0, yaw=-abs(int(yaw_pulses)))

    def strafe_left(self, mag: int):
        """Seitwärts nach links. Falls invertiert, tausche Vorzeichen hier."""
        self._ll.command(base_fwd=0, strafe=+abs(int(mag)), yaw=0)

    def strafe_right(self, mag: int):
        """Seitwärts nach rechts."""
        self._ll.command(base_fwd=0, strafe=-abs(int(mag)), yaw=0)

    # --- für kalibrierte Drehung / Bremse ---
    def yaw_spin(self, yaw_pulses: int):
        """Nur Yaw drehen (kein Vortrieb). Positiv = links, Negativ = rechts."""
        self._ll.command(base_fwd=0, strafe=0, yaw=int(yaw_pulses))

    def brake_yaw(self, opposite_pulses=8, duration=0.06):
        """Kurzer Gegenimpuls, um Restträgheit abzufangen."""
        if opposite_pulses > 0:
            self.rotate_right(opposite_pulses)
        elif opposite_pulses < 0:
            self.rotate_left(abs(opposite_pulses))
        time.sleep(max(0.0, float(duration)))
        self.stop()

    def stop(self):
        self._ll.stop_all()
        time.sleep(0.02)