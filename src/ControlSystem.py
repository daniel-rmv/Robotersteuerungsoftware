#!/usr/bin/env python3
# Main Runner / CLI – minimal: --remote oder --follow_wall
# Kompatibel: akzeptiert zusätzlich altes --mode {remote,follow_wall,beep}

import argparse, time, sys
import termios, tty, select

from high_level.MotorSystem import MotorSystem
from high_level.NavigationSystem import NavigationSystem
from low_level.LiDARReader import MS200Driver
from high_level.LiDARSystem import LiDARSystem
from high_level.BuzzerSystem import BuzzerSystem

# ---------- Remote-Parameter ----------
REMOTE_POLL_S            = 0.02      # Steuerloop ~50 Hz

# Baseline: genug groß für die erste Überbrückung
BASE_INITIAL_HOLD_S      = 0.60      # Start-Überbrückung (wird adaptiv kalibriert)
ADAPTIVE_MIN_S           = 0.25
ADAPTIVE_MAX_S           = 0.90
ADAPTIVE_MARGIN_S        = 0.06      # + Sicherheitsaufschlag auf gemessenen Repeat-Delay

# Nach Loslassen schnell abfallen
MOV_RELEASE_HOLD_S       = 0.08      # Bewegung (~80 ms)
BUZZER_RELEASE_HOLD_S    = 0.25      # Buzzer stabiler Dauerton

REMOTE_FWD_MM_S          = 60.0
REMOTE_YAW_PULSES        = 16
REMOTE_STRAFE_P          = 22

# Hardware-Richtungen (nur für Remote); +1 oder -1
REMOTE_YAW_SIGN          = +1
REMOTE_STRAFE_SIGN       = -1   # <<< N=links, M=rechts (so wie angezeigt)

def _clamp(x, a, b): return max(a, min(b, x))

def run_remote(motors: MotorSystem, buzzer: BuzzerSystem):
    """
    Tasten:
      w: vorwärts      s: rückwärts
      a: links drehen  d: rechts drehen
      n: strafe links  m: strafe rechts
      b: Buzzer (halten)
      SPACE: Stop      q: Quit
    """
    print("\n--- REMOTE MODE ---")
    print("W/S = vor/zurück   A/D = links/rechts drehen")
    print("N/M = strafe L/R   B = Buzzer (halten)   SPACE = Stop   Q = Quit")
    print(f"(v≈{REMOTE_FWD_MM_S:.0f} mm/s, yaw={REMOTE_YAW_PULSES}, strafe={REMOTE_STRAFE_P})\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)   # Ctrl+C bleibt nutzbar

    # --- Achs-Zustände für adaptive Startüberbrückung ---
    axes = {
        "fwd":  {"val":0.0, "dead":0.0, "ih":BASE_INITIAL_HOLD_S, "t0":0.0, "repeat_seen":False},
        "yaw":  {"val":0,   "dead":0.0, "ih":BASE_INITIAL_HOLD_S, "t0":0.0, "repeat_seen":False},
        "strf": {"val":0,   "dead":0.0, "ih":BASE_INITIAL_HOLD_S, "t0":0.0, "repeat_seen":False},
        "buzz": {"on":False,"dead":0.0, "ih":BASE_INITIAL_HOLD_S, "t0":0.0, "repeat_seen":False},
    }

    def _start_axis(ax_name, new_val, kind="mov"):
        ax = axes[ax_name]
        first = ( (ax_name!="buzz" and ax["val"]==0) or (ax_name=="buzz" and not ax["on"]) )
        now = time.time()
        if ax_name != "buzz":
            ax["val"] = new_val
        else:
            if not ax["on"]: buzzer.on()
            ax["on"] = True
        if first:
            ax["t0"] = now
            ax["repeat_seen"] = False
            ax["dead"] = now + ax["ih"]
        else:
            ax["dead"] = now + (BUZZER_RELEASE_HOLD_S if kind=="buzz" else MOV_RELEASE_HOLD_S)

    def _maybe_adapt(ax_name, dt):
        # dt = Zeit zwischen erstem Druck und erstem Repeat für diese Achse
        ax = axes[ax_name]
        new_ih = _clamp(dt + ADAPTIVE_MARGIN_S, ADAPTIVE_MIN_S, ADAPTIVE_MAX_S)
        ax["ih"] = 0.5*ax["ih"] + 0.5*new_ih

    def _bridge_gap(ax_name):
        # Falls der erste Repeat noch nicht kam, Deadline mini-weise verlängern (ohne Off-Impuls)
        ax = axes[ax_name]
        now = time.time()
        if not ax["repeat_seen"] and (now - ax["t0"]) < ADAPTIVE_MAX_S:
            ax["dead"] = now + 0.05  # 50 ms Brücke
            return True
        return False

    try:
        last_apply = 0.0
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], REMOTE_POLL_S)
            now = time.time()

            if rlist:
                c = sys.stdin.read(1).lower()
                if c == 'q':
                    break
                if c == ' ':
                    # harter Stop
                    axes["fwd"]["val"] = 0.0; axes["yaw"]["val"] = 0; axes["strf"]["val"] = 0
                    axes["fwd"]["dead"] = axes["yaw"]["dead"] = axes["strf"]["dead"] = 0.0
                    if axes["buzz"]["on"]: buzzer.off()
                    axes["buzz"]["on"] = False; axes["buzz"]["dead"] = 0.0
                    print("[STOP]")
                    continue

                # --- Bewegungen setzen ---
                if c == 'w':
                    _start_axis("fwd", +abs(REMOTE_FWD_MM_S), "mov")
                elif c == 's':
                    _start_axis("fwd", -abs(REMOTE_FWD_MM_S), "mov")
                elif c == 'a':
                    _start_axis("yaw", +abs(REMOTE_YAW_PULSES)*REMOTE_YAW_SIGN, "mov")
                elif c == 'd':
                    _start_axis("yaw", -abs(REMOTE_YAW_PULSES)*REMOTE_YAW_SIGN, "mov")
                elif c == 'n':
                    _start_axis("strf", +abs(REMOTE_STRAFE_P)*REMOTE_STRAFE_SIGN, "mov")
                elif c == 'm':
                    _start_axis("strf", -abs(REMOTE_STRAFE_P)*REMOTE_STRAFE_SIGN, "mov")
                elif c == 'b':
                    _start_axis("buzz", None, "buzz")

                # --- Adaptiv: ersten Repeat erkennen & kalibrieren ---
                if c in ('w','s'):
                    ax = axes["fwd"]
                    if ax["t0"]>0 and not ax["repeat_seen"]:
                        dt = now - ax["t0"]
                        if 0.10 < dt < 1.20:
                            ax["repeat_seen"] = True
                            _maybe_adapt("fwd", dt)
                elif c in ('a','d'):
                    ax = axes["yaw"]
                    if ax["t0"]>0 and not ax["repeat_seen"]:
                        dt = now - ax["t0"]
                        if 0.10 < dt < 1.20:
                            ax["repeat_seen"] = True
                            _maybe_adapt("yaw", dt)
                elif c in ('n','m'):
                    ax = axes["strf"]
                    if ax["t0"]>0 and not ax["repeat_seen"]:
                        dt = now - ax["t0"]
                        if 0.10 < dt < 1.20:
                            ax["repeat_seen"] = True
                            _maybe_adapt("strf", dt)

            # --- Deadlines prüfen; erste Repeat-Lücke überbrücken ---
            for name in ("fwd","yaw","strf"):
                ax = axes[name]
                if (ax["val"] != 0 or name=="fwd" and isinstance(ax["val"], float)) and ax["dead"]>0 and now > ax["dead"]:
                    if not _bridge_gap(name):
                        ax["val"] = 0 if name!="fwd" else 0.0
                        ax["dead"] = 0.0
            # Buzzer separat
            bz = axes["buzz"]
            if bz["on"] and now > bz["dead"]:
                buzzer.off(); bz["on"] = False; bz["dead"] = 0.0

            # --- zyklisch anwenden ---
            if (now - last_apply) >= REMOTE_POLL_S:
                motors.drive_full(
                    forward_mm_s = axes["fwd"]["val"],
                    strafe_pulses = axes["strf"]["val"],
                    yaw_pulses    = axes["yaw"]["val"]
                )
                last_apply = now

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            if axes["buzz"]["on"]: buzzer.off()
        except Exception:
            pass
        motors.stop()

def main():
    ap = argparse.ArgumentParser(description="Robot Control System (kurze CLI)")
    # Neue kurze Flags:
    ap.add_argument("--remote", action="store_true", help="Manuell per Tastatur steuern")
    ap.add_argument("--follow_wall", action="store_true", help="Autonom: Wand folgen & vorne stoppen/180°")
    # Alt-kompatibel:
    ap.add_argument("--mode", choices=["remote", "follow_wall", "beep"], default=None)

    # (Optional) Feineinstellungen – Standardwerte wie von dir zuletzt genutzt
    ap.add_argument("--left_target", type=float, default=180.0)
    ap.add_argument("--front_stop",  type=float, default=120.0)
    ap.add_argument("--forward",     type=float, default=60.0)
    ap.add_argument("--left_mode",   choices=["single","window"], default="single")
    ap.add_argument("--front_mode",  choices=["single","window"], default="single")
    ap.add_argument("--calib_file",  default=".turn_calib.json")
    ap.add_argument("--yaw_fast",    type=int, default=16)
    ap.add_argument("--yaw_slow",    type=int, default=6)
    ap.add_argument("--ratio_fast",  type=float, default=0.80)
    ap.add_argument("--brake_opp",   type=int, default=8)
    ap.add_argument("--brake_time",  type=float, default=0.06)

    args = ap.parse_args()

    # --- Modus auflösen (neue Flags > alt)
    mode = None
    if args.remote: mode = "remote"
    elif args.follow_wall: mode = "follow_wall"
    else: mode = args.mode

    if mode is None:
        print("Benutzung:\n  python3 ControlSystem.py --remote\n  python3 ControlSystem.py --follow_wall")
        return

    if mode == "remote":
        motors = MotorSystem()
        # Buzzer: GPIO, BCM6, active HIGH (dein verifiziertes Setup)
        buzzer = BuzzerSystem(backend="gpio", gpio_pin=6, gpio_active="high", pwm_hz=0)
        print("[Buzzer] Backend=GPIO  pin=BCM6  active=HIGH")
        try:
            run_remote(motors, buzzer)
        except KeyboardInterrupt:
            pass
        finally:
            motors.stop(); buzzer.off(); buzzer.close()
        return

    if mode == "follow_wall":
        motors = MotorSystem()
        drv   = MS200Driver()
        lidar = LiDARSystem(drv)
        # Buzzer auch hier verwenden
        buzzer = BuzzerSystem(backend="gpio", gpio_pin=6, gpio_active="high", pwm_hz=0)

        drv.start(); time.sleep(0.4)

        nav = NavigationSystem(
            motors, lidar,
            left_target_mm=args.left_target,
            front_stop_mm=args.front_stop,
            forward_mm_s=args.forward,
            left_mode=args.left_mode,
            front_mode=args.front_mode,
            calib_file=args.calib_file,
            yaw_fast=args.yaw_fast,
            yaw_slow=args.yaw_slow,
            ratio_fast=args.ratio_fast,
            brake_opp=args.brake_opp,
            brake_time=args.brake_time
        )

        print(f"[Start] FOLLOW_WALL  fwd={args.forward}  target={args.left_target}  stop={args.front_stop}")
        try:
            # --- 3s Start-Buzzer ---
            print("[BEEP] Start in 3s …")
            buzzer.on(); time.sleep(3.0); buzzer.off()

            # --- Fahren bis Front-Stop ---
            nav.straight_leftwall_stop()
            motors.stop(); time.sleep(0.15)

            # --- 180° links auf der Stelle ---
            print("[TURN] 180° left …")
            turned = False
            try:
                # bevorzugt kalibriert über NavigationSystem
                if hasattr(nav, "turn_deg_left"):
                    nav.turn_deg_left(180.0); turned = True
                elif hasattr(nav, "turn_left_180"):
                    nav.turn_left_180(); turned = True
                elif hasattr(nav, "turn"):
                    nav.turn(180.0, direction="left"); turned = True
            except Exception:
                turned = False

            if not turned:
                # Fallback: kurzer manueller Spin, basierend auf deinen yaw_fast-Parametern
                motors.drive_full(0.0, 0, +abs(args.yaw_fast))
                time.sleep(1.20)  # ggf. anpassen falls nötig
                motors.stop()

            motors.stop(); time.sleep(0.1)

            # --- 3s Abschluss-Buzzer ---
            print("[BEEP] Done.")
            buzzer.on(); time.sleep(3.0); buzzer.off()

        except KeyboardInterrupt:
            pass
        finally:
            motors.stop(); drv.stop()
            try:
                buzzer.off()
            except Exception:
                pass
            buzzer.close()
            print("Stopped.")
        return

    if mode == "beep":
        buzzer = BuzzerSystem(backend="gpio", gpio_pin=6, gpio_active="high", pwm_hz=0)
        print("[Beep-Test] 1s an …"); buzzer.on(); time.sleep(1.0); buzzer.off(); print("[Beep-Test] aus.")
        buzzer.close()
        return

if __name__ == "__main__":
    main()