#!/usr/bin/env python3
# Low-Level Buzzer Treiber: Hiwonder SDK, Demo-Modul oder GPIO (RPi.GPIO oder libgpiod)

import time

class BuzzerDriver:
    """
    backend: "auto" | "sdk" | "demo" | "gpio"
    gpio_pin: BCM-Pin (z.B. 6) falls backend="gpio"
    gpio_active_high: True=aktiv HIGH, False=aktiv LOW
    pwm_hz: 0 = kein PWM (aktiver Buzzer), >0 = PWM-Frequenz (passiver Buzzer, nur mit RPi.GPIO)
    """
    def __init__(self, backend="auto", gpio_pin=None, gpio_active_high=True, pwm_hz=0):
        self.backend = backend
        self.gpio_pin = gpio_pin
        self.gpio_active_high = bool(gpio_active_high)
        self.pwm_hz = int(pwm_hz)

        self.mode = None
        self.ok = False

        # SDK / Demo
        self._sdk_obj = None
        self._demo = None

        # GPIO (RPi.GPIO)
        self._GPIO = None
        self._pwm = None
        self._pwm_started = False

        # gpiod (libgpiod v1)
        self._gpiod = None
        self._gpiod_chip = None
        self._gpiod_line = None

        # Backends der Reihe nach probieren
        if self.backend in ("auto", "sdk"):
            self._try_init_sdk()
        if not self.ok and self.backend in ("auto", "demo"):
            self._try_init_demo()
        if not self.ok and self.backend in ("auto", "gpio", "GPIO", "Gpio"):
            # Erst RPi.GPIO, dann gpiod – beides "gpio"-Backend
            self._try_init_rpigpio()
            if not self.ok:
                self._try_init_gpiod()

        if not self.ok:
            print("[BuzzerDriver] Kein passendes Backend gefunden.")

    # ---------- Hiwonder SDK ----------
    def _try_init_sdk(self):
        sdk = None
        try:
            import ros_robot_controller_sdk as _sdk
            sdk = _sdk
        except Exception:
            try:
                from low_level import ros_robot_controller_sdk as _sdk
                sdk = _sdk
            except Exception:
                sdk = None

        if not sdk:
            return

        obj = None
        try:
            if hasattr(sdk, "Board"):
                obj = sdk.Board()
                self.mode = "sdk_board"
            elif hasattr(sdk, "Robot"):
                obj = sdk.Robot()
                self.mode = "sdk_robot"
        except Exception:
            obj = None

        if not obj:
            return

        names = dir(obj)
        has_buzz = any(("buzz" in n.lower() or "beep" in n.lower()) for n in names)
        if not has_buzz:
            return

        self._sdk_obj = obj
        self.ok = True
        print(f"[BuzzerDriver] SDK aktiv: {self.mode}")

    # ---------- Demo-Modul ----------
    def _try_init_demo(self):
        try:
            import buzzer_control_demo as bcd
        except Exception:
            return
        names = dir(bcd)
        has_on = any(k in names for k in ("buzzer_on", "beep_on", "on"))
        has_off = any(k in names for k in ("buzzer_off", "beep_off", "off"))
        if not (has_on and has_off):
            return
        self._demo = bcd
        self.mode = "demo_funcs"
        self.ok = True
        print("[BuzzerDriver] Demo-Backend aktiv (buzzer_control_demo.py)")

    # ---------- GPIO via RPi.GPIO ----------
    def _try_init_rpigpio(self):
        try:
            import RPi.GPIO as GPIO
        except Exception:
            return  # kein RPi.GPIO – versuchen wir gpiod später

        if self.gpio_pin is None:
            print("[BuzzerDriver] GPIO-Pin nicht gesetzt (--buzzer_pin).")
            return

        try:
            self._GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)

            if self.pwm_hz > 0:
                self._pwm = GPIO.PWM(self.gpio_pin, self.pwm_hz)
                self._pwm_started = False
                self.mode = "gpio_pwm"
            else:
                self.mode = "gpio"

            self.ok = True
            print(f"[BuzzerDriver] GPIO (RPi.GPIO) aktiv: BCM pin={self.gpio_pin} active={'HIGH' if self.gpio_active_high else 'LOW'} pwm_hz={self.pwm_hz}")
        except Exception as e:
            print("[BuzzerDriver] RPi.GPIO init Fehler:", e)
            self._GPIO = None
            self._pwm = None

    # ---------- GPIO via libgpiod (v1 API) ----------
    def _try_init_gpiod(self):
        try:
            import gpiod  # python3-libgpiod
        except Exception as e:
            print("[BuzzerDriver] libgpiod nicht verfügbar:", e)
            return

        if self.gpio_pin is None:
            print("[BuzzerDriver] gpiod: GPIO-Pin nicht gesetzt (--buzzer_pin).")
            return

        try:
            chipname = "gpiochip0"
            chip = gpiod.Chip(chipname)
            line = chip.get_line(int(self.gpio_pin))
            # Standard: Ausgang, Startpegel = inaktiv
            default_val = 1 if (False == self.gpio_active_high) else 0
            line.request(consumer="buzzer", type=gpiod.LINE_REQ_DIR_OUT, default_val=default_val)
            self._gpiod = gpiod
            self._gpiod_chip = chip
            self._gpiod_line = line
            self.mode = "gpiod"
            self.ok = True
            print(f"[BuzzerDriver] GPIO (gpiod) aktiv: line={self.gpio_pin} active={'HIGH' if self.gpio_active_high else 'LOW'}")
        except Exception as e:
            print("[BuzzerDriver] gpiod init Fehler:", e)
            self._gpiod = None
            self._gpiod_chip = None
            self._gpiod_line = None

    # ---------- SDK/Demo Helfer ----------
    @staticmethod
    def _call_if(obj, name, *args, **kw):
        if hasattr(obj, name):
            try:
                return getattr(obj, name)(*args, **kw)
            except Exception:
                pass
        return None

    def _sdk_on(self):
        o = self._sdk_obj
        return (
            self._call_if(o, "setBuzzer", 1) or
            self._call_if(o, "buzzerOn") or
            self._call_if(o, "beep", 1) or
            self._call_if(o, "set_beep", 1)
        )

    def _sdk_off(self):
        o = self._sdk_obj
        return (
            self._call_if(o, "setBuzzer", 0) or
            self._call_if(o, "buzzerOff") or
            self._call_if(o, "beep", 0) or
            self._call_if(o, "set_beep", 0)
        )

    def _demo_on(self):
        for n in ("buzzer_on", "beep_on", "on"):
            if self._call_if(self._demo, n) is not None:
                return

    def _demo_off(self):
        for n in ("buzzer_off", "beep_off", "off"):
            if self._call_if(self._demo, n) is not None:
                return

    # ---------- GPIO Set ----------
    def _gpio_set(self, state: bool):
        # RPi.GPIO
        if self._GPIO:
            if self._pwm:
                try:
                    if state and not self._pwm_started:
                        self._pwm.start(50.0)   # 50% Duty
                        self._pwm_started = True
                    elif not state and self._pwm_started:
                        self._pwm.stop()
                        self._pwm_started = False
                except Exception:
                    pass
            else:
                try:
                    level = 1 if (state == self.gpio_active_high) else 0
                    self._GPIO.output(self.gpio_pin, level)
                except Exception:
                    pass
            return

        # gpiod
        if self._gpiod_line:
            try:
                val = 1 if (state == self.gpio_active_high) else 0
                self._gpiod_line.set_value(val)
            except Exception:
                pass

    # ---------- Public API ----------
    def on(self):
        if not self.ok:
            return
        if self.mode and self.mode.startswith("sdk"):
            self._sdk_on()
        elif self.mode == "demo_funcs":
            self._demo_on()
        else:
            self._gpio_set(True)

    def off(self):
        if not self.ok:
            return
        if self.mode and self.mode.startswith("sdk"):
            self._sdk_off()
        elif self.mode == "demo_funcs":
            self._demo_off()
        else:
            self._gpio_set(False)

    def beep(self, seconds=0.2):
        self.on()
        time.sleep(max(0.0, float(seconds)))
        self.off()

    def close(self):
        # Clean up
        try:
            if self._pwm and self._pwm_started:
                self._pwm.stop()
            if self._GPIO:
                self._GPIO.cleanup(self.gpio_pin if self.gpio_pin is not None else None)
        except Exception:
            pass
        try:
            if self._gpiod_line:
                self._gpiod_line.release()
            if self._gpiod_chip:
                self._gpiod_chip.close()
        except Exception:
            pass