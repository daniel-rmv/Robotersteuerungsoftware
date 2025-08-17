#!/usr/bin/env python3
# High-Level Buzzer API – dünner Wrapper um low_level.BuzzerDriver

import time
from low_level.BuzzerDriver import BuzzerDriver

class BuzzerSystem:
    def __init__(self, backend="auto", gpio_pin=None, gpio_active="high", pwm_hz=0):
        self._drv = BuzzerDriver(
            backend=backend,
            gpio_pin=gpio_pin,
            gpio_active_high=(str(gpio_active).lower() != "low"),
            pwm_hz=pwm_hz
        )

    def on(self):
        self._drv.on()

    def off(self):
        self._drv.off()

    def beep(self, seconds=0.2):
        self._drv.beep(seconds)

    def pattern(self, on_ms=120, off_ms=120, times=3):
        on_s = max(0.0, on_ms / 1000.0)
        off_s = max(0.0, off_ms / 1000.0)
        for _ in range(int(times)):
            self.on();  time.sleep(on_s)
            self.off(); time.sleep(off_s)

    def close(self):
        self._drv.close()