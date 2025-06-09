from Motor import Motor
import math

class MotorController:
    def __init__(self, i2c_port: int, a=110, b=97.5, wheel_diameter=96.5, pulse_per_cycle=44 * 178):
        self.i2c_port = i2c_port
        self.motors = [Motor(self.i2c_port, i + 1) for i in range(4)]
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.pulse_per_cycle = pulse_per_cycle

    def set_all_speeds(self, speeds):
        if len(speeds) != 4:
            raise ValueError("Speeds-Liste muss 4 Elemente enthalten.")
        for motor, speed in zip(self.motors, speeds):
            motor.set_speed(speed)

    def stop_all(self):
        for motor in self.motors:
            motor.stop()

    def speed_to_pulse(self, speed_mm_s: float) -> int:
        pulse = speed_mm_s / (math.pi * self.wheel_diameter) * self.pulse_per_cycle * 0.01
        return int(pulse)

    def compute_wheel_speeds(self, velocity: float, direction: float, angular_rate: float):
        rad = math.radians(direction - 90)
        vx = velocity * math.cos(rad)
        vy = velocity * math.sin(rad)
        sum_ab = self.a + self.b
        v1 = vy - vx - sum_ab * angular_rate
        v2 = vy + vx + sum_ab * angular_rate
        v3 = vy + vx - sum_ab * angular_rate
        v4 = vy - vx + sum_ab * angular_rate
        return [v1, v2, v3, v4]

    def drive(self, velocity_mm_s: float, direction_deg: float, angular_rate_rad_s: float):
        wheel_speeds_mm = self.compute_wheel_speeds(velocity_mm_s, direction_deg, angular_rate_rad_s)
        pulses = [self.speed_to_pulse(v) for v in wheel_speeds_mm]
        self.set_all_speeds(pulses)

    def drive_forward(self, speed_mm_s: float):
        self.drive(speed_mm_s, 90, 0)

    def drive_backward(self, speed_mm_s: float):
        self.drive(speed_mm_s, 270, 0)

    def strafe_right(self, speed_mm_s: float):
        self.drive(speed_mm_s, 0, 0)

    def strafe_left(self, speed_mm_s: float):
        self.drive(speed_mm_s, 180, 0)

    def rotate(self, angular_rate_rad_s: float):
        self.drive(0, 0, angular_rate_rad_s)
