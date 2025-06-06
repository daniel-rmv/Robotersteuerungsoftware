import time
from MotorController import MotorController

class NavigationSystem:
    def __init__(self, motor_controller: MotorController):
        self.mc = motor_controller

    def go_forward(self, speed_mm_s: float):
        self.mc.drive_forward(speed_mm_s)

    def go_backward(self, speed_mm_s: float):
        self.mc.drive_backward(speed_mm_s)

    def strafe_right(self, speed_mm_s: float):
        self.mc.strafe_right(speed_mm_s)

    def strafe_left(self, speed_mm_s: float):
        self.mc.strafe_left(speed_mm_s)

    def rotate(self, angular_rate_rad_s: float):
        self.mc.rotate(angular_rate_rad_s)

    def stop(self):
        self.mc.stop_all()


# Test Lauf
if __name__ == "__main__":
    mc = MotorController(i2c_port=1)
    nav = NavigationSystem(mc)

    try:
        nav.strafe_right(50)
        time.sleep(2)
        nav.stop()
        time.sleep(1)
        nav.rotate(0.4)
        time.sleep(2)
        nav.stop()
    except KeyboardInterrupt:
        nav.stop()
        print("Abgebrochen, Motoren gestoppt.")
