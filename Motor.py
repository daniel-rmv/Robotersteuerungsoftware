import smbus2

class Motor:
    def __init__(self, i2c_port: int, motor_id: int):
        self.i2c_port = i2c_port
        self.motor_id = motor_id  # 1 bis 4
        self.module_address = 0x34  # Adresse des Motor-Drivers

    def set_speed(self, speed: int):
        """
        Setzt die Geschwindigkeit des Motors.
        speed: int (z.B. Pulse pro 10 ms; negativ für Rückwärts)
        """
        if not (1 <= self.motor_id <= 4):
            raise ValueError(f"Ungültige motor_id: {self.motor_id}")
        register = 50 + self.motor_id  # Register 51..54
        try:
            with smbus2.SMBus(self.i2c_port) as bus:
                bus.write_i2c_block_data(self.module_address, register, [speed & 0xFF])
        except Exception as e:
            print(f"Fehler beim Setzen der Geschwindigkeit Motor {self.motor_id}: {e}")

    def stop(self):
        """Stoppt den Motor."""
        self.set_speed(0)
