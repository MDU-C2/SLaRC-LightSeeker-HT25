#!/usr/bin/env python
import math

class MotorTelemetry:
    def __init__(self):
        self.rpm = 0
        self.current_a = 0.0
        self.angle_deg = 0.0
        self.temperature_c = 0
        self.error_code = 0

        self.status = 0
        self.flags = 0

        self.gear_ratio = 9
        self.wheel_diameter_m = 0.35

        self.wheel_rpm = 0.0
        self.wheel_rad_s = 0.0
        self.linear_m_s = 0.0

    def update_from_frame(self, data):

        pos_raw = (data[0] << 8) | data[1]
        if pos_raw & 0x8000:
            pos_raw -= 0x10000
        raw_deg = pos_raw * 0.1
        self.angle_deg = raw_deg % 360.0

        rpm_raw = (data[2] << 8) | data[3]
        if rpm_raw & 0x8000:
            rpm_raw -= 0x10000
        self.rpm = rpm_raw

        cur_raw = (data[4] << 8) | data[5]
        if cur_raw & 0x8000:
            cur_raw -= 0x10000
        self.current_a = cur_raw * 0.01

        self.temperature_c = data[6]
        self.error_code = data[7]

        self.status = data[0]
        self.flags = data[1]

        try:
            self.wheel_rpm = float(self.rpm) / float(self.gear_ratio)
        except Exception:
            self.wheel_rpm = 0.0

        self.wheel_rad_s = (self.wheel_rpm * 2.0 * math.pi) / 60.0
        self.linear_m_s = self.wheel_rad_s * (self.wheel_diameter_m / 2.0)
