#!/usr/bin/env python
import math
from utils.motor.motor_api import Motor

# Grouping motors left and right depending on ID
class MotorGroup:
    def __init__(self, left, right, track_width_m, wheel_radius_m):
        # Left and right
        self.left = left
        self.right = right

        # Required for twist math
        self.track_width = track_width_m
        self.wheel_radius = wheel_radius_m

    # Loops all motors in left and sets RPM
    def set_left(self, rpm):
        for m in self.left:
            m.set_rpm(rpm)

    # Same but right
    def set_right(self, rpm):
        for m in self.right:
            m.set_rpm(rpm)

    # Using Skidsteer v_y and v_z are never used, as well as w_x and w_y
    def twist_to_rpm(self, v_x: float, w_z: float):
        # Wheel linear velocity (m/s)
        v_left  = float(v_x) - float(w_z) * (self.track_width / 2.0)    # v_x : forward linear velocity  (m/s)
        v_right = float(v_x) + float(w_z) * (self.track_width / 2.0)    # w_z : angular velocity         (rad/s)

        # Convert m/s to RPM
        circ = 2.0 * math.pi * self.wheel_radius

        # Converts wheel linear velocity to wheel rotational speed in RPM
        left_rpm  = (v_left / circ) * 60.0 * 100
        right_rpm = (v_right / circ) * 60.0 * 100

        return [float(left_rpm), float(right_rpm)]
    
    # apply twist vector to all left wheels and right wheels
    def apply_twist(self, v_x: float, w_z: float):
        [left_rpm, right_rpm] = self.twist_to_rpm(v_x, w_z)
        self.set_left(left_rpm)
        self.set_right(right_rpm)
        print("applyTwist: ", left_rpm)

# Skid-steer controller function
class SkidSteerController:
    def __init__(self, group, max_rpm, turn_strength):
        self.group = group
        self.max_rpm = max_rpm
        self.turn_strength = turn_strength

        # Initialize and set to 0
        self.throttle = 1.0
        self.steering = 0.0

        # Physics-based motor simulation
        self.current_left_rpm = 0.0
        self.current_right_rpm = 0.0
