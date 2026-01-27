#!/usr/bin/env python
import math

class SkidSteerOdometry:
    # Initialize
    def __init__(self, track_width_m):
        self.x = 0.0            # Pos in X
        self.y = 0.0            # Pos in Y
        self.heading = 0.0      # Direction in radians
        self.track_width = track_width_m    # wheel base / width between center points from the left wheel to right wheel

    def update(self, v_left, v_right, dt):
        # Linear velocity
        v = (v_left + v_right) / 2.0
        # Angular velocity
        w = (v_right - v_left) / self.track_width

        # Integrate movement over time dt
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        self.heading += w * dt

        # Normalize heading to interval [-pi, pi]
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
        return self.x, self.y, self.heading
