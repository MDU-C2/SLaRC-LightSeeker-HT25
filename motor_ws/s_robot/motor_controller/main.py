#!/usr/bin/env python
import time
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from utils.motor.can_bus import create_bus
from utils.motor.motor_api import Motor
from utils.motor.motor_group import MotorGroup
from utils.motor.motor_odometry import SkidSteerOdometry
from utils.motor.motors import LEFT_MOTORS, RIGHT_MOTORS

from utils.battery.can_bus_battery import create_battery_bus
from utils.battery.communication_handler import Battery

# Safely bring down CAN Bus
def bring_down_can(can_id):
    try:
        subprocess.call(["sudo", "ip", "link", "set", "can_id", "down"])
    except Exception as e:
        print("Warning: bring_down_can failed:", e) # Debug

# Battery controller for ROS
class BatteryController(Node):
    def __init__(self):
        super().__init__("battery_controller")
        self.create_publisher()

        # Start CAN Bus
        self.bus = create_battery_bus()

    def __del__(self):
        # Shutdown Battery CAN bus
        try:
            self.bus.shutdown()
        except:
            pass
        
        bring_down_can("can1")
        print("Battery CAN interface shut down.")    

# Call motors
def build_motors(bus, ids):
    return [Motor(bus, mid) for mid in ids]

# Motor controller for ROS
class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.create_subscription(TwistStamped, "/cmd_vel", self.receive_twist, 10)

        # Start CAN Bus
        self.bus = create_bus()

        # Initialize motors
        self.left = build_motors(self.bus, LEFT_MOTORS)
        self.right = build_motors(self.bus, RIGHT_MOTORS)
        self.motors = self.left + self.right

        # Create MotorGroup with twist parameters
        self.group = MotorGroup(self.left, self.right, track_width_m=0.5, wheel_radius_m=0.175)    # arbitrarily set width + radius

        # Init motor state
        for m in self.motors:
            m.set_mode(0)
            time.sleep(0.01)

            m.set_current(0)
            time.sleep(0.01)

            m.set_rpm(0)
            time.sleep(0.01)

            m.set_mode(2)
            time.sleep(0.01)

        # Odometry initializers with a track_width set arbitrarily to 0.5m
        self.odom = SkidSteerOdometry(track_width_m=0.5)
        self.last_odom_time = time.time()

    def __del__(self):
        # Shutdown CAN bus
        try:
            self.bus.shutdown()
        except:
            pass
        
        bring_down_can("can0")
        print("Motor CAN interface shut down.")

    # Store received twist (v_x and w_z) into shared structure
    def receive_twist(self, msg: TwistStamped):

        v_x = float(msg.twist.linear.x)
        w_z = float(msg.twist.angular.z)

        #print(v_x, w_z)
        # Apply skid-steer output using Twist
        self.group.apply_twist(v_x, w_z)

# Main function
def main(args=None):
    rclpy.init(args=args)  # initialize ROS client library
    node = MotorController()
    batteryNode = Battery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
