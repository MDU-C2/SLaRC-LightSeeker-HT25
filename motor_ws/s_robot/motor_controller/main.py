#!/usr/bin/env python
import time
import subprocess
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from utils.motor.can_bus import create_bus
from utils.motor.motor_api import Motor
from utils.motor.motor_group import MotorGroup
from utils.motor.motor_odometry import SkidSteerOdometry
from utils.motor.motor_telemetry import MotorTelemetry
from utils.motor.motors import LEFT_MOTORS, RIGHT_MOTORS

from utils.battery.can_bus_battery import create_battery_bus
from utils.battery.communication_handler import Battery, BatteryManager

from s_msgs.msg import BatteryInfo

# Safely bring down CAN Bus #ALREADY IMPLEMENTED IN C
def bring_down_can():
    try:
        subprocess.call(["sudo", "ip", "link", "set", "can0", "down"])
        subprocess.call(["sudo", "ip", "link", "set", "can1", "down"])
    except Exception as e:
        print("Warning: bring_down_can failed:", e) # Debug

# Battery controller for ROS
class BatteryController(Node):
    def __init__(self):
        super().__init__("battery_controller")
        
        self.batteryPublisher = self.create_publisher(BatteryInfo, "battery_info", 10)
        self.publish_timer_batt = self.create_timer(10, self.publish_battery_info)
        # Start CAN Bus
        self.bus = create_battery_bus()

        # Initialize batteries
        self.batteryManager = BatteryManager(self.bus)

    def __del__(self):
        # Shutdown Battery CAN bus
        try:
            self.bus.shutdown()
        except:
            pass
        
        bring_down_can()
        print("Battery CAN interface shut down.")

    def publish_battery_info(self):
        # Create BatteryInfo ROS-message
        result_data = self.batteryManager.receive_packet()

        # No packet/data
        if result_data is None:
            return

        # result_data = (CAN-ID, packet_bytes)
        batt_id, packet = result_data

        # Grab the battery object and update their values
        battery = self.batteryManager.batteries[batt_id]

        # Create ROS-message and publish to topic
        msg = battery.pack_message_to_ros()
        self.batteryPublisher.publish(msg)

# Call motors
def build_motors(bus, ids):
    return [Motor(bus, mid) for mid in ids]

# Motor controller for ROS
class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Odometry initializers with a track_width set arbitrarily to 0.5 m and publish it at 10 Hz
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.last_odom_time = time.time()
        self.odom = SkidSteerOdometry(track_width_m=0.5)
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)
        
        self.create_subscription(TwistStamped, "/cmd_vel", self.receive_twist, 10)
        
        #self.motorPublisher = self.create_publisher(MotorInfo, "motor_info", 10)
        self.publish_timer_motor = self.create_timer(10, self.ROS_telemetry)

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

    def __del__(self):
        # Shutdown CAN bus
        try:
            self.bus.shutdown()
        except:
            pass
        
        bring_down_can()
        print("Motor CAN interface shut down.")

    # Store received twist (v_x and w_z) into shared structure
    def receive_twist(self, msg: TwistStamped):
        v_x = float(msg.twist.linear.x)
        w_z = float(msg.twist.angular.z)

        #print(v_x, w_z)
        # Apply skid-steer output using Twist
        self.group.apply_twist(v_x, w_z)

    def publish_odometry(self):
        now = time.time()
        dt = now - self.last_odom_time
        self.last_odom_time = now

        # RPM -> m/s conversion for left and right
        v_left = self.left[0].rpm_to_speed()
        v_right = self.right[0].rpm_to_speed()

        # Update odometry state
        x, y, yaw = self.odom.update(v_left, v_right, dt)

        # Build Odometry message: timestamp it, set frames, and publish robot pose in the odom -> base_link transform
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # UGV position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # UGV orientation as quaternion (yaw)
        msg.pose.pose.orientation = Quaternion(
            x = 0.0,
            y = 0.0,
            z = math.sin(yaw/2.0),
            w = math.cos(yaw/2.0),
        )

        # UGV Velocities
        msg.twist.twist.linear.x = (v_left + v_right) / 2.0
        msg.twist.twist.angular.z = (v_right - v_left) / self.odom.track_width

        # Publish odom
        self.odom_pub.publish(msg)

    def ROS_telemetry(self: MotorTelemetry):
        self.update_from_frame()

# Main function
def main(args=None):
    rclpy.init(args=args)  # initialize ROS client library
    #motorNode = MotorController()
    batteryNode = BatteryController()
    print("Battery node created")
    #rclpy.spin(motorNode)
    rclpy.spin(batteryNode)
    #motorNode.destroy_node()
    batteryNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
