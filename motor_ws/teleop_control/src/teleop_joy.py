#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock


MAX_LINEAR = 1.0     # m/s
MAX_ANGULAR = 2.0    # rad/s

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')

        # Subscribes to joystick data /joy
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publishes velocity commands to TurtleSim
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Joystick axis setup
        self.axis_linear = 1      # left stick vertical
        self.axis_angular = 0     # left stick horizontal

        # Button mappings 
        self.slow_button_1 = 4    # L1
        self.slow_button_2 = 6    # L2
        self.fast_button_1 = 5    # R1
        self.fast_button_2 = 7    # R2

        # Speed scales
        self.low_linear = 0.4
        self.low_angular = 0.8
        self.high_linear = 1.0
        self.high_angular = 1.0

        self.get_logger().info("Teleop with Joystick use L-side slow speed, and R-side fast")

    def joy_callback(self, msg):
        velocity = TwistStamped()
        velocity.header.stamp = Clock().now().to_msg()
        velocity.header.frame_id = ''

        # Check which mode we're in
        slow_mode = msg.buttons[self.slow_button_1] 
        fast_mode = msg.buttons[self.fast_button_1] 

        # Select speed
        if slow_mode:
            lin_scale = self.low_linear
            ang_scale = self.low_angular
        if fast_mode:
            lin_scale = self.high_linear
            ang_scale = self.high_angular
        else:
            # Nothing pressed then stop
            lin_scale = 0.0
            ang_scale = 0.0

        # Read joystick input
        linear_input = msg.axes[self.axis_linear]
        angular_input = msg.axes[self.axis_angular]

        

        #Flip turning direction when reversing
        if linear_input < 0.3: 
            angular_input *= -1.0

        
        lin_vel = linear_input * lin_scale
        ang_vel = angular_input * ang_scale

        
        lin_vel = max(-MAX_LINEAR, min(MAX_LINEAR, lin_vel))
        ang_vel = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang_vel))

        # Assign and publish 
        velocity.twist.linear.x = lin_vel
        velocity.twist.angular.z = ang_vel

        self.pub.publish(velocity)




def main():
    rclpy.init()
    node = TeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
