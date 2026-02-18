#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import random
import numpy as np


class LeaderController(Node):
    """
    Leader robot controller that follows a scripted path with randomized
    continuous poses, speeds, and orientations.
    """
    
    def __init__(self):
        super().__init__('leader_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for control loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Path parameters
        self.path_type = 'lemniscate'  # figure-8 pattern
        self.scale = 10.0  # scale of the pattern
        self.time = 0.0
        
        # Randomized parameters (continuous)
        self.base_linear_speed = random.uniform(0.3, 0.8)  # m/s
        self.base_angular_speed = random.uniform(0.1, 0.3)  # rad/s
        
        # Noise parameters for continuous variation
        self.speed_noise_freq = random.uniform(0.1, 0.3)
        self.speed_noise_amp = random.uniform(0.1, 0.2)
        self.angular_noise_freq = random.uniform(0.1, 0.3)
        self.angular_noise_amp = random.uniform(0.05, 0.15)
        
        # Previous position for trajectory tracking
        self.prev_x = 0.0
        self.prev_y = 0.0
        
        self.get_logger().info(f'Leader controller started')
        self.get_logger().info(f'Base speed: {self.base_linear_speed:.2f} m/s')
        self.get_logger().info(f'Base angular speed: {self.base_angular_speed:.2f} rad/s')
    
    def lemniscate_path(self, t):
        """
        Generate a figure-8 (lemniscate) path.
        Returns (x, y, theta) for the desired position and orientation.
        """
        a = self.scale
        
        # Parametric equations for lemniscate
        denominator = 1 + (math.sin(t) ** 2)
        x = a * math.cos(t) / denominator
        y = a * math.sin(t) * math.cos(t) / denominator
        
        # Calculate orientation from path derivative
        dt = 0.01
        denominator_next = 1 + (math.sin(t + dt) ** 2)
        x_next = a * math.cos(t + dt) / denominator_next
        y_next = a * math.sin(t + dt) * math.cos(t + dt) / denominator_next
        
        dx = x_next - x
        dy = y_next - y
        theta = math.atan2(dy, dx)
        
        return x, y, theta
    
    def control_loop(self):
        """
        Main control loop that generates velocity commands.
        """
        # Update time with randomized speed
        speed_variation = math.sin(self.time * self.speed_noise_freq) * self.speed_noise_amp
        time_increment = (self.base_linear_speed + speed_variation) * 0.02 / self.scale
        self.time += time_increment
        
        # Get desired path point
        x, y, theta = self.lemniscate_path(self.time)
        
        # Calculate velocity with continuous randomization
        twist = Twist()
        
        # Add continuous noise to linear velocity
        linear_noise = math.sin(self.time * 2.0 * self.speed_noise_freq) * self.speed_noise_amp
        twist.linear.x = self.base_linear_speed + linear_noise
        
        # Add continuous noise to angular velocity
        angular_noise = math.sin(self.time * 2.0 * self.angular_noise_freq) * self.angular_noise_amp
        
        # Calculate turn rate from path curvature
        dt = 0.02
        _, _, theta_next = self.lemniscate_path(self.time + time_increment)
        angular_vel = (theta_next - theta) / dt
        
        twist.angular.z = angular_vel + angular_noise
        
        # Limit velocities to reasonable ranges
        twist.linear.x = max(0.1, min(1.0, twist.linear.x))
        twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Periodically regenerate random parameters for variation
        time_int = int(self.time)
        if time_int > 0 and time_int % 10 == 0 and (self.time - time_int) < 0.02:  # Every 10 seconds
            self.base_linear_speed = random.uniform(0.3, 0.8)
            self.base_angular_speed = random.uniform(0.1, 0.3)
            self.get_logger().info(f'Updated speeds - Linear: {self.base_linear_speed:.2f}, Angular: {self.base_angular_speed:.2f}')


def main(args=None):
    rclpy.init(args=args)
    leader_controller = LeaderController()
    
    try:
        rclpy.spin(leader_controller)
    except KeyboardInterrupt:
        pass
    finally:
        leader_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
