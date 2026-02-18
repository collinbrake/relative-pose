#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random
import numpy as np


class FollowerController(Node):
    """
    Follower robot controller that maintains a position within LIDAR range
    of the leader with randomized relative distance and angle.
    """
    
    def __init__(self):
        super().__init__('follower_controller')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to both robots' ground truth poses
        self.leader_pose_sub = self.create_subscription(
            Odometry,
            '/leader/ground_truth',
            self.leader_pose_callback,
            10
        )
        
        self.follower_pose_sub = self.create_subscription(
            Odometry,
            '/follower/ground_truth',
            self.follower_pose_callback,
            10
        )
        
        # Timer for control loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Robot poses
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_theta = 0.0
        
        self.follower_x = -5.0
        self.follower_y = 0.0
        self.follower_theta = 0.0
        
        # Following parameters (randomized but continuous)
        self.lidar_max_range = 30.0  # Maximum LIDAR range
        self.desired_distance = random.uniform(5.0, 15.0)  # Stay within LIDAR range
        self.desired_angle = random.uniform(-math.pi, math.pi)  # Relative angle to leader
        
        # Noise parameters for continuous variation
        self.distance_noise_freq = random.uniform(0.05, 0.15)
        self.distance_noise_amp = random.uniform(1.0, 3.0)
        self.angle_noise_freq = random.uniform(0.05, 0.15)
        self.angle_noise_amp = random.uniform(0.2, 0.5)
        
        # Control gains
        self.kp_linear = 0.5
        self.kp_angular = 2.0
        
        self.time = 0.0
        
        self.get_logger().info(f'Follower controller started')
        self.get_logger().info(f'Desired distance: {self.desired_distance:.2f} m')
        self.get_logger().info(f'Desired angle: {self.desired_angle:.2f} rad')
    
    def leader_pose_callback(self, msg):
        """Update leader position from odometry."""
        self.leader_x = msg.pose.pose.position.x
        self.leader_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.leader_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def follower_pose_callback(self, msg):
        """Update follower position from odometry."""
        self.follower_x = msg.pose.pose.position.x
        self.follower_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.follower_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        """
        Main control loop that generates velocity commands to maintain
        desired relative position to leader.
        """
        self.time += 0.02
        
        # Add continuous randomization to desired distance and angle
        distance_noise = math.sin(self.time * self.distance_noise_freq) * self.distance_noise_amp
        angle_noise = math.sin(self.time * self.angle_noise_freq) * self.angle_noise_amp
        
        current_desired_distance = self.desired_distance + distance_noise
        current_desired_angle = self.desired_angle + angle_noise
        
        # Ensure distance stays within LIDAR range
        current_desired_distance = max(3.0, min(20.0, current_desired_distance))
        
        # Calculate desired position relative to leader
        desired_x = self.leader_x + current_desired_distance * math.cos(self.leader_theta + current_desired_angle)
        desired_y = self.leader_y + current_desired_distance * math.sin(self.leader_theta + current_desired_angle)
        
        # Calculate error
        error_x = desired_x - self.follower_x
        error_y = desired_y - self.follower_y
        
        # Distance and angle to desired position
        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        
        # Angle error (shortest angular distance)
        angle_error = angle_to_target - self.follower_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Generate velocity commands
        twist = Twist()
        
        # Proportional control
        twist.linear.x = self.kp_linear * distance_error
        twist.angular.z = self.kp_angular * angle_error
        
        # Limit velocities
        twist.linear.x = max(0.0, min(1.0, twist.linear.x))
        twist.angular.z = max(-1.5, min(1.5, twist.angular.z))
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Periodically regenerate random baseline parameters
        time_int = int(self.time)
        if time_int > 0 and time_int % 15 == 0 and (self.time - time_int) < 0.02:  # Every 15 seconds
            self.desired_distance = random.uniform(5.0, 15.0)
            self.desired_angle = random.uniform(-math.pi, math.pi)
            self.get_logger().info(f'Updated following params - Distance: {self.desired_distance:.2f}, Angle: {self.desired_angle:.2f}')


def main(args=None):
    rclpy.init(args=args)
    follower_controller = FollowerController()
    
    try:
        rclpy.spin(follower_controller)
    except KeyboardInterrupt:
        pass
    finally:
        follower_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
