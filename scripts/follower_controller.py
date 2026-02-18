#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random


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
        
        # LIDAR and follower-path parameters
        self.lidar_max_range = 30.0  # Maximum LIDAR range
        self.front_fov_half_angle = math.radians(80.0)  # Front-facing lidar hemisphere-ish
        self.min_proximity_limit = 0.5  # Hard minimum distance to leader (m)
        self.soft_proximity_limit = 1.5  # Start active separation behavior
        self.braking_distance = 3.0  # Taper forward speed when inside this distance

        # Scripted path around the leader with continuous randomization
        self.base_distance = random.uniform(6.0, 14.0)
        self.base_relative_angle = random.uniform(-1.0, 1.0)

        self.distance_freq_1 = random.uniform(0.04, 0.10)
        self.distance_amp_1 = random.uniform(1.0, 2.5)
        self.distance_freq_2 = random.uniform(0.08, 0.18)
        self.distance_amp_2 = random.uniform(0.5, 1.5)
        self.distance_phase_2 = random.uniform(0.0, 2.0 * math.pi)

        self.angle_freq_1 = random.uniform(0.04, 0.10)
        self.angle_amp_1 = random.uniform(0.2, 0.5)
        self.angle_freq_2 = random.uniform(0.08, 0.18)
        self.angle_amp_2 = random.uniform(0.1, 0.3)
        self.angle_phase_2 = random.uniform(0.0, 2.0 * math.pi)
        
        # Control gains (path following + keep leader in front FOV)
        self.kp_linear = 0.5
        self.kp_angular_path = 1.8
        self.kp_angular_front = 1.2
        
        self.time = 0.0
        self.prev_leader_distance = None
        
        self.get_logger().info(f'Follower controller started')
        self.get_logger().info(f'Base distance: {self.base_distance:.2f} m')
        self.get_logger().info(f'Base relative angle: {self.base_relative_angle:.2f} rad')

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
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

        # Scripted relative path around leader (continuous randomization)
        distance_wave = (
            self.distance_amp_1 * math.sin(self.time * self.distance_freq_1)
            + self.distance_amp_2 * math.sin(self.time * self.distance_freq_2 + self.distance_phase_2)
        )
        angle_wave = (
            self.angle_amp_1 * math.sin(self.time * self.angle_freq_1)
            + self.angle_amp_2 * math.sin(self.time * self.angle_freq_2 + self.angle_phase_2)
        )

        current_desired_distance = self.base_distance + distance_wave
        current_desired_angle = self.base_relative_angle + angle_wave

        # Keep desired spacing inside usable lidar envelope
        current_desired_distance = max(3.0, min(18.0, current_desired_distance))
        
        # Calculate desired position relative to leader
        desired_x = self.leader_x + current_desired_distance * math.cos(self.leader_theta + current_desired_angle)
        desired_y = self.leader_y + current_desired_distance * math.sin(self.leader_theta + current_desired_angle)
        
        # Calculate error
        error_x = desired_x - self.follower_x
        error_y = desired_y - self.follower_y
        
        # Distance and angle to desired position
        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        
        # Angle error for path target
        path_angle_error = self.wrap_angle(angle_to_target - self.follower_theta)

        # Keep leader visible in front-mounted lidar (back is occluded)
        leader_dx = self.leader_x - self.follower_x
        leader_dy = self.leader_y - self.follower_y
        leader_distance = math.sqrt(leader_dx**2 + leader_dy**2)
        leader_bearing = self.wrap_angle(math.atan2(leader_dy, leader_dx) - self.follower_theta)

        if self.prev_leader_distance is None:
            closing_rate = 0.0
        else:
            closing_rate = (leader_distance - self.prev_leader_distance) / 0.02
        self.prev_leader_distance = leader_distance

        if abs(leader_bearing) > self.front_fov_half_angle:
            front_error = leader_bearing - math.copysign(self.front_fov_half_angle, leader_bearing)
        else:
            front_error = leader_bearing
        
        # Generate velocity commands
        twist = Twist()
        
        # Proportional control
        twist.linear.x = self.kp_linear * distance_error
        twist.angular.z = (
            self.kp_angular_path * path_angle_error
            + self.kp_angular_front * front_error
        )

        # Reduce forward speed when leader drifts toward rear/edge of FOV
        front_visibility_scale = max(0.0, math.cos(leader_bearing))
        twist.linear.x *= (0.25 + 0.75 * front_visibility_scale)

        # If leader gets close to lidar limit, prioritize closing the gap
        if leader_distance > 0.85 * self.lidar_max_range:
            twist.linear.x = max(twist.linear.x, 0.5)
            twist.angular.z += 0.5 * leader_bearing

        # Braking envelope: prevent forward push as follower gets closer
        if leader_distance < self.braking_distance:
            distance_margin = max(0.0, leader_distance - (self.min_proximity_limit + 0.1))
            braking_span = max(0.01, self.braking_distance - (self.min_proximity_limit + 0.1))
            forward_scale = distance_margin / braking_span
            max_forward_speed = 0.5 * forward_scale
            twist.linear.x = min(twist.linear.x, max_forward_speed)

            # If actively closing while near, clamp harder
            if closing_rate < -0.05:
                twist.linear.x = min(twist.linear.x, 0.1)
        
        # Hard/soft proximity overrides (prevent contact and pushing)
        if leader_distance <= self.min_proximity_limit:
            twist.linear.x = -0.8
            twist.angular.z = 1.5 if leader_bearing >= 0.0 else -1.5
        elif leader_distance <= self.soft_proximity_limit:
            separation = self.soft_proximity_limit - leader_distance
            twist.linear.x = min(twist.linear.x, -0.15 - 0.6 * separation)
            twist.angular.z += 1.0 * (1.0 if leader_bearing >= 0.0 else -1.0)

        # Limit velocities
        twist.linear.x = max(-0.5, min(1.0, twist.linear.x))
        twist.angular.z = max(-1.5, min(1.5, twist.angular.z))
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Periodically regenerate random baseline parameters
        time_int = int(self.time)
        if time_int > 0 and time_int % 15 == 0 and (self.time - time_int) < 0.02:
            self.base_distance = random.uniform(6.0, 14.0)
            self.base_relative_angle = random.uniform(-1.0, 1.0)
            self.distance_freq_1 = random.uniform(0.04, 0.10)
            self.distance_freq_2 = random.uniform(0.08, 0.18)
            self.angle_freq_1 = random.uniform(0.04, 0.10)
            self.angle_freq_2 = random.uniform(0.08, 0.18)
            self.get_logger().info(
                f'Updated scripted path - Distance: {self.base_distance:.2f}, '
                f'Angle: {self.base_relative_angle:.2f}'
            )


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
