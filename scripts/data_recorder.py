#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import csv
import os
from datetime import datetime
import numpy as np
import struct


class DataRecorder(Node):
    """
    Records leader pose, follower pose, and LIDAR pointcloud data to CSV files.
    """
    
    def __init__(self):
        super().__init__('data_recorder')
        
        # Create output directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.expanduser(f'~/relative_pose_data_{timestamp}')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CSV file paths
        self.leader_pose_file = os.path.join(self.output_dir, 'leader_pose.csv')
        self.follower_pose_file = os.path.join(self.output_dir, 'follower_pose.csv')
        self.lidar_info_file = os.path.join(self.output_dir, 'lidar_info.csv')
        self.pointcloud_dir = os.path.join(self.output_dir, 'pointclouds')
        os.makedirs(self.pointcloud_dir, exist_ok=True)
        
        # Initialize CSV files
        self.init_csv_files()
        
        # Subscribers
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
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/follower/pointcloud',
            self.lidar_callback,
            10
        )
        
        # Counters
        self.leader_count = 0
        self.follower_count = 0
        self.lidar_count = 0
        
        self.get_logger().info(f'Data recorder started')
        self.get_logger().info(f'Output directory: {self.output_dir}')
    
    def init_csv_files(self):
        """Initialize CSV files with headers."""
        # Leader pose CSV
        with open(self.leader_pose_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_sec', 'timestamp_nsec', 'x', 'y', 'z', 
                           'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw',
                           'vel_linear_x', 'vel_linear_y', 'vel_linear_z',
                           'vel_angular_x', 'vel_angular_y', 'vel_angular_z'])
        
        # Follower pose CSV
        with open(self.follower_pose_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_sec', 'timestamp_nsec', 'x', 'y', 'z',
                           'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw',
                           'vel_linear_x', 'vel_linear_y', 'vel_linear_z',
                           'vel_angular_x', 'vel_angular_y', 'vel_angular_z'])
        
        # LIDAR info CSV
        with open(self.lidar_info_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_sec', 'timestamp_nsec', 'frame_id',
                           'height', 'width', 'point_count', 'pointcloud_file'])
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def leader_pose_callback(self, msg):
        """Record leader pose data."""
        pose = msg.pose.pose
        twist = msg.twist.twist
        
        # Convert quaternion to Euler
        roll, pitch, yaw = self.quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        
        # Write to CSV
        with open(self.leader_pose_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                roll,
                pitch,
                yaw,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z
            ])
        
        self.leader_count += 1
        if self.leader_count % 100 == 0:
            self.get_logger().info(f'Recorded {self.leader_count} leader poses')
    
    def follower_pose_callback(self, msg):
        """Record follower pose data."""
        pose = msg.pose.pose
        twist = msg.twist.twist
        
        # Convert quaternion to Euler
        roll, pitch, yaw = self.quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        
        # Write to CSV
        with open(self.follower_pose_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                roll,
                pitch,
                yaw,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z
            ])
        
        self.follower_count += 1
        if self.follower_count % 100 == 0:
            self.get_logger().info(f'Recorded {self.follower_count} follower poses')
    
    def lidar_callback(self, msg):
        """Record LIDAR pointcloud data."""
        # Save pointcloud to binary file
        pc_filename = f'pointcloud_{self.lidar_count:06d}.bin'
        pc_filepath = os.path.join(self.pointcloud_dir, pc_filename)
        
        # Extract points from PointCloud2 message
        points = self.extract_points(msg)
        
        # Save as binary file (more efficient than CSV for large point clouds)
        points.tofile(pc_filepath)
        
        # Write metadata to CSV
        with open(self.lidar_info_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                msg.height,
                msg.width,
                len(points),
                pc_filename
            ])
        
        self.lidar_count += 1
        if self.lidar_count % 10 == 0:
            self.get_logger().info(f'Recorded {self.lidar_count} point clouds ({len(points)} points each)')
    
    def extract_points(self, cloud_msg):
        """Extract XYZ points from PointCloud2 message."""
        # Parse point cloud data
        points = []
        point_step = cloud_msg.point_step
        
        for i in range(0, len(cloud_msg.data), point_step):
            # Extract x, y, z (assuming float32 format)
            x = struct.unpack('f', cloud_msg.data[i:i+4])[0]
            y = struct.unpack('f', cloud_msg.data[i+4:i+8])[0]
            z = struct.unpack('f', cloud_msg.data[i+8:i+12])[0]
            
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])
        
        return np.array(points, dtype=np.float32)
    
    def __del__(self):
        """Print summary on shutdown."""
        self.get_logger().info('='*50)
        self.get_logger().info('Data Recording Summary:')
        self.get_logger().info(f'  Leader poses recorded: {self.leader_count}')
        self.get_logger().info(f'  Follower poses recorded: {self.follower_count}')
        self.get_logger().info(f'  Point clouds recorded: {self.lidar_count}')
        self.get_logger().info(f'  Output directory: {self.output_dir}')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    data_recorder = DataRecorder()
    
    try:
        rclpy.spin(data_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        data_recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
