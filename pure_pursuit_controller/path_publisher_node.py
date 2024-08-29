#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.coordinates = [[-11.0,51.9],[-7.0,51.9],[-3.0,51.9],[1.0,51.9],[5.0,51.9],[9.0,51.9],[13.0,51.9],[17.0,51.9],[18.5,52.7],[19.7,52.7],
                            [22.0,52.7],[22.85,52.5],[22.9,50.0],[22.9,46.0],[22.9,42.0],[22.9,38.0],[22.9,34.0]]

    def timer_callback(self):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        poses = []
        for (x, y) in self.coordinates:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation
            poses.append(pose)
        
        path_msg.poses = poses
        self.publisher_.publish(path_msg)
        self.get_logger().info('Publishing path')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
