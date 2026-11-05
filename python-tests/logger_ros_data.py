
#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ROSDataLogger(Node):
    def __init__(self):
        super().__init__('ros_data_logger')

        # Create output directory relative to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir, 'testlogs')
        os.makedirs(log_dir, exist_ok=True)

        filename = os.path.join(
            log_dir,
            f"velocity_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
        )

        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        # CSV header
        self.writer.writerow([
            'timestamp',
            'cmd_linear_x',
            'odom_linear_x',
            'cmd_angular_z',
            'odom_angular_z'
        ])

        # Latest commanded velocities
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Logging velocity data to: {filename}')

    def cmd_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def odom_callback(self, msg):
        # Use odometry timestamp
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9

        self.writer.writerow([
            timestamp,
            self.cmd_linear_x,
            msg.twist.twist.linear.x,
            self.cmd_angular_z,
            msg.twist.twist.angular.z
        ])

        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ROSDataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
