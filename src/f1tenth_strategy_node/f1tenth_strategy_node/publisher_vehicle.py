#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from squaternion import Quaternion


class CarPublisher(Node):
        def __init__(self):
            super().__init__('car_publisher')
            self.publisher_ = self.create_publisher(Odometry, 'odometry', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)

        def timer_callback(self):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = 2 * cos(self.deg * (pi/180))
            msg.pose.pose.position.y = 2 * sin(self.deg * (pi/180))
            msg.pose.pose.position.z = 0.0

            quat = Quaternion.from_euler(0.0, 0.0, self.deg, degrees=True)
            msg.pose.pose.orientation.x = quat[1]
            msg.pose.pose.orientation.y = quat[2]
            msg.pose.pose.orientation.z = quat[3]
            msg.pose.pose.orientation.w = quat[0]

            self.publisher_.publish(msg)
            self.get_logger().info('Publishing car position: %s'% msg)


def main(args=None):
    rclpy.init(args=args)

    car_publisher = CarPublisher()

    rclpy.spin(car_publisher)

    car_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    