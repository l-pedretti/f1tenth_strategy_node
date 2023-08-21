#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import random

class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'obstacle', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.poses = []

    def timer_callback(self):
        n = random.randint(0,5)
        for i in range(n):
            pose = Pose()

            pose.position.x = random.uniform(0.0,10.0)
            pose.position.y = random.uniform(0.0,10.0)
            pose.position.z = random.uniform(0.0,10.0)

            self.poses.append(pose)

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self.poses

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing obstacles : ')
        for i in range(n):
            print('[%s, %s, %s]'% (msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z))

        self.poses = []


def main(args=None):
    rclpy.init(args=args)

    obstacle_publisher = ObstaclePublisher()
    try:
        rclpy.spin(obstacle_publisher)
    except KeyboardInterrupt:
        print('\n Interrupted')

    obstacle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()