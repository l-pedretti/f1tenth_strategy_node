#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from math import sin, cos, pi
from squaternion import Quaternion

class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'obstacle', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.deg = 0
        self.poses = []
        self.update = 1
        self.max = 30

    def timer_callback(self):
        pose = Pose()

        pose.position.x = 2 * cos(self.deg * (pi/180))
        pose.position.y = 2 * sin(self.deg * (pi/180))
        pose.position.z = 0.0
        x = 2 * cos(self.deg * (pi/180))
        y = 2 * sin(self.deg * (pi/180))
    
        quat = Quaternion.from_euler(0.0, 0.0, self.deg, degrees=True)

        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        pose.orientation.w = quat[0]

        self.poses.append(pose)

        #if(len(self.poses) == self.max):
        #self.poses.pop(0)

        msg = PoseArray()
        msg.header.frame_id = "world"
        msg.poses = self.poses[-self.i:]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing obstacles poses: %s'% msg)

        self.i += self.update
        self.update = 1 if self.i == 1 else (-1 if self.i == self.max else self.update)
        self.deg += 5

        self.poses = []


def main(args=None):
    rclpy.init(args=args)

    obstacle_publisher = ObstaclePublisher()

    rclpy.spin(obstacle_publisher)

    obstacle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()