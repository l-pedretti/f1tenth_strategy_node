#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
import random 

# sending simulated car position values
class CarPublisher(Node):
        def __init__(self):
            #creating car publisher
            super().__init__('car_publisher')
            self.publisher_ = self.create_publisher(Odometry, 'car', 10)
            timer_period = 0.5
            self.timer = self.create_timer(timer_period, self.timer_callback)

        def timer_callback(self):
            msg = Odometry()
            #generating random values for car position
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = random.uniform(0.0,10.0)
            msg.pose.pose.position.y = random.uniform(0.0,10.0)
            msg.pose.pose.position.z = random.uniform(0.0,10.0)

            self.publisher_.publish(msg)
            self.get_logger().info('Publishing car position: [%s, %s, %s]'% (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))


def main(args=None):
    rclpy.init(args=args)

    car_publisher = CarPublisher()
    try:
        #initializing car node
        rclpy.spin(car_publisher)
    #exception to avoid keyboard interrupt error
    except KeyboardInterrupt:
        print('\n Interrupted')
    #destroying car node
    car_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    