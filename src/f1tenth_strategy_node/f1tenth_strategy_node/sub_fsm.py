import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#subscription to published informations about fsm current state
class FsmSubscriber(Node):

    def __init__(self):
        super().__init__('fsm_subscriber')
        #creating subscription
        self.subscription = self.create_subscription(
            String,
            'fsm_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Current FSM state: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    fsm_subscriber = FsmSubscriber()
    try:
        #initializing fsm subscriber node
        rclpy.spin(fsm_subscriber)
    #exception to avoid keyboard interrupt error
    except KeyboardInterrupt:
        print('\n Interrupted')
    #destroying fsm subscriber node
    fsm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()