# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#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
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor

globalStart = False
readyStart = False
countObs, di, d0, dth = 0
minL = 0
class ObstacleSubscriber(Node):

    def __init__(self):
        super().__init__('obstacle_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            'obstacle',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

class CarSubscriber(Node):

    def __init__(self):
        super().__init__('car_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'car',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)


# define state Ready
class ReadyState(State):
    def __init__(self):
        super().__init__(outcomes=["G"])

    def execute(self, blackboard):
        print("Executing state READY")
        time.sleep(3)

        
        if globalStart == True:
            return "G"

# define state Global
class GlobalState(State):
    def __init__(self):
        super().__init__(outcomes=["F", "R"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state GLOBAL")
        time.sleep(3)

        if countObs == 0:
            return "R"
        elif readyStart == True:
            return "F"

# define state Follow
class FollowState(State):
    def __init__(self):
        super().__init__(outcomes=["OI", "OO", "G", "R"])

    def execute(self, blackboard):
        print("Executing state FOLLOW")
        time.sleep(3)

    if di <= d0:
        return "OI"
    elif d0 < di:
        return "OO"
    elif countObs == 0:
        return "G"
    elif
        return "R"

# define state OI
class OIState(State):
    def __init__(self):
        super().__init__(outcomes=["G", "R"])

    def execute(self, blackboard):
        print("Executing state OVERTAKE INSIDE")
        time.sleep(3)

        print(blackboard.foo_str)
        return "outcome3"

# define state OO
class OOState(State):
    def __init__(self):
        super().__init__(outcomes=["G", "R"])

    def execute(self, blackboard):
        print("Executing state OVERTAKE OUTSIDE")
        time.sleep(3)

        print(blackboard.foo_str)
        return "outcome3"
    
class DemoNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["outcome4"])

        # add states
        sm.add_state("FOO", FooState(),
                     transitions={"outcome1": "BAR",
                                  "outcome2": "outcome4"})
        sm.add_state("BAR", BarState(),
                     transitions={"outcome3": "FOO"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO", sm)

        # execute
        outcome = sm()
        print(outcome)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    obstacle_subscriber = ObstacleSubscriber()
    executor.add_node(obstacle_subscriber)
    car_subscriber = CarSubscriber()
    executor.add_node(car_subscriber)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_subscriber.destroy_node()
    car_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
