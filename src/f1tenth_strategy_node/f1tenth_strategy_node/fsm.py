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

import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String

from yasmin import State as yState
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from f1tenth_strategy_node.lifecycle import LifecycleNode
from lifecycle_msgs.msg import State as lState
from lifecycle_msgs.msg import Transition

globalStart = False
readyStart = False
current_fsm_state = ''
countObs = 0
di = 0
d0 = 0 
minL = 0
obsList = []


class LifecycleTalker (LifecycleNode):
    def __init__(self):
        super().__init__("lc_talker")
        self.pubcount = 0
        self.declare_parameter('dth', 0)
        
    def on_configure(self):
        self.pub = self.create_publisher(String, "lifecycle_chatter", 10)
        self.get_logger().info("on_configure() is called")
        self.create_timer(1.0, self.publish_callback)
        dth = rclpy.parameter.Parameter(
            'dth',
            rclpy.Parameter.Type.INTEGER,
            0
        )
        parameters = [dth]
        self.set_parameters(parameters)
        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_cleanup(self):
        self.get_logger().info("on_cleanup() is called")
        globalStart = False
        readyStart = False
        countObs = 0
        di = 0
        d0 = 0 
        dth = 0
        minL = 0
        obsList = []
        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_activate(self):
        self.get_logger().info("on_activate() is called")
        executor = MultiThreadedExecutor()
        self.obstacle_subscriber = ObstacleSubscriber()
        self.car_subscriber = CarSubscriber()
        self.fsm_publisher = FsmPublisher()
        self.fsm_node = FsmNode()


        executor.add_node(self.obstacle_subscriber)
        executor.add_node(self.car_subscriber)
        executor.add_node(self.fsm_publisher)
        executor.add_node(self.fsm_node)

        executor.spin()
        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_deactivate(self):
        self.get_logger().info("on_deactivate() is called")
        self.obstacle_subscriber.destroy_node()
        self.car_subscriber.destroy_node()
        self.fsm_node.destroy_node()
        self.fsm_publisher.destroy_node()
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def publish_callback(self):
        if(self.state == lState.PRIMARY_STATE_ACTIVE):
            self.pubcount += 1
            self.pub.publish(String(data = "Lifecycle (Python) Hello World #" + str(self.pubcount)))



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

class FsmPublisher(Node):

    def __init__(self):
        super().__init__('fsm_publisher')
        self.publisher_ = self.create_publisher(String, 'fsm_state', 10)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("Executing publisher")

    def timer_callback(self):
        print("Sending publisher")
        msg = String()
        msg.data = current_fsm_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)



class Obstacle():

    def __init__(self):
        self.pose = 0
        self.di = 0
        self.d0 = 0

class ReadyState(yState):
    def __init__(self):
        super().__init__(["G","R"])
        
    def execute(self, blackboard):
        print("Executing state Ready")
        time.sleep(3)
        global current_fsm_state
        current_fsm_state = 'R'
        if globalStart == True:
            return "G"
        else:
            return "R"
    

class GlobalState(yState):
    def __init__(self):
        super().__init__(["F", "R", "G"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state Global")
        time.sleep(3)
        global current_fsm_state
        current_fsm_state = 'G'

        if countObs == 0:
            return "R"
        elif readyStart == True:
            return "F"
        else:
            return "G"

class FollowState(yState):
    def __init__(self):
        super().__init__(["OI", "OO", "G", "R","F"])
        

    def execute(self, blackboard):
        print("Executing state Follow")
        time.sleep(3)
        global current_fsm_state
        current_fsm_state = 'F'

        if di <= d0:
            return "OI"
        elif d0 < di:
            return "OO"
        elif countObs == 0:
            return "G"
        elif readyStart == True:
            return "R"
        else: 
            return "F"

class OIState(yState):
    def __init__(self):
        super().__init__(["G", "R", "OI"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state OI")
        time.sleep(3)
        global current_fsm_state
        current_fsm_state = 'OI'

        if countObs == 0:
            return "G"
        elif readyStart == True:
            return "R"
        else:
            return "OI"

class OOState(yState):
    def __init__(self):
        super().__init__(["G", "R", "OO"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state OO")
        time.sleep(3)
        global current_fsm_state
        current_fsm_state = 'OO'

        if countObs == 0:
            return "G"
        elif readyStart == True:
            return "R"
        else:
            return "OO"


class FsmNode(Node):

    def __init__(self):
        super().__init__("fsm_node")

        # create a state machine
        sm = StateMachine(outcomes=["shutdown"])

        # add states
        sm.add_state("READY", ReadyState(),
                     transitions={"G": "GLOBAL",
                                  "R": "READY"})
        sm.add_state("GLOBAL", GlobalState(),
                     transitions={"R": "READY",
                                  "F": "FOLLOW",
                                  "G": "GLOBAL"})
        sm.add_state("FOLLOW", FollowState(),
                     transitions={"OI": "OVERTAKE INSIDE",
                                  "OO": "OVERTAKE INSIDE",
                                  "G": "GLOBAL",
                                  "R": "READY",
                                  "F": "FOLLOW"})

        sm.add_state("OVERTAKE INSIDE", OIState(),
                     transitions={"G": "GLOBAL",
                                  "R": "READY",
                                  "OI": "OVERTAKE INSIDE"})
        sm.add_state("OVERTAKE OUTSIDE", OOState(),
                     transitions={"G": "GLOBAL",
                                  "R": "READY",
                                  "OO": "OVERTAKE OUTSIDE"})


        # pub
        YasminViewerPub(self, "strategy_fsm_node", sm)

        # execute
        outcome = sm()
        print(outcome)


def main(args=None):
    rclpy.init(args=args)
    
    o = Obstacle()
    obsList.append(o)
    
    lifecycle_talker = LifecycleTalker()
    
    rclpy.spin(lifecycle_talker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()