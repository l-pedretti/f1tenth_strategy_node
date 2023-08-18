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
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from f1tenth_strategy_node.lifecycle import LifecycleNode
from lifecycle_msgs.msg import State as lState
from lifecycle_msgs.msg import Transition
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from tutorial_interfaces.srv import ChangeState
import threading
import random 

globalStart = False
readyStart = False
current_fsm_state = ''
countObs = 0
carPos = 0
minL = 0
obsList = []


class LifecycleTalker (LifecycleNode):
    def __init__(self):
        super().__init__("lc_talker")
        self.pubcount = 0
        self.declare_parameter('dth', 0)
       
        self.srv = self.create_service(ChangeState, 'fsm_changeState', self.fsm_changeState_callback)

    def fsm_changeState_callback(self, request, response):
        if request.state == 'R':
            global readyStart
            readyStart = True
        elif request.state == 'G':
            global globalStart
            globalStart = True
        self.get_logger().info('Incoming request for fsm state change\n')
        
        response.response = 'state changed'

        return response
    
    def on_configure(self):
        self.get_logger().info("on_configure() is called")
        dth = rclpy.parameter.Parameter(
            'dth',
            rclpy.Parameter.Type.INTEGER,
            0
        )
        parameters = [dth]
        #DA SISTEMARE
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

        executor.add_node(self.obstacle_subscriber)
        executor.add_node(self.car_subscriber)
        executor.add_node(self.fsm_publisher)

        t1 = threading.Thread(target = executor.spin)
        t1.start()

        self.fsm_node = FsmNode
        t2 = threading.Thread(target = self.fsm_node)
        t2.start()


        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_deactivate(self):
        self.get_logger().info("on_deactivate() is called")
        self.obstacle_subscriber.destroy_node()
        self.car_subscriber.destroy_node()
        self.fsm_node.destroy_node()
        self.fsm_publisher.destroy_node()
        return Transition.TRANSITION_CALLBACK_SUCCESS


class ObstacleSubscriber(Node):

    def __init__(self):
        super().__init__('obstacle_subscriber')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            PoseArray,
            'obstacle',
            self.listener_callback,
            10,
            callback_group=client_cb_group)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        global obsList, countObs
        obsList = []
        countObs = 0
        for pose in msg.poses:
            di = random.randint(0,100)
            d0 = random.randint(0,100)
            obs = Obstacle(pose, di, d0)
            obsList.append(obs)
            countObs+=1
            print(pose)


class CarSubscriber(Node):

    def __init__(self):
        super().__init__('car_subscriber')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            Odometry,
            'car',
            self.listener_callback,
            10,
            callback_group=client_cb_group)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        global carPos 
        carPos = msg.pose.pose
        print(carPos)

class FsmPublisher(Node):

    def __init__(self):
        super().__init__('fsm_publisher')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_ = self.create_publisher(String, 'fsm_state', 10, callback_group=client_cb_group)
        timer_period = 1  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = current_fsm_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

class Obstacle():

    def __init__(self, pose, di, d0):
        self.pose = pose
        self.di = di
        self.d0 = d0

class ReadyState(yState):
    def __init__(self):
        super().__init__(["G","R"])
        
    def execute(self, blackboard):
        global current_fsm_state
        if current_fsm_state != 'R':
            print("Executing state Ready")
        time.sleep(3)
        global globalStart 
        current_fsm_state = 'R'
        if globalStart == True:
            globalStart = False
            return "G"
        else:
            return "R"
    
class GlobalState(yState):
    def __init__(self):
        super().__init__(["F", "R", "G"])
        self.counter = 0

    def execute(self, blackboard):
        global current_fsm_state
        if current_fsm_state != 'G':
            print("Executing state Global")
        time.sleep(3)
        global readyStart
        current_fsm_state = 'G'
        countObs = 5
        if readyStart == True:
            readyStart = False
            return "R"
        elif countObs != 0:
            return "F"

        else:
            return "G"

class FollowState(yState):
    def __init__(self):
        super().__init__(["OI", "OO", "G", "R","F"])
        
    def execute(self, blackboard):
        global current_fsm_state
        if current_fsm_state != 'F':
            print("Executing state Follow")
        time.sleep(3)
        current_fsm_state = 'F'
        global readyStart
        if readyStart == True:
            readyStart = False
            return "R"
        elif di <= d0:
            return "OI"
        elif d0 < di:
            return "OO"
        elif countObs == 0:
            return "G"
        else: 
            return "F"

class OIState(yState):
    def __init__(self):
        super().__init__(["G", "R", "OI"])
        self.counter = 0

    def execute(self, blackboard):
        global current_fsm_state
        if current_fsm_state != 'OI':
            print("Executing state OI")
        time.sleep(3)
        global readyStart
        current_fsm_state = 'OI'
        if readyStart == True:
            readyStart = False
            return "R"
        elif countObs == 0:
            return "G"
        else:
            return "OI"

class OOState(yState):
    def __init__(self):
        super().__init__(["G", "R", "OO"])
        self.counter = 0

    def execute(self, blackboard):
        global current_fsm_state
        if current_fsm_state != 'OO':
            print("Executing state OO")
        time.sleep(3)
        global readyStart
        current_fsm_state = 'OO'
        if readyStart == True:
            readyStart = False
            return "R"
        elif countObs == 0:
            return "G"
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
    
    
    lifecycle_talker = LifecycleTalker()

    rclpy.spin(lifecycle_talker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()