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
import math 

globalSwitch = False
readySwitch = False
shutdown = False
current_fsm_state = ''
countObs = 0
dth = 0.0
carPos = Pose()
obsList = []
tNc = 0.0
lock = threading.Lock()


class LifecycleTalker (LifecycleNode):
    def __init__(self):
        super().__init__("lc_talker")
        self.declare_parameter("dth", 3.0)
        self.declare_parameter("tnc", 5.0)
        self.srv = self.create_service(ChangeState, 'fsm_changeState', self.fsm_changeState_callback)

    def fsm_changeState_callback(self, request, response):
        self.get_logger().info('Incoming request for fsm state change')
        lock.acquire()
        if request.state == 'R':
            global readySwitch
            readySwitch = True
        elif request.state == 'G':
            global globalSwitch
            globalSwitch = True
        lock.release()    
        
        response.response = 'state changed'
        
        return response
    
    def on_configure(self):
        self.get_logger().info("on_configure() is called")
        lock.acquire()
        global dth
        dth = self.get_parameter("dth").get_parameter_value().double_value
        global tNc
        tNc = self.get_parameter("tnc").get_parameter_value().double_value
        lock.release() 
        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_cleanup(self):
        self.get_logger().info("on_cleanup() is called")
        lock.acquire()
        global globalSwitch, readySwitch, countObs, dth, tNc, obsList, current_fsm_state, carPos, shutdown
        globalSwitch = False
        readySwitch = False
        shutdown = False
        countObs = 0
        dth = 0.0
        tNc = 0.0 
        obsList = []
        globalSwitch = False
        current_fsm_state = ''
        carPos = Pose() 
        lock.release() 
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

        self.t1 = threading.Thread(target = executor.spin)
        self.t1.start()

        self.fsm_node = FsmNode()
        self.t2 = threading.Thread(target = self.fsm_node.execute)
        self.t2.start()


        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_deactivate(self):
       
        self.get_logger().info("on_deactivate() is called")
        if (current_fsm_state == 'R'):
            global shutdown 
            shutdown = True
            self.obstacle_subscriber.destroy_node()
            self.car_subscriber.destroy_node()
            self.fsm_node.destroy_node()
            self.fsm_publisher.destroy_node()
        else:
            print("FSM must be in READY state to call shutdown!")

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
        lock.acquire()
        global carPos 
        carPos = msg.pose.pose
        lock.release()

class FsmPublisher(Node):

    def __init__(self):
        super().__init__('fsm_publisher')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_ = self.create_publisher(String, 'fsm_state', 10, callback_group=client_cb_group)
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = current_fsm_state
        self.publisher_.publish(msg)

class Obstacle():

    def __init__(self, pose, di, d0):
        self.pose = pose
        self.di = di
        self.d0 = d0

class ReadyState(yState):
    def __init__(self):
        super().__init__(["S","G","R"])
        
    def execute(self, blackboard):
        lock.acquire()
        global current_fsm_state
        if current_fsm_state != 'R':
            print("Executing state Ready")
       
        global globalSwitch, shutdown 
        current_fsm_state = 'R'
        if shutdown == True:
            shutdown == False
            lock.release()
            return "S"
        elif globalSwitch == True:
            globalSwitch = False
            lock.release()
            return "G"
        else:
            lock.release()
            return "R"
        
            

class GlobalState(yState):
    def __init__(self):
        super().__init__(["F", "R", "G"])
        self.counter = 0

    def execute(self, blackboard):
        global current_fsm_state
        lock.acquire()
       
        
        if current_fsm_state != 'G':
            print("Executing state Global")
        global readySwitch
        current_fsm_state = 'G'

        if readySwitch == True:
            readySwitch = False
            lock.release()
            return "R"
        elif countObs != 0:
            lock.release()
            return "F"
        else:
            lock.release()
            return "G"

class FollowState(yState):
    def __init__(self):
        super().__init__(["R", "OI", "OO", "G", "F"])
        self.timer = 0

    def execute(self, blackboard):
        lock.acquire()
        global current_fsm_state
        if current_fsm_state != 'F':
            self.timer = time.perf_counter()
            print("Executing state Follow")
    
        current_fsm_state = 'F'
        global readySwitch

        min_dist = 1000.0
        min_obs = Obstacle(Pose(), 0, 0)
        for obs in obsList:
            obs_point = [obs.pose.position.x, obs.pose.position.y, obs.pose.position.z]
            car_point = [carPos.position.x, carPos.position.y, carPos.position.z]
            dist = math.dist(obs_point, car_point)
            if (dist < dth) & (dist < min_dist):
                min_obs = obs
                min_dist = dist

        if readySwitch == True:
            readySwitch = False
            lock.release()
            return "R"
        elif min_obs.di <= min_obs.d0 and min_dist < 1000:
            lock.release()
            return "OI"
        elif min_obs.di > min_obs.d0 and min_dist < 1000:
            lock.release()
            return "OO"
        elif countObs == 0 and self.timer > tNc:
            lock.release()
            return "G"
        else:
            lock.release() 
            return "F"

class OIState(yState):
    def __init__(self):
        super().__init__(["R", "G", "F", "OI"])
        self.counter = 0

    def execute(self, blackboard):
        lock.acquire()
        global current_fsm_state
        if current_fsm_state != 'OI':
            print("Executing state OI")
       
        global readySwitch
        current_fsm_state = 'OI'

        stop_overtaking = True
        for obs in obsList:
            obs_point = [obs.pose.position.x, obs.pose.position.y, obs.pose.position.z]
            car_point = [carPos.position.x, carPos.position.y, carPos.position.z]
            dist = math.dist(obs_point, car_point)
            if (dist > dth):
                stop_overtaking = False

        if readySwitch == True:
            readySwitch = False
            lock.release()
            return "R"
        elif countObs == 0:
            lock.release()
            return "G"
        elif stop_overtaking == True:
            lock.release()
            return "F"
        else:
            lock.release()
            return "OI"

class OOState(yState):
    def __init__(self):
        super().__init__(["R", "G", "F", "OO"])
        self.counter = 0

    def execute(self, blackboard):
        lock.acquire()
        global current_fsm_state
        if current_fsm_state != 'OO':
            print("Executing state OO")

        global readySwitch
        current_fsm_state = 'OO'

        stop_overtaking = True
        for obs in obsList:
            obs_point = [obs.pose.position.x, obs.pose.position.y, obs.pose.position.z]
            car_point = [carPos.position.x, carPos.position.y, carPos.position.z]
            dist = math.dist(obs_point, car_point)
            if (dist > dth):
                stop_overtaking = False

        if readySwitch == True:
            readySwitch = False
            lock.release()
            return "R"
        elif countObs == 0:
            lock.release()
            return "G"
        elif stop_overtaking == True:
            lock.release()
            return "F"
        else:
            lock.release()
            return "OO"


class FsmNode(Node):

    def __init__(self):
        super().__init__("fsm_node")

        # create a state machine
        self.sm = StateMachine(outcomes=["SHUTDOWN"])

        # add states
        self.sm.add_state("READY", ReadyState(),
                     transitions={"S": "SHUTDOWN",
                                  "G": "GLOBAL",
                                  "R": "READY"})
        self.sm.add_state("GLOBAL", GlobalState(),
                     transitions={"R": "READY",
                                  "F": "FOLLOW",
                                  "G": "GLOBAL"})
        self.sm.add_state("FOLLOW", FollowState(),
                     transitions={"OI": "OVERTAKE INSIDE",
                                  "OO": "OVERTAKE OUTSIDE",
                                  "G": "GLOBAL",
                                  "R": "READY",
                                  "F": "FOLLOW"})

        self.sm.add_state("OVERTAKE INSIDE", OIState(),
                     transitions={"G": "GLOBAL",
                                  "R": "READY",
                                  "F": "FOLLOW",
                                  "OI": "OVERTAKE INSIDE"})
        self.sm.add_state("OVERTAKE OUTSIDE", OOState(),
                     transitions={"G": "GLOBAL",
                                  "R": "READY",
                                  "F": "FOLLOW",
                                  "OO": "OVERTAKE OUTSIDE"})


        # pub
        YasminViewerPub(self, "strategy_fsm_node", self.sm)

    # execute
    def execute(self):
        outcome = self.sm()
        print(outcome)


def main(args=None):

    rclpy.init(args=args)
    
    lifecycle_talker = LifecycleTalker()
    try:
        rclpy.spin(lifecycle_talker)
    except KeyboardInterrupt:
        print('\n Interrupted')

    lifecycle_talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()