
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

#class that implements managed nodes functionality 
class LifecycleTalker (LifecycleNode):
    def __init__(self): #declaration of parameters
        super().__init__("lc_talker")
        self.declare_parameter("dth", 3.0)
        self.declare_parameter("tnc", 5.0)
        self.srv = self.create_service(ChangeState, 'fsm_changeState', self.fsm_changeState_callback)
    
    #change state callback 
    def fsm_changeState_callback(self, request, response):
        self.get_logger().info('Incoming request for fsm state change')
        #mutex acquisition
        lock.acquire()

        if request.state == 'R':
            #switching to ready state 
            global readySwitch
            readySwitch = True
        elif request.state == 'G':
            #switching to global state 
            global globalSwitch
            globalSwitch = True
        #mutex release
        lock.release()    
        
        response.response = 'state changed'
        
        return response
    
    #configure callback
    def on_configure(self):
        self.get_logger().info("on_configure() is called")
        #mutex acquisition
        lock.acquire()
        global dth
        dth = self.get_parameter("dth").get_parameter_value().double_value
        global tNc
        tNc = self.get_parameter("tnc").get_parameter_value().double_value
        #mutex release 
        lock.release() 
        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    #cleanup callback
    def on_cleanup(self):
        self.get_logger().info("on_cleanup() is called")
        #mutex acquisition
        lock.acquire()
        #initialization of global variables
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
        #mutex release
        lock.release() 
        return Transition.TRANSITION_CALLBACK_SUCCESS

    #activate callback  
    def on_activate(self):
        self.get_logger().info("on_activate() is called")
        #instantiation of class nodes
        executor = MultiThreadedExecutor()
        self.obstacle_subscriber = ObstacleSubscriber()
        self.car_subscriber = CarSubscriber()
        self.fsm_publisher = FsmPublisher()

        executor.add_node(self.obstacle_subscriber)
        executor.add_node(self.car_subscriber)
        executor.add_node(self.fsm_publisher)
        #splitting nodes in 2 threads to get them working in parallel
        self.t1 = threading.Thread(target = executor.spin)
        self.t1.start()

        self.fsm_node = FsmNode()
        self.t2 = threading.Thread(target = self.fsm_node.execute)
        self.t2.start()


        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    #deactivate callback
    def on_deactivate(self):
       
        self.get_logger().info("on_deactivate() is called")
        #destroying all the nodes if fsm is in ready state
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

#CLASS OBSTACLE SUBSCRIBER: getting obstacles data and saving it into global variables
class ObstacleSubscriber(Node):

    def __init__(self):
        super().__init__('obstacle_subscriber')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        #creating the subscriber
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
            #generating random values for di, d0
            di = random.randint(0,100)
            d0 = random.randint(0,100)
            obs = Obstacle(pose, di, d0)
            obsList.append(obs)
            countObs+=1

#CLASS CAR SUBSCRIBER: getting car position and saving it to global carPos 
class CarSubscriber(Node):

    def __init__(self):
        super().__init__('car_subscriber')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        #creating the subscriber
        self.subscription = self.create_subscription(
            Odometry,
            'car',
            self.listener_callback,
            10,
            callback_group=client_cb_group)
        self.subscription  # prevent unused variable warning

    #callback definition
    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        #mutex acquire
        lock.acquire()
        global carPos 
        carPos = msg.pose.pose
        #mutex release
        lock.release()

#CLASS FSM PUBLISHER: taking current fsm state and publishing it
class FsmPublisher(Node):

    def __init__(self):
        super().__init__('fsm_publisher')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        #creating the publisher
        self.publisher_ = self.create_publisher(String, 'fsm_state', 10, callback_group=client_cb_group)
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = current_fsm_state
        self.publisher_.publish(msg)

class Obstacle():
    #initializing attributes
    def __init__(self, pose, di, d0):
        self.pose = pose
        self.di = di
        self.d0 = d0

#CLASSES FOR EACH STATE: Ready, Global,Follow, Overtake Inside, Overstake Outside
class ReadyState(yState):
    def __init__(self):
        super().__init__(["S","G","R"])
        
    def execute(self, blackboard):
        #mutex acquire
        lock.acquire()
        global current_fsm_state
        #controls across all possible cases
        if current_fsm_state != 'R':
            print("Executing state Ready")
        global globalSwitch, shutdown 
        current_fsm_state = 'R'
        if shutdown == True:
            shutdown == False
            #mutex release
            lock.release()
            return "S"
        elif globalSwitch == True:
            globalSwitch = False
            #mutex release
            lock.release()
            return "G"
        else:
            #mutex release
            lock.release()
            return "R"
        

class GlobalState(yState):
    def __init__(self):
        super().__init__(["F", "R", "G"])
        self.counter = 0

    def execute(self, blackboard):
        global current_fsm_state
        #mutex acquire
        lock.acquire()
        #controls across all possible cases
        if current_fsm_state != 'G':
            print("Executing state Global")
        global readySwitch
        current_fsm_state = 'G'

        if readySwitch == True:
            readySwitch = False
            #mutex release
            lock.release()
            return "R"
        elif countObs != 0:
            #mutex release
            lock.release()
            return "F"
        else:
            #mutex release
            lock.release()
            return "G"

class FollowState(yState):
    def __init__(self):
        super().__init__(["R", "OI", "OO", "G", "F"])
        self.timer = 0

    def execute(self, blackboard):
        #mutex acquire
        lock.acquire()
        global current_fsm_state
        #controls across all possible cases
        if current_fsm_state != 'F':
            self.timer = time.perf_counter()
            print("Executing state Follow")
    
        current_fsm_state = 'F'
        global readySwitch

        min_dist = 1000.0
        min_obs = Obstacle(Pose(), 0, 0)
        #list of obstacles
        for obs in obsList:
            obs_point = [obs.pose.position.x, obs.pose.position.y, obs.pose.position.z]
            car_point = [carPos.position.x, carPos.position.y, carPos.position.z]
            dist = math.dist(obs_point, car_point)
            #checking obstacles list to find the nearest one
            if (dist < dth) & (dist < min_dist):
                min_obs = obs
                min_dist = dist

        if readySwitch == True:
            readySwitch = False
            #mutex release
            lock.release()
            return "R"
        elif min_obs.di <= min_obs.d0 and min_dist < 1000:
            #mutex release
            lock.release()
            return "OI"
        elif min_obs.di > min_obs.d0 and min_dist < 1000:
            #mutex release
            lock.release()
            return "OO"
        elif countObs == 0 and self.timer > tNc:
            #mutex release
            lock.release()
            return "G"
        else:
            #mutex release
            lock.release() 
            return "F"

class OIState(yState):
    def __init__(self):
        super().__init__(["R", "G", "F", "OI"])
        self.counter = 0

    def execute(self, blackboard):
        #mutex acquire
        lock.acquire()
        global current_fsm_state
        #controls across all possible cases
        if current_fsm_state != 'OI':
            print("Executing state OI")
       
        global readySwitch
        current_fsm_state = 'OI'
        #flag for stopping the overtaking
        stop_overtaking = True
        for obs in obsList:
            obs_point = [obs.pose.position.x, obs.pose.position.y, obs.pose.position.z]
            car_point = [carPos.position.x, carPos.position.y, carPos.position.z]
            dist = math.dist(obs_point, car_point)
            #checking if the overstaking is finished
            if (dist > dth):
                stop_overtaking = False

        if readySwitch == True:
            readySwitch = False
            #mutex release
            lock.release()
            return "R"
        elif countObs == 0:
            #mutex release
            lock.release()
            return "G"
        elif stop_overtaking == True:
            #mutex release
            lock.release()
            return "F"
        else:
            #mutex release
            lock.release()
            return "OI"

class OOState(yState):
    def __init__(self):
        super().__init__(["R", "G", "F", "OO"])
        self.counter = 0

    def execute(self, blackboard):
        #mutex acquire
        lock.acquire()
        global current_fsm_state
        #controls across all possible cases
        if current_fsm_state != 'OO':
            print("Executing state OO")

        global readySwitch
        current_fsm_state = 'OO'
        #flag for stopping the overtaking
        stop_overtaking = True
        for obs in obsList:
            obs_point = [obs.pose.position.x, obs.pose.position.y, obs.pose.position.z]
            car_point = [carPos.position.x, carPos.position.y, carPos.position.z]
            dist = math.dist(obs_point, car_point)
            #checking if the overstaking is finished
            if (dist > dth):
                stop_overtaking = False

        if readySwitch == True:
            readySwitch = False
            #mutex release
            lock.release()
            return "R"
        elif countObs == 0:
            #mutex release
            lock.release()
            return "G"
        elif stop_overtaking == True:
            #mutex release
            lock.release()
            return "F"
        else:
            #mutex release
            lock.release()
            return "OO"

#CLASS FOR THE FSM NODE
class FsmNode(Node):

    def __init__(self):
        super().__init__("fsm_node")

        # create the state machine
        self.sm = StateMachine(outcomes=["SHUTDOWN"])

        # add states
        self.sm.add_state("READY", ReadyState(),
                    #ready state possible transitions
                     transitions={"S": "SHUTDOWN",
                                  "G": "GLOBAL",
                                  "R": "READY"})
        self.sm.add_state("GLOBAL", GlobalState(),
                    #global state possible transitions
                     transitions={"R": "READY",
                                  "F": "FOLLOW",
                                  "G": "GLOBAL"})
        self.sm.add_state("FOLLOW", FollowState(),
                    #follow state possible transitions
                     transitions={"OI": "OVERTAKE INSIDE",
                                  "OO": "OVERTAKE OUTSIDE",
                                  "G": "GLOBAL",
                                  "R": "READY",
                                  "F": "FOLLOW"})

        self.sm.add_state("OVERTAKE INSIDE", OIState(),
                    #overtake state inside possible transitions
                     transitions={"G": "GLOBAL",
                                  "R": "READY",
                                  "F": "FOLLOW",
                                  "OI": "OVERTAKE INSIDE"})
        self.sm.add_state("OVERTAKE OUTSIDE", OOState(),
                    #overtake state outside possible transitions
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
        #initializing main node
        rclpy.spin(lifecycle_talker)
        #exception to avoid keyboard interupt error
    except KeyboardInterrupt:
        print('\n Interrupted')
    #desroying main node
    lifecycle_talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()