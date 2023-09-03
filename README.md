# F1TENTH STRATEGY NODE

[![ROS2](https://user-images.githubusercontent.com/80155305/215286774-4c54bc8b-1ded-4bbd-991e-bc269bb6542f.png)](https://www.ros.org/)



## Description
This is a real time application for racing strategy of F1Tenth vehicle.
The strategic node implements two finite state machines.
The first one has been implemented using the Managed Nodes feature of ROS2. 
This first state machine has to manage the node lifecycle, allowing it to be configured, activated and deactivated from the outside.
The second one is the strategic one: it has been implemented through the use of Yasmin library.
The node receives as input from the ROS2 network the position of the vehicle (of type Odometry) and a vector containing the positions of possible obstacles (of type PoseArray), then it decides the current state of the vehicle.

## Libraries


- [Yasmin](https://github.com/uleroboticsgroup/yasmin) 
- [Lifecycle implementation](https://github.com/wesleysliao/ros2_lifecycle_py/) 


## Installation

Strategy node requires [ROS2](https://docs.ros.org/en/foxy/Installation.html) foxy

Move to your workspace folder and run
```sh
git clone https://github.com/l-pedretti/f1tenth_strategy_node
```

Then to clone Yasmin

```sh
cd f1tenth_strategy_node/src
git clone https://github.com/uleroboticsgroup/simple_node.git
git clone https://github.com/uleroboticsgroup/yasmin.git

cd yasmin
pip3 install -r requirements.txt
```

Then build
```sh
cd f1tenth_strategy_node
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Execution

From f1tenth_strategy_node folder, open 5 new terminals: 
Fsm node:
```sh
source install/setup.bash
ros2 run f1tenth_strategy_node fsm
```
Car publisher:
```sh
source install/setup.bash
ros2 run f1tenth_strategy_node pub_car
```
Obstacle publisher:
```sh
source install/setup.bash
ros2 run f1tenth_strategy_node pub_obstacle
```
Fsm subscriber: 
```sh
source install/setup.bash
ros2 run f1tenth_strategy_node sub_fsm
```
Commands executor:
```sh
source install/setup.bash

ros2 service call /lc_talker/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: "dth", value: {type: 2, double_value: 10.0}}]}"

ros2 service call /lc_talker/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: "tnc", value: {type: 2, double_value: 10.0}}]}"

ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 1, label: configure}}"

ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 3, label: activate}}"

ros2 service call /fsm_changeState tutorial_interfaces/srv/ChangeState "{state: G}"
```

#### Deactivate
```sh
ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 4, label: deactivate}}"
```

#### Clean up
```sh
ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2, label: cleanup}}"
```


## License

[GNU GPLv3](https://www.gnu.org/licenses/gpl-3.0.html#license-text) 



  
