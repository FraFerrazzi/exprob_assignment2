# Surveillance Robot - Assignment 2
**The second ROS-based assignment for the Experimental Robotics Laboratory course held at the [University of Genoa](https://unige.it/it/).**  

---

## Documentation
Click on the following link https://fraferrazzi.github.io/exprob_assignment2/ to visualize the Sphinx documentation regarding the project.

---

## Introduction

This repository contains ROS-based software architecture that simulates a robot used for surveillance purposes. \
The robot is placed inside a given indoor environment. \
First of all, the robot needs to understand its surroundings by retrieving information from aruco markers placed in the initial room. The information contains the names of the rooms, their coordinates in the environment, and their connections with other locations. \
Once all markers have been detected, the robot's objective is to go around the map, simulating a surveillance task when it gets inside a new location. \
The robot moves autonomously in the environment until the battery state becomes low. When the battery is low, the robot goes to the charging location and, once the battery is full, goes back to the surveillance behavior. \
The program interacts with an ontology to retrieve essential information to achieve the desired behavior. \
A short video shows the execution of the software architecture:



In the main terminal, it is visible the execution and the screen output of the MoveBase and Gmapping algorithms. \
On the top right, the state_machine.py node implements the Final State Machine and shows every transition from one state to another to achieve the desired behavior of the program. The choice was to keep the User Interface not too complex and with few outputs for each state. This was done to avoid too much information on the screen that could confuse the reader. In general, the following steps are shown: the current state that is being executed, the most important actions regarding the state, and the transition to the next state. \
On the right, the other three xterm windows appear once the program is launched. \
The window at the top is the aruco_detection.cpp which shows when an aruco marker is detected and every information related to it. \
The one in the middle represents the move_cam.cpp node and shows the motion of the arm placed on the robot. \
As soon as every marker gets detected, these two nodes are shut down. \
The one at the bottom is the robot_battery_state.py and is responsible for generating the battery_low signal. It manages also the charging action of the robot when the battery is low. \
This video reports the execution of the program when the surveillance2_random.launch is used. \
The only difference concerning the surveillance2_manual.launch is that the GUI of the robot_battery_state node is different compared to the surveillance2_random.launch, and the battery_low signal is randomly generated using the latter mentioned launch 

## How to run

This software is based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already provides the required dependencies. \
If the Docker image is not used, it is necessary to download some essential packages. If a new version of ROS is installed on your machine, the suggestion is to follow the procedure written in this link: https://github.com/EmaroLab/armor/issues/7. \
Instead, if an older version of ROS is present in your machine, please refer to: https://github.com/EmaroLab/armor. \
In both cases, the procedure explained in the README files should be followed and the needed repositories must be cloned and built in your ROS workspace. \
After Armor has been correctly downloaded and built, the current [repository](https://github.com/FraFerrazzi/exprob_assignment2) regarding this project must be downloaded and built in the ROS workspace, typing the following command in the terminal:
```bash
cd <absolute path to your ros_workspace>/src
git clone https://github.com/FraFerrazzi/exprob_assignment2.git
cd ..
catkin_make
```
Once the previous commands have been correctly executed it is possible to launch the program. \
Use the following command to launch the software with a keyboard-base interface for the battery level.
```bash
roslaunch exprob_assignemnt2 surveillance2_manual.launch
```

Use the following command to launch the software with randomized stimulus for the battery level.
```bash
roslaunch exprob_assignemnt1 surveillance2_random.launch
```

Four new terminal windows are going to be opened, making a total of five open windows when the program gets launched. \
One corresponds to the `state_machine.py` GUI which gives visual feedback on what is happening during the execution of the software architecture. One shows a user interface regarding the battery level, controlled by the `robot_battery_state.py` node, which stimuli can be both random or manual depending on the launch file used to run the project.
The other two display the behavior of the `aruco_detection.cpp` and the `move_cam.cpp`. The former allows seeing when a marker is detected and its information, whereas the latter states the correct functioning of the arm's motion. \
The fifth window regards the main terminal where warnings coming from `MoveBase` and `Gmapping` are printed.

---

## Description

The project consists of a software architecture for a surveillance robot located inside a given indoor environment. \
The robot builds the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present in the initial location. After that, the robot starts the patrolling algorithm by relying on autonomous navigation strategies (mapping/planning) and the information collected and stored in the ontology during the previous step. When a room is reached, it performs a complete scan of the room. \
When the battery of the robot is low, it goes to the charging station and waits until it is full again before continuing the surveillance task.

### Environemnt

The environment is characterized by: 4 rooms, 3 corridors, and 7 doors. The following image shows the environment:



The difference between rooms and corridors is that a corridor has more than one door, allowing communication with multiple rooms. \
The ontology is defined using the software [Protèjè](https://protege.stanford.edu) and the [Armor](https://github.com/EmaroLab/armor) ontology manager. This allows having knowledge regarding the structure and information of the environment also in the code. 

### Robot Model

The robot used in this project is a four-wheeled mobile robot. The robot is equipped with a laser scanner placed at its front, essential to detect the environment and possible obstacles around it. \
Also, the robot has an arm with an RGBD camera mounted at the top. The arm can rotate around itself thanks to a continuous joint placed at the base. Also, the camera, used for surveillance and detection purposes, can tilt thanks to a revolute joint which allows getting a better view of the surroundings. \
The robot is reported below:



### Behavior

The robot spawns in a pre-defined initial location, which is part of the 'E' corridor, placed at coordinates: (-6.0,11) concerning the map frame. \  First of all, before moving, the robot scans its surroundings thanks to the camera placed at the top of its arm. The initial location has seven markers that need to be detected to retrieve the environmental information. The markers are detected by making the arm rotate 360 degrees and making the camera tilt upwards and downwards while doing so. \
Once every marker is detected, the ontology is updated and the robot rotates around itself to scan the room thanks to the laser scanner so it does not run into walls. \
Once this first phase is completed, the robot reasons about the environment and decides on the next room that will be visited according to a specific policy later introduced. The goal is reached by relying on autonomous navigation strategies. While the robot moves, thanks to the SLAM algorithm, the map gets updated in real-time and the path planned to reach the goal can be modified depending on possible detected obstacles. \
When the robot reaches the target location, a surveillance task begins which performs a complete scan of the room rotating the base of the arm of 360 degrees. Once the location has been scanned, the reasoner phase is called again, and so on. \
When the battery of the robot becomes low, a charging mechanism is implemented.
The procedure consists in reaching the charging location, which is the 'E' corridor, and simulating a charging task by wasting time in that specific location. \
When the battery is fully charged the robot starts again his surveillance behavior. \
When the robot's battery is not low, the robot moves in the environment according to the following surveillance policy:
- The robot stays mainly in corridors.
- If a reachable room has not been visited for some time, it becomes urgent and the robot should visit it.

The urgency of one location is determined by computing the difference between the last time that the robot moved and the last time that the issued location has been visited. When this difference becomes higher than a threshold, the specific location becomes urgent.


## Assumptions

During the development of the project, some simplified assumptions were done to make an easier model of a surveillance robot:
 - The robot moves in a 2D, pre-defined, known environment without obstacles.
 - Rooms have only one door and corridors have at least two doors. One location can only have one door shared with another location.
 - The charging location is also the initial location of the robot, and it is pre-defined.
 - The number of rooms, corridors, and doors is fixed, only the layout changes.
 - The planner node does not implement a real planner. It creates a path composed of random via points. Between the different via points there is a delay to simulate computation. The planner is used mainly to waste time.
 - The controller node does not implement a real controller. It does not guide the robot nor make the robot follow the path generated by the planner. As the planner, the controller is used to waste time putting a delay between the via points of the path.
 - The battery can become low at any time, and the robot immediately reacts to this event. 
 - The battery low is a signal that does not keep into account the true level of charge of the battery. The signal arrives when a random delay expires.
 - The reasoner state is considered to be atomic. In this way, even if a battery low signal arrives, the ontology query keeps working until it is not done. This decision was made since the robot does not move while it is reasoning and the process lasts few instants, which is neglectable compared to other functions.
 - When a battery low signal comes, all the previous plans and controls are delayed and the reasoning done by the `reasoner()` method is changed by imposing the charge location as next room to reach. In this way, the robot must reason again before going to the next location.
 - The choice of the next location that will be visited keeps into account only temporal stimulus, excluding data that could come from sensors such as cameras or microphones.
 - The robot only simulates a surveillance task, so there is no actual surveillance of the environment.
 - The recharge of the battery does not charge a battery but just wastes time to simulate the task.
 - The timestamp of the robot and the timestamp of the location which the robot visits are updated when the robot gets to the issued location, so when the `controller()` method has done its execution.
 - When the battery status becomes low, the robot reaches the charging location even if it is not reachable at the moment.
 - All the locations inside the map are set to be URGENT when the program is launched. The assumption is that the robot has not moved in a long time, therefore has not visited any location for a period longer than the threshold.
 
## Limitations

Most of the limitations derive from the hypothesis that were done during the implementation of the software architecture. \
The fact that the environment is in 2D constrains the map to be allocated only on one floor, without the possibility of having stairs or slopes. Also, the structure is fixed, so it has a pre-defined number of rooms, corridors, and doors. There would be the need to change a bit the code to maintain a reasonable structure for an indoor environment if one of these numbers needs to be changed. \
The planner and the controller as the surveillance task and the charge of the battery are purely done to waste time, giving limitations to the actual tasks that the robot can perform. For example, the robot can not deduce if there is a person in the room or cannot generate a reasonable path to go from one location to another. \
The robot can only check the urgency of adjacent locations that it can reach in a specific time instant, excluding all the locations that are not reachable in the same time instant. \
The robot states that a location is urgent only based on the timeslot for which the issued location has not been visited, not caring about other possible stimuli.

---

## Software Architecture

The software architecture of the project is further explained in this section. 
First of all the general organization of the repository and the dependencies are pointed out. 
Later on, the general execution of the architecture is discussed with the help of explicative diagrams.

### Repository Organization

This repository contains a ROS package named `exprob_assignment1` that includes the following resources.
 - [CMakeLists.txt](CMakeLists.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [launcher/](launcher/): Contains the configuration to launch this package.
    - [surveillance_manual.launch](launcher/surveillance_manual.launch): It launches this package allowing to manually set when
      the battery state becomes low.
    - [surveillance_random.launch](launcher/surveillance_random.launch): It launches this package with 
      a random-based stimulus for the battery status.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the target and the current points, the feedback, and the results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, the feedback, and the results 
      concerning motion controlling.
 - [scripts/](scripts/): It contains the implementation of each software component.
    - [state_machine.py](scripts/state_machine.py): It implements the final state machine for the software architecture.
    - [robot_battery_state.py](scripts/robot_battery_state.py): It implements the management of the robot's battery level.
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.
 - [utilities/exprob_assignment1](utilities/exprob_assignment1/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](utilities/exprob_assignment1/architecture_name_mapper.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
    - [state_machine_helper.py](utilities/exprob_assignment1/state_machine_helper.py): It contains the methods called in the 
      [state_machine.py](scripts/state_machine.py) node to make the code easier and cleaner to read.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.
 - [doc/](doc/): It contains the files to visualize the Sphinx documentation.
 - [topological_map/](topological_map/): It contains the Tbox of the ontology used in this software
   architecture. It is also the repository in which the complete ontology is saved for debugging purposes.

### Dependencies

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and 
[rospy](http://wiki.ros.org/rospy) for using python with ROS. Rospy allows defining ROS nodes, 
services, and related messages. \
Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define
action servers. In particular, the action services are implemented by using the [SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137) and the [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html). \
The Finite States Machine using the software components provided in this repository is based on [SMACH](http://wiki.ros.org/smach).
It is possible to check the [tutorials](http://wiki.ros.org/smach/Tutorials) related to SMACH, for an overview of its 
functionalities. In addition, it is advised to exploit the [smach_viewer](http://wiki.ros.org/smach_viewer)
node to visualize and debug the implemented Finite States Machine. \
Another dependency is [xterm](https://manpages.ubuntu.com/manpages/trusty/man1/xterm.1.html) which allows opening multiple terminals to have a clear view of what every single node does while the program is running. \
Also, [Armor](https://github.com/EmaroLab/armor) is essential in this project to use the ontology and ensure the desired behavior thought for the software architecture.

## Software Discussion

The software architecture includes four python scripts, which are: `state_machine.py`, `planner.py`, `controller.py`, and `robot_battery_state.py`. There is an additional script: `state_machine_helper.py`, which implements all the methods called in the `state_machine.py` script. The functioning of the program is explained below, using also some explicative diagrams such as:
- State diagram.
- Component diagram.
- Sequence diagram.

### State diagram

The first diagram shows the state machine implemented in the code. The figure helps to understand the logic of the project:

<img src="https://github.com/FraFerrazzi/exprob_assignment1/blob/main/diagrams/state_diagram.drawio.png" width="900">

The state machine is composed of seven states, which are:
- `Build World`: state in which the Tbox of the ontology is loaded and then manipulated to create the desired environment according to the request. This state builds the Abox of the ontology. It can be possible to save the ontology for debugging purposes by uncommenting a few lines of code in the `state_machine_helper.py` script (lines: 282-283).
- `Reasoner`: state that queries the ontology to retrieve essential information used for the surveillance behavior of the robot. The reachable rooms are checked and the robot chooses where to go next based on their urgency or the type of location.
- `Planner`: state that plans a path of random via points going from the current point to a random target point defined inside the environmental limits. This is not an actual planner but just a dummy implementation created to waste time.
- `Controller`: state that receives the path composed of via points defined by the planner and wastes some time for each point defined in the path. This is not an actual controller that makes the robot follow the desired path. It is just a dummy implementation of a real controller.
- `Surveillance`: state in which the robot, once it arrives in a new location, checks the room. This is also a dummy implementation since the state wastes time while it checks if a battery-low stimulus arrives.
- `Reach Charge`: state that makes the robot reach the charging location when its battery becomes low. This state sets as next location that needs to be reached the charging location 'E' and calls the `planner` and `controller` to simulate the motion of the robot.
- `Charge`: state in which the robot charges its battery when it gets low. It is implemented using a blocking service that wastes time simulating the recharge action for a real battery. When the timer expires, the battery of the robot becomes full.

### Component diagram

In the following image the component diagram is reported:

<img src="https://github.com/FraFerrazzi/exprob_assignment1/blob/main/diagrams/component_diagram.drawio.png" width="800">

As shown in the diagram, there are four nodes implemented for the software architecture, plus an additional node (`ARMOR`) which was coded by the [EmaroLab](https://github.com/EmaroLab) group. \ 
The latter node is essential to guarantee the communication between the ontology, developed with the software [Protèjè](https://protege.stanford.edu), and the ROS scripts created for this project. \
The other scripts are briefly described below:
- `state_machine.py`: as can be seen in the component diagram, this node is the core of the whole architecture. Every other node later explained communicates with this script to ensure the correct behavior of the software. In this node, the final state machine of the project is implemented, which initializes and manages the earlier mentioned states: `Build World`, `Reasoner`, `Planner`, `Controller`, `Surveillance`, `Reach Charge`, and `Charge`. To support this node, a helper class was created, which is present in the `state_machine_helper.py` node that implements some methods called inside the `state_machine.py`.
- `robot_battery_state.py`: this node is responsible for managing the robot's battery level. It can give a battery low signal in two ways: randomly after a delay, manually waiting for the user's input. When the battery becomes low, a service is called to recharge the battery which is also implemented in this node. The communication with the `state_machine.py` node is possible thanks to the `SetBool.srv` standard service.
- `planner.py`: it is a node that, given the current position and the target position, returns a path of random via points. It is not an actual planner since the path has no physical meaning but it is just done to waste time. Communication with the `state_machine.py` node is possible thanks to the `Plan.action` action service.
- `controller.py`: it is a node that, given the path of via points created by the planner, simulates the motion of the robot based on a random delay between each point. It is not an actual controller since it does not control the movement of the robot but it is just done to waste time. Communication with the `state_machine.py` node is possible thanks to the `Control.action` action service.

For a better overview of the scripts, I suggest going back to the beginning of this README file and checking the Sphinx documentation. \
The nodes `robot_battery_state.py`, `planner.py`, and `controller.py` were previously implemented by Professor [buoncubi](https://github.com/buoncubi), foundable in the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository. The names of the scripts are respectively: `robot_states.py`, `planner.py`, and `controller.py`. \
I have made some changes to the previously mentioned scripts to better fit the current software architecture.

### Sequence diagram

The state diagram focuses on the timing of the communication between the different nodes. \
The beginning corresponds to the instant in which the software architecture is launched. The diagram is shown the case in which `battery_low = False` for one full execution cycle and becomes `battery_low = True` after the `state_machine.py` retrieves from the ARMOR service the information regarding the new location and the updated timestamp.

<img src="https://github.com/FraFerrazzi/exprob_assignment1/blob/main/diagrams/sequence_diagram.drawio.png" width="900">

The first action done during execution is creating the Abox of the ontology, achieved by the node `state_machine.py` which sends some requests to the ARMOR service and waits until the environment is correctly created. \
When the world is ready, the `state_machine.py` node queries the ontology to retrieve essential information regarding the location status (i.e. URGENT, ROOM, CORRIDOR, LOCATION) of the reachable room to allow the reasoner method to implement the surveillance policy of the robot. \
Once the next location is chosen, the `state_machine.py` sends a request to the `planner.py` giving the coordinates of the current position of the robot and the next position. The response is the path composed of via points to go from the current to the next location. \
At this point, the `controller.py` makes sure that the location will be reached. The request is sent by the `state_machine.py`, which is the path provided by the planner, and the response is the target location once the robot reaches it. \
The `state_machine.py` queries again the ontology to update the new position of the robot and to update the timestamp of the location and of the robot itself. \
The sequence of the loop is always the same until a `battery_low = True` signal is issued. When this happens, the robot gets to the charging location by sending a request to the `planner.py` and `controller.py` in the same way described above but imposing the charging location as the next location. \
Once the robot is ready to charge itself, a charging request is sent to the `robot_battery_state.py` node, which is the same one that published the `battery_low` signal. When the robot is fully charged, the response setting the `battery_low = False` is received by the `state_machine.py`

---

## ROS Parameters

This software requires the following ROS parameters.
 
 - `config/environment_size`: It represents the environment boundaries as a list of two floating
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `test/random_plan_points`: It represents the number of via points in a plan. It is a list of n
   elements, where n is a random value inside the interval: `[min_n, max_n]`. The chosen random value 
   within such an interval defines the length of each plan.

 - `test/random_plan_time`: It represents the time required to compute the next via point by the 
   planner. The time is chosen randomly inside the `[min_time, max_time]` interval, which is in seconds. 

 - `test/random_motion_time`: It represents the time required to reach the next via point. The time 
   is chosen randomly inside the `[min_time, max_time]` interval, which is in seconds.  
   
 - `test/random_sense/battery_charge`: It indicates the time necessary to recharge the battery of the 
   robot. The time is chosen randomly inside the `[min_time, max_time]` interval, which is in seconds. 
   
 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or 
   deactivates (`False`) the random-based generation of stimulus for the battery level.
   If this parameter is `True`, then the parameter below is also 
   required. If it is `False`, the parameter below is not used.
 

In addition, the `surveillance_random.launch` also requires the following parameter. This 
occurs because `test/random_sense/active` has been set to `True`.

 - `test/random_sense/battery_time`: It indicates the time that needs to elapse to have a random low 
   battery stimulus. The time is chosen randomly inside the `[min_time, max_time]` interval, which is in seconds.

---

## Possible Improvements

The improvements regarding this software architecture would be to solve some limitations present in the system, by making more realistic assumptions. \
A list of possible ideas is reported below:
- Put sensors on the robot. In this way, it could be possible to make it work in a 3D environment which does not have to be pre-determined. Also, if the robot is equipped with the correct sensors, it could be possible to perform a surveillance action and not just simulate it.
- The robot can know its position and the position that it needs to reach in the environment. In this way, the `planner()` and the `controller()` methods could give a reasonable path to go from the current location to the target location by a set of via-points, and controlling the wheels of the robot in such a way that it follows the desired path.
- Provide a real battery to the robot. In this way, it could be possible to implement a charging action once the battery is low.
- Create a simulation environment in which the robot can be tested to see the correctness of the algorithms and test its possibilities in different maps, trying also on outdoor environments.

---

## Author
Author: *Francesco Ferrazzi* \
Student ID: *s5262829* \
Email: *s5262829@studenti.unige.it*
