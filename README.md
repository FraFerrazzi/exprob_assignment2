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



In the main terminal, it is visible the execution and the screen output of the `MoveBase` and `Gmapping` algorithms. \
The main focus goes to the `gazebo` simulator, which is placed in the middle. \
On the top right, the `state_machine.py` node implements the Final State Machine and shows every transition from one state to another to achieve the desired behavior of the program. The choice was to keep the User Interface not too complex and with few outputs for each state. This was done to avoid too much information on the screen that could confuse the reader. In general, the following steps are shown: the current state that is being executed, the most important actions regarding the state, and the transition to the next state. \
On the left, the other three xterm windows appear once the program is launched. \
The window at the top is the `aruco_detection.cpp` which shows when an aruco marker is detected and every information related to it. \
The one in the middle represents the `move_cam.cpp` node and shows the motion of the arm placed on the robot. \
As soon as every marker gets detected, these two nodes are shut down. Now, the `Rviz` screen is partially visible, to show mainly what the robot sees through its camera. \
The one at the bottom is the `robot_battery_state.py` and is responsible for generating the battery_low signal. It manages also the charging action of the robot when the battery is low. \
This video reports the execution of the program when the surveillance2_manual.launch is used, therefore the user must decite when the low battery signal is issued by typing `l` on the `robot_battery_state.py` node. \
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

<img width="900" alt="Environment" src="https://user-images.githubusercontent.com/91314392/218328770-f5aaae53-38c5-4afb-9647-6c60660ac8e0.png">

The difference between rooms and corridors is that a corridor has more than one door, allowing communication with multiple rooms. \
The ontology is defined using the software [Protèjè](https://protege.stanford.edu) and the [Armor](https://github.com/EmaroLab/armor) ontology manager. This allows having knowledge regarding the structure and information of the environment also in the code. 

### Robot Model

The robot used in this project is a four-wheeled mobile robot. The robot is equipped with a laser scanner placed at its front, essential to detect the environment and possible obstacles around it. \
Also, the robot has an arm with an RGBD camera mounted at the top. The arm can rotate around itself thanks to a continuous joint placed at the base. Also, the camera, used for surveillance and detection purposes, can tilt thanks to a revolute joint which allows getting a better view of the surroundings. \
The robot is reported below:

<img width="500" alt="RobotModel" src="https://user-images.githubusercontent.com/91314392/218328774-9ee5c2a4-2c1a-45d7-94cc-37f690e1d171.png">

### Behavior

The robot spawns in a pre-defined initial location, which is part of the 'E' corridor, placed at coordinates: (-6.0,11) concerning the map frame. \  
First of all, before moving, the robot scans its surroundings thanks to the camera placed at the top of its arm. The initial location has seven markers that need to be detected to retrieve the environmental information. The markers are detected by making the arm rotate 360 degrees and making the camera tilt upwards and downwards while doing so. \
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

---

## Assumptions

During the development of the project, some simplified assumptions were done to make an easier model of a surveillance robot:
 - The robot moves in a pre-defined environment without moving obstacles, therefore it does not change over time.
 - Rooms have only one door and corridors have at least two doors. One location can only have one door shared with another location.
 - The charging location is also the initial location of the robot, and it is pre-defined.
 - The battery can become low at any time, and the robot immediately reacts to this event. 
 - The battery low is a signal that does not consider the battery's charge level. The signal arrives when a random delay expires.
 - The reasoner state is considered to be atomic. In this way, even if a battery low signal arrives, the ontology query keeps working until it is not done. This decision was made since the robot does not move while it is reasoning and the process lasts a few instants, which is neglectable compared to other actions.
 - When a battery low signal comes, all the previous plans and controls are delayed and the reasoning done by the `reasoner()` method is changed by imposing the charge location as the next room to reach. As soon as the battery is full, the robot must reason again before getting to the next goal.
 - The choice of the next location that will be visited keeps into account only temporal stimulus, excluding data that could come from sensors such as cameras or microphones.
 - The robot scans the target location as a surveillance task, making the camera rotate 360 degrees around itself when it reaches the goal.
 - The recharge of the battery does not charge a battery but just wastes time to simulate the task.
 - The timestamp of the robot and the timestamp of the location which the robot visits are updated when the robot gets to the issued location, so when the `go_to_goal()` method has done its execution.
 - When the battery status becomes low, the robot reaches the charging location even if it is not reachable at the moment.
 - All the locations inside the map are set to be URGENT when the program is launched, except for the initial location 'E'. The assumption is that the robot has not moved in a long time, therefore has not visited any location for a period longer than the threshold.
 - All the aruco markers present in the environment are located in the initial room where the robot spawns. Using the information coming from these markers is possible to obtain the full representation and every information concerning the environment. 
 - The color defining the environment, i.e. walls and blocks, was changed to white. This allows easier detection of the aruco markers.
 
## Limitations

Most of the limitations derive from the hypothesis that were done during the implementation of the software architecture. \
The environment is allocated only on one floor, without the possibility of having stairs or slopes. The structure is fixed, so it has a pre-defined number of rooms, corridors, and doors, and there are no moving entities in the environment. The map is static, therefore easier to manage, but this does not allow to have particular stimuli that could trigger an alert signal while the robot is surveilling the environment, e.g. a moving person. \
The charge of the battery is done to waste time, giving limitations to the actual task that the robot could perform. \
The robot can only check the urgency of adjacent locations that it can reach in a specific time instant, excluding all the locations that are not reachable in the same time instant. \
The robot states that a location is urgent only based on the timeslot for which the issued location has not been visited, not caring about other possible stimuli. \
After every aruco marker has been detected in the initial phase, the nodes for the aruco detection are shut down, therefore if other arucos were placed in the environment the robot would not have the possibility to detect them. \
The model of the robot in the simulation does not concern a real mobile robot. It is just a simplification used for test purposes.

---

## Software Architecture

The software architecture of the project is further explained in this section. 
First of all the general organization of the repository and the dependencies are pointed out. 
Later on, the general execution of the architecture is discussed with the help of explicative diagrams.

### Repository Organization

This repository contains a ROS package named `exprob_assignment2` that includes the following resources.
 - [CMakeLists.txt](CMakeLists.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [make.bat](make.bat): Used to generate the documentation of the code.  
 - [index.rst](index.rst): Used to generate the documentation of the code.  
 - [conf.py](conf.py): Used to generate the documentation of the code. 
 - [Doxyfile.in](Doxyfile.in): Used to generate the documentation of the code.  
 - [Makefile](Makefile): Used to generate the documentation of the code.  
 - [build/](_build/): Contains the files generated when launching `make_html` for the documentation.
 - [launch/](launch/): Contains the configuration to launch this package.
    - [surveillance2_manual.launch](launch/surveillance2_manual.launch): It launches this package allowing to manually set when
      the battery state becomes low.
    - [surveillance2_random.launch](launch/surveillance2_random.launch): It launches this package with 
      a random-based stimulus for the battery status.
 - [config](config/): Contains the files for configuring the system.
    - [motors_config.yaml](config/motors_config.yaml): Configuration of the controllers for the robot's joints.
    - [robot_config.rviz](config/robot_config.rviz): Configuration of the rviz simulation.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [RoomConnection.msg](msg/RoomConnection.msg): It is the message defining the connections for each location.
 - [srv/](srv/): It contains the definition of each service used by this software.
    - [ArmInfo.srv](srv/ArmInfo.srv): It allows deciding when the robot's arm should stop rotating.
    - [RoomInformation.srv](srv/RoomInformation.srv): It sends the ID of the detected marker and receives environmental information.
    - [WorldInit.srv](srv/WorldInit.srv): It sends the environmental information and receives the status of the communication.
 - [param](param/): Contains the parameter files `.yaml` used by `Move Base` to make the robot move in the environment.
 - [scripts/](scripts/): It contains the implementation of the python nodes.
    - [state_machine.py](scripts/state_machine.py): It implements the final state machine for the software architecture.
    - [robot_battery_state.py](scripts/robot_battery_state.py): It implements the management of the robot's battery level.
 - [utilities/exprob_assignment2](utilities/exprob_assignment2/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](utilities/exprob_assignment2/architecture_name_mapper.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in the scripts.
    - [state_machine_helper.py](utilities/exprob_assignment2/state_machine_helper.py): It contains the methods called in the 
      [state_machine.py](scripts/state_machine.py) node to make the code easier and cleaner to read.
 - [src/](src/): It contains the implementation of the c++ nodes.
    - [aruco_detection.cpp](src/aruco_detection.cpp): It allows the detection of aruco markers placed in the environment.
    - [move_cam.cpp](src/move_cam.cpp): It allows to move the arm of the robot for markers detection purposes.
    - [marker_server.cpp](src/marker_server.cpp): It allows getting the environmental information from the markers.
 - [urdf/](urdf/): It contains the files to generate the model of the robot.
    - [materials.xacro](urdf/materials.xacro): Defines the colors used for each element of the robot.
    - [robot.xacro](urdf/robot.xacro): Defines the structure of the robot.
    - [robot.gazebo](urdf/robot.gazebo): Defines the visualization of the robot in gazebo.
 - [worlds/](worlds/): It contains the simulation environment.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.
 - [docs/](docs/): It contains the files to visualize the Sphinx documentation.
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
To detect the aruco markers, the sotware architecture relies on [OpenCV](https://opencv.org). Some documentation can be found clicking [here](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html).

## Software Discussion

The software architecture includes two python scripts, which are: `state_machine.py` and `robot_battery_state.py`. There is an additional script: `state_machine_helper.py`, which implements all the methods called in the `state_machine.py` script. There are also other three additional nodes implemented in c++, which are: `aruco_detection.cpp`, `move_cam.cpp`. and `marker_server.cpp`. The latter one was implemented by Prof. Recchiuto. \
The functioning of the program is explained below, using also some explicative diagrams such as:
- State diagram.
- Component diagram.
- Sequence diagram.

### State diagram

The first diagram shows the state machine implemented in the code. The figure helps to understand the logic of the project:

![StateDiagram](https://user-images.githubusercontent.com/91314392/218321874-0ef1d6cf-ff24-4270-a734-4b2289e8e759.png)

The state machine is composed of seven states, which are:
- `Build World`: state in which the Tbox of the ontology is loaded and then manipulated to create the desired environment according to the request. This state builds the Abox of the ontology. It can be possible to save the ontology for debugging purposes by uncommenting a few lines of code in the `state_machine_helper.py` script (lines: 282-283).
- `Reasoner`: state that queries the ontology to retrieve essential information used for the surveillance behavior of the robot. The reachable rooms are checked and the robot chooses where to go next based on their urgency or the type of location.
- `Motion`: state that plans a path from the current position of the robot to the goal position. This is possible thanks to the `Move Base` algorithm, which is also capable of modifying the path if new obstacles are detected between the robot and the goal. It also controls the robot to make it follow the generated path.
- `Surveillance`: state in which the robot, once it arrives in a new location, scans the room. This a dummy implementation since the robot checks the room rotating the camera of 360 degrees but there is not an actual implementation that could react to stimuli.
- `Reach Charge`: state that makes the robot reach the charging location when its battery becomes low. This state sets as next location that needs to be reached the charging location 'E' and calls the `go_to_goal()` method used also in the `Motion` state to make the robot reach the desired target.
- `Charge`: state in which the robot charges its battery when it gets low. It is implemented using a blocking service that wastes time simulating the recharge action for a real battery. When the timer expires, the battery of the robot becomes full.

### Component diagram

In the following image the component diagram is reported:

![ComponentDiagram](https://user-images.githubusercontent.com/91314392/218321900-76f5e139-15c3-4822-b1ef-36b1feac87b8.png)

As shown in the diagram, there are five nodes implemented for the software architecture, plus additional nodes active, which are:
- `ARMOR`: which was coded by the [EmaroLab](https://github.com/EmaroLab) group, used to update and query the ontology. 
- `gazebo`: ROS simulation environment.
- `move_base`: Action server used to plan and control the robot for reaching a desired goal described by Cartesian coordinates.
- `slam_gmapping`: Simultaneous localization and mapping algorithm.
The other scripts are briefly described below:
- `state_machine.py`: as can be seen in the component diagram, this node is the core of the whole architecture. Almost every node in the architecture communicates with this script to ensure the correct behavior of the software. In this node, the final state machine of the project is implemented, which initializes and manages the earlier mentioned states: `Build World`, `Reasoner`, `Motion`, `Surveillance`, `Reach Charge`, and `Charge`. To support this node, a helper class was created, which is present in the `state_machine_helper.py` node that implements some methods called inside the `state_machine.py`.
- `robot_battery_state.py`: this node is responsible for managing the robot's battery level. It can give a battery low signal in two ways: randomly after a delay, manually waiting for the user's input. When the battery becomes low, a service is called to recharge the battery which is also implemented in this node. The communication with the `state_machine.py` node is possible thanks to the `SetBool.srv` standard service.
- `aruco_detection.cpp`: This is a client node used to detect the aruco markers placed in the initial location. Every time a new ID belonging to a marker is detected, it is sent to the `marker_server.cpp` and its location's information are retrieved. The same information is sent to the `state_machine.py`, later used to update the ontology. When all the markers have been detected, a request is sent to the `move_cam.cpp`. When a response is received this node is shut down.
- `move_cam.cpp`: this node allows the motion of the arm of the robot. The base link is rotated of 360 degrees meanwhile the camera is tilted downwards and upwards to detect all the markers in the initial location. When all of them are detected, a request from the `aruco_detection.cpp` arrives. The arm is moved to the home position and the node gets shut down.
- `marker_server.cpp`: it is a node implemented by Professor [Recchiuto](https://github.com/CarmineD8). It is used to retrieve the information of a specific location corresponding to an ID coming from an aruco marker.

For a better overview of the scripts, I suggest going back to the beginning of this README file and checking the Sphinx documentation. \
The nodes `robot_battery_state.py` and `marker_server.cpp` were previously implemented by Professor [Buoncompagni](https://github.com/buoncubi), foundable in the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, and Professor [Recchiuto](https://github.com/CarmineD8). \
I have made a few changes to the `robot_battery_state.py` script to better fit the current software architecture.

### Sequence diagram

The state diagram focuses on the timing of the communication between the different nodes. \
The beginning corresponds to the instant in which the software architecture is launched. The diagram is shown the case in which `battery_low = False` for one full execution cycle and becomes `battery_low = True` after the `Surveillance` state.

![SequenceDiagram](https://user-images.githubusercontent.com/91314392/218325703-69d63953-d1b8-4395-975d-18a54dabfa28.png)

As soon as the architecture is launched, every node shown in the diagram starts running. \
For the whole simulation time, `gazebo` keeps sending `/scan` and `/tf` information to the `slam_gmapping` algorithm to update the map and allow the robot to understand its surroundings. \
At the same time the `joint state` are sent to the `move_cam.cpp` to get the state of the robot arm's joints and allow them to move thanks to the `joint command`. This action is done recursively until a request from the `aruco_detection.cpp` is received. \
Also, `aruco_detection.cpp` acquires `camera image` from `gazebo` and processes them to acquire aruco markers. When markers are detected, the `marker ID` is sent to the `marker_server.cpp` and the environmental information are retrieved. Every time this happens, the received data is sent to the `state_machine.py` which will be used later on to build the ontology of the environment. \
As soon as all markers have been detected, the `Aruco detection` phase ends and the `Build map` starts.
The first action of the `Build map` phase is to load the ontology and create its Abox, achieved by the node `state_machine.py` which sends some requests to the ARMOR service and waits until the environment is correctly created. \
At this point, the `Surveillance behavior` starts. \
When the world is ready, the `state_machine.py` node queries the ontology to retrieve essential information regarding the location status (i.e. URGENT, ROOM, CORRIDOR, LOCATION) of the reachable room to allow the reasoner method to implement the surveillance policy of the robot. \
Once the next location is chosen, the `state_machine.py` sends the `goal coordinates` to `move_base` to reach the target. As soon as the action is completed the `state_machine.py` queries again the ontology to update the new position of the robot and to update the timestamp of the location and of the robot itself. \
The sequence of this phase is always the same until a `battery_low = True` signal is issued. When this happens, the robot gets to the charging location thanks to `move_base` in the same way described above but imposing the charging location as the goal location. \
Once the robot is ready to charge itself, a charging request is sent to the `robot_battery_state.py` node, which is the same one that published the `battery_low` signal. When the robot is fully charged, the response setting the `battery_low = False` is received by the `state_machine.py`. \
As soon as the battery is full, the `Surveillance behavior` starts again from the reasoning of the ontology and the robot stays in this phase until a new `battery_low = True` signal is issued.

---

## ROS Parameters

This software requires the following ROS parameters.
   
 - `test/random_sense/battery_charge`: It indicates the time necessary to recharge the battery of the 
   robot. The time is chosen randomly inside the `[min_time, max_time]` interval, which is in seconds. 
   
 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or 
   deactivates (`False`) the random-based generation of stimulus for the battery level.
   If this parameter is `True`, then the parameter below is also 
   required. If it is `False`, the parameter below is not used.
 

In addition, the `surveillance2_random.launch` also requires the following parameter. This 
occurs because `test/random_sense/active` has been set to `True`.

 - `test/random_sense/battery_time`: It indicates the time that needs to elapse to have a random low 
   battery stimulus. The time is chosen randomly inside the `[min_time, max_time]` interval, which is in seconds.

---

## Possible Improvements

The improvements regarding this software architecture would be to solve some limitations present in the system, by making more realistic assumptions. \
A list of possible ideas is reported below:
- Make the environment dynamic to test the robot's performance, allowing the robot to perform a real surveillance task, and giving alarm signals every time a moving entity is detected.
- Define the model of the robot more realistically, maybe using an existing model already implemented by developers that could be used also in real life.
- Provide a real battery to the robot. In this way, it could be possible to implement a charging action once the battery is low.
- Change the controller of the robot from a `Planar Move` controller to a more realistic one, e.g. a `Skid Drive` controller.

---

## Author
Author: *Francesco Ferrazzi* \
Student ID: *s5262829* \
Email: *s5262829@studenti.unige.it*
