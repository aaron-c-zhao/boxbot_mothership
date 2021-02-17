# Boxbot - Carton infeed robot

** Migrated from gitlab **


[[_TOC_]]

## Repository structure

The repository consists of mainly four parts:

* **Arduino led strip**:  Arduino sketches for led strip control
* **Calculations Python**: Python scripts which calculate the physical properties of the robot
* **boxbot-go**: Go programs which implement the entire Robot Application System (RAS), parts of the Robot Control System (RCS), and several controller drivers
* **rcs**: C++ programs which implement most of the Robot Control System (RCS) and hardware drivers

More specific information please refer to the structure graph and the comments below. For in-depth description, please you can find it  at [RAS implementation](#Boxbot-go) and [RCS implementation](#Rcs).  
Some directions are omitted from the structure graph, which means they can be ignored as they are either language specific files,
or are no longer actively used.

```shell
.
├── Arduino led strip    # arduino sketches that control the led strip
├── Calculations Python	# calculation scripts for physical properties of the robot
├── boxbot-go	# the boxbot-go project
│   ├── internal # package containing internal constants
│   ├── pkg
│   │   ├── event # persistent event helper
│   │   ├── gripper_serial # gripper controller driver for initial gripper algorithm
│   │   ├── gripper_serial_v2 # gripper controller driver to improved gripper algorithm
│   │   ├── gripper_solver # implementation of the gripper calculations
│   │   ├── hardware # wrapper for the hardware service
│   │   ├── ik_solver # prototype for custom kinematics solver
│   │   ├── lang # package containing all messages
│   │   ├── log # logger implementation
│   │   ├── msgs # all message definitions
│   │   ├── node # several helpers and interfaces for ROS nodes
│   │   ├── path_solver # prototype for custom path planning (moveit backup)
│   │   ├── pins # pin handler used to ease interacting with GPIO pins on the raspberry pi
│   │   ├── rcs_link # wrapper for interacting with RCS
│   │   ├── robot_move # wrapper for interacting with the moveit interface
│   │   ├── servo_serial # servo controller driver
│   │   ├── srvs
│   │   │   ├── boxbot_autonomous # service definitions for the RCS FSM
│   │   │   ├── srv_hardware # service definitions for hardware services
│   │   │   ├── srv_ras # service definitions for ras services
│   │   │   └── srv_rcs # service definitions for rcs services
│   │   ├── stepper_serial # stepper controller driver
│   │   ├── ui # wrapper for interacting with the ui node
│   │   ├── watchdog # wrapper for interacting with the watchdog node
│   │   └── ws # websocket server used for the management interface
│   └── services
│       ├── ras
│       │   ├── base # the RAS base node
│       │   ├── interface # the RAS interface node (controls GPIO pins)
│       │   ├── logging # the RAS logging node
│       │   ├── mock_interface # a mock node for the interface node, in order to facilitate testing without hardware
│       │   ├── notifications # the RAS notifications node
│       │   ├── ui # a Vue project containing both the maintenance and the operating interface
│       │   └── watchdog # the RAS watchnode node
│       └── rcs
│           ├── rcs_autonomous # the RCS FSM
│           ├── rcs_control # prototype for movement controller (ros_control backup)
│           ├── servo_motor # servo driver
│           ├── stepper_motor # stepper driver
├── doc	
├── images_videos
└── rcs	# ROS based RCS system
    ├── catkin_ws
    │   ├── doc
    │   └── src
    │       ├── boxbot_arm_ikfast_plugin	# custom IKFast plugin for Moveit
    │       ├── boxbot_autonomous	# provides interface for the RAS system
    │       ├── boxbot_control	# controller manager configuration package for Gazebo simulation
    │       ├── boxbot_cv	# computer vision 
    │       ├── boxbot_description	# URDF description of the robot
    │       ├── boxbot_hardware	# implements hardware_interface of ros_control
    │       ├── boxbot_moveit_config	# Moveit configuration package
    │       ├── boxbot_odrive_control	# Odrive service message definition
    │       └── boxbot_stepper_control	# stepper motor control message definition
    └── hardware	# drivers for the hardware
        ├── encoder_reader # program to read encoder data
        ├── gripper_program # program which runs on the gripper
        ├── motor_driver # program which runs the motor driver
        └── servo_driver # program which runs the servo driver
```

## 1. Boxbot-go
The RAS system consists of several nodes, and helper wrappers. Furthermore, the interface -implemented in Vue + Typescript- is also contained in boxbot-go.
Besides that, several components of the RCS system are also implemented in boxbot-go, such as the FSM, several controller drivers and prototypes for custom kinematics, path planning, and control loops. 

The boxbot-go project follows a [monorepo structure](https://github.com/flowerinthenight/golang-monorepo).

### RAS Nodes
All nodes discussed in the Detailed Design Report have been implemented:
* Base node: This node includes all business logic of the RAS subsystem, which includes the base FSM. 
The full FSM can be found by simply starting the node, it will print the entire FSM in graphviz format.
* Interface node: This node interfaces directly with the hardware (GPIO pins) on the raspberry pi.
* Logging node: This node ensures all logging events are properly logged, and can be easily retrieved from the database.
* Mock interface node: This node mocks the interface node, which means the entire system can be tested without being connected to the appropriate hardware.
* Notification node: This node does not have a lot of functionality in our prototype, but serves as an entry point for external communication.
* Watchdog node: This node continuously monitors the other nodes (both RAS and RCS nodes), and notifies the base node of any discrepancies. Furthermore, this node also keeps the hardware watchdog alive.

All nodes have a configuration file, which can be found in the config.go file in the appropriate service. 
As these configuration options use environment variables, the settings can be changed by setting environment variables on the
executing computer.

Each node can be run as a normal Go program. This means they can be compiled, and executed just like normal Go programs.
Alternatively, the nodes can be launched in a suitable IDE (such as [GoLand](https://www.jetbrains.com/go/)) 
by using the provided run configurations.

### User Interface
There exist two main user interfaces; one for the operator and one for the maintenance technician. Both are implemented with
Vue 2 + Vuetify + Typescript, and can be found in the RAS ui service. Initially, it was planned for these to directly use ROS,
but this proved to be cumbersome, so it was decided to use regular HTTP requests in combination with websockets. 

The design for both interfaces can be found in the Detailed Design Report. 

#### Operator interface
In the prototype the operator interface is fully functional, and uses websockets to provide the operator with the latest 
information. This interface is relatively simple, and has only one main component, which can be found in the operating components
Furthermore, this interface is fully incorporated in the maintenance interface, for the sake of simplicity. 

#### Maintenance interface
Unfortunately, this interface is not fully functional in the prototype, as it was a low priority requirement. 
The frontend is mostly done, and the backend is fully functional. It uses both HTTP requests and websockets to provide
all information. To be more specific, websockets are used to provide technicians with live notifications from the robot, 
while regular HTTP requests are used for everything else. In order to reduce duplicate code, the HTTP server translates every
request to its ROS equivalent, essentially forming a bridge between ROS and the regular web, while remaining maintainable.

In order for the maintenance interface to be fully functional, actual API calls will have to be made in the Typescript code. 
Once that's done, the interface is completed.

### Controller drivers
All direct hardware control with realtime constraints (motor control) and/or several combined sensors (gripper) are implemented on microcontrollers, ESP8266s in this case.
These microcontrollers have to be controlled by the RCS system, so there should be a bridge between ROS and the microcontrollers. 
For ease of use, and to use readily available USB cables, it was decided to use a plain serial connection to each microcontroller.

There exist controller drivers for the stepper motors, servo motor, and gripper. The exact protocol for each of these can be found 
in the handover document, or by looking at the code. The drivers can be launched in a suitable IDE (such as [GoLand](https://www.jetbrains.com/go/)) 
by using the provided run configurations. The drivers can also be compiled, and executed. 

Each driver has a configuration, which can be found in the config.go file in each service. As these configurations are 
environment settings, they can also be set by changing environment variables on the executing computer without needing 
a recompile. The default settings can also be found in aforementioned files. 

### RCS FSM
Initially, this FSM would be created in C++, using the Boost library. However, this proved to be too complicated and bloated
for a simple FSM. Since the RAS was already implemented in Go at that point, it was decided to also implement the RCS
FSM in Go, to save time. 

This node works identical to the RAS base node, with regard to running and configuring it. Just as with the RAS FSM, the 
RCS FSM prints itself in Graphviz format on startup, which can be used to inspect the FSM in detail.

### Movement prototypes
At some point during the project, it looked like MoveIt and ros_control were not going to work within the given timeframe.
There were several issues with the kinematics solver and controller, and both internet and the technical coaches were unable 
to provide any help. Therefore, it was decided to create a prototype as a backup. This prototype would handle the inverse kinematics,
path planning, and control loops.

After some tests, it became clear the backup worked, and would be a viable option. At around the same time, we got
MoveIt + ros_control working, so the movement prototypes were no longer necessary. However, it was decided to keep the movement
prototypes, as they have been very useful in thoroughly understanding the actions required to move the robot, and they can still
serve as a base or example for a custom motion planner. 

The path planner is very simple -but it works-, as it does not attempt to create a linear path for the end-effector in world space.
Instead, it will simply make a motion plan which ensures all joints will arrive at their targets at the same time. It
achieves this by calculating all trajectories in time-optimal mode (keeping the joint limits in mind), then using the slowest 
time and recomputing the trajectories in fixed-time mode. Since singularities are not likely to happen in this robot,
a smooth path can be planned by simple interpolating the linear path. For this purpose [RH Taylor's bounded deviation joint paths](https://pdfs.semanticscholar.org/e01a/58608f4e68f31c7b9e7cdbddceae645727bb.pdf)
can be used, which provides a simple, yet elegant solution to selecting intermediate targets.

Images of the path planner in action can be found in the pkg > path_solver directory.

## 2. Rcs
The RCS system mainly implements three components, namely Moveit, ros_control, and the computer vision. They will be briefly introduced in the following sections.

### Moveit

[Moveit](https://moveit.ros.org/) is the framework that integrates necessary functionalities for robot manipulating, such as Inverse Kinematic(IK) solver, Motion Planner and Collison Checking. In this project, Moveit was adopted to control the robot arm reaching a designated position smoothly. Three package are involved within this process:

1. [boxbot_arm_ikfast_plugin](./rcs/catkin_ws/src/boxbot_arm_ikfast_plugin): Since our robot is customized and has less than 6 Degree Of Freedom(DOF), the default kinematic solvers(KDL and Track-it) cannot solve inverse kinematic for our robot. Thus we have to generate our own IKFast Ik solver plugin from the URDF description of our robot. This package contains the resultant plugin. This package can not be launched or run individually, it should *only* be used together with Moveit.
2. [boxbot_moveit_config](./rcs/catkin_ws/src/boxbot_moveit_config): Moveit can not work without proper configuration. This package contains all necessary configuration files generated by the set up assistant script provided by Moveit and launch files. Note that the configuration files have been modified to fit our situation, including using a customized IK solver, lower joint limits, controller setup, etc. So any regenerated configuration package will not work whit Boxbot.  With [demo_gazebo.launch](./rcs/catkin_ws/src/boxbot_moveit_config/launch/boxbot_moveit.launch) you can launch this package with Gazebo simulation.  A customized launch file [boxbot_moveit.launch](./rcs/catkin_ws/src/boxbot_moveit_config/launch/boxbot_moveit.launch) is also provided to work with the real robot. Note that this launch should not be launched individually, it's included in a higher level launch file. 
3. [boxbot_autonomous](./rcs/catkin_ws/src/boxbot_autonomous): This package implements a boxbot_planner class that wraps the C++ interface of Moveit. It also provides a interface implemented with ROS services to the outside. You can call the service to invoke the corresponding Moveit functionality. Please refer to the user manual for detailed information. With [autonomous.launch](./rcs/catkin_ws/src/boxbot_autonomous/launch/autonomous.launch) you can launch this package together with aforementioned packages.  

### Ros_control

ros_control act as the middleware between Moveit and the hardware. It has two major components - controller manager and the hardware_interface. The controller manager launches controllers which provides a FollowJointTrajectory action interface to Moveit, and convert the trajectory commands to position/velocity/effort commands depending on which kind of controller you are using. Then it gives the command to the hardware driver. Additionally, the joint_limit will be enforced by hardware_interface. 

The above process is realized in the [boxbot_hardware](./rcs/catkin_ws/src/boxbot_hardware) package. You can launch the node with [test.launch](./rcs/catkin_ws/src/boxbot_hardware/launch/test.launch). 

### Computer vision

A single camera is powered by computer vision algorithms to search for carton stacks. [boxbot_cv](./rcs/catkin_ws/src/boxbot_cv) package contains the code that implements the image processing pipeline, reference frame transforming and camera calibration. Due to time limit, though the code is completed, but it has not been properly integrated with the whole system. Further tuning need be done before it can work normally.  

You can launch the node by launching [boxbot_cv.launch](./rcs/catkin_ws/src/boxbot_cv/launch/bxobot_cv.launch). 

### Other packages

Besides the packages that have been mentioned above, there are several other packages left:

* [boxbot_description](./rcs/catkin_ws/src/boxbot_cv): This package contains URDF description of the robot and all necessary assets including meshes and configuration files. The URDF description will loaded into the ROS parameter server each time the RCS system is started. So you should make sure the content of the URDF is in sync with the real robot and the IKFast plugin. To examine the form factor of the robot you can launch [display.launch](./rcs/catkin_ws/src/boxbot_description/launch/display.launch).
* [boxbot_control](./rcs/catkin_ws/src/boxbot_control): This package merely contains configuration file for ros_control controller manager to use in the Gazebo simulation. It should not be launched individually.
* [boxbot_odrive_control](./rcs/catkin_ws/src/boxbot_odrive_control): This package is a container for the service message definition for ODrive control. Since ODrive  is deprecated in the end, this package will not concern the execution of RCS.
* [boxbot_stepper_control](./rcs/catkin_ws/src/boxbot_stepper_control): This package contains the service message definition for stepper control. It will be loaded by the hardware package. 

## Contact
If any questions remain, feel free to contact the developers:
* Tim Anema (tim.anema@hotmail.nl): Main responsibilities are **boxbot-go** and **hardware drivers**
* Chengrui Zhao (aaron.zhaocr@gmail.com): Main responsibilities are **MoveIt**, **ros_control**, and **OpenCV**
