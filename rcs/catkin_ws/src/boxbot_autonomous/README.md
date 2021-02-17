# Services that FSM may call

### 1. boxbot_autonomous/rcs_control

#### Attributes

**Request**

* cmd:
    * 0 (or ControlCmd::MOVE): move the eef to certain position. goal required(value of w will be ignored).
    * 1 (STOP): Stop any ongoing trajectory execution
    * 2 (HOME): Set the joint state to 0
    * 3 (GET_LOCATION): Retrieve the current location of the eef(including yaw)
    * 4 (ROTATE): Rotate the gripper. goal.w is required
    * 5 (SET_POS): Set the joint states to designated position including x, y, z, and w. goal required.
* goal:
    * will be ignored most of the cases except MOVE, ROTATE and SET_POS

**Response**

* location:
    * contains the current location of eef when GET_LOCATION is called. w is the yaw.
* error_code: indicates where goes wrong if the service fails.


TODO: return the base angle
TODO: define error_code in msg

### 2. boxbot_odrive_control/boxbot_descend
see [boxbot_descend](../boxbot_odrive_control/README.md).

TODO: change this service to cater to step motor











