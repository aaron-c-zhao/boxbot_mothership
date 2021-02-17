package main

import (
	gripperserial "boxbot-go/pkg/gripper_serial_v2"
	"fmt"
	"log"
	"time"
)

func main() {
	log.Println("starting stepper node")

	var s *gripperserial.SerialGripper

	s = gripperserial.New("/dev/ttyUSB0", func() {
		fmt.Println("grip completed")
	}, func() {
		fmt.Println("grip failed")
	}, func(sensor gripperserial.Sensor) {
		fmt.Printf("sensor hit %d\n", sensor)
	}, func() {
		fmt.Printf("start higher\nmoving higher for fixed amount\n")
		time.Sleep(time.Second * 5)
		fmt.Printf("moving higher err: %v\n", s.CommandCompleted())
		fmt.Printf("moving right\n")
	}, func() {
		fmt.Printf("right edge found\nmoving back to starting position\n")
		time.Sleep(time.Second * 5)
		fmt.Printf("moved to starting position\nmoving down for sensor hit\n")
		fmt.Printf("moving down err: %v\n", s.CommandCompleted())
	}, func() {
		fmt.Printf("high low edge found\nmoving to calculated position\n")
		time.Sleep(time.Second * 5)
		fmt.Printf("moved to calculated position\nmoving lower to grip\n")
		time.Sleep(time.Second * 5)
		fmt.Printf("moving lower err: %v\n", s.CommandCompleted())
	})

	fmt.Printf("start err:%v\n", s.Start())
	fmt.Printf("search err: %v\nmoving lower to carton indefinately\n", s.StartSearch())
	for {
	}

	log.Println("stopping stepper node")
}

/**
Gripper setup:
Left side        Right side (grip side)
|               | <- Upper limit switch
|               | <- Upper IR sensor
|               |
|---------------| <- Central limit switch
|               |
|               | <- Lower IR sensor
|        XX     | <- Lower limit switch
XX = Gripper limit switch (triggered by small notch on right side, when the right side is properly closed)

Picking order (sensors on the right, and stacks cannot move over other stacks of the same height):
6 4 2
5 3 1


Gripper algo V1 (might be infeasible due to restrictions with servo):
NOTE: Left, right, up, down, lower, and higher are relative to the gripper itself.
NOTE: 0 = not done in gripper itself, but done in advance

0.a: Find approximate location of center and rotation of carton with OpenCV
0.b: Move center of gripper to the left (with constant C1) of the carton center, taking the rotation into account
1: Move lower until central limit switch trigger (carton has been hit)
2: Move higher for a constant value C2, so the gripper hovers C2 above the carton stack
3: Move to the right until one of the IR sensors loses sight (so is at the carton edge)
4: If this first sensor was the upper IR sensor:
      Rotate the gripper CCW, and move to the right every time the upper IR sensor sees carton again to compensate
      Continue until both the upper and lower IR sensors see nothing
   Else:
      Rotate the gripper CW, and move to the right every time the lower IR sensor sees carton again to compensate
      Continue until both the upper and lower IR sensors see nothing
5: Move to the left for a constant value C3, so the right gripper side is somewhere between the right side and
   center of the carton
6: Move down until the lower IR sensor loses the carton (it is now at the lower edge of the carton)
7: Move up for a constant value C4, so the gripper is at the center (in height)
8: Move right for a constant value C3, so the gripper is at the center (in width)
9: Move low for a constant value C5, so the gripper is now over the stack. If either the upper, central, or lower
   limit switches trigger, stop and realign for a maximum of C6 times, and call for help if it still fails.
10: Close the gripper, and wait for a predetermined amount of time C7. If the gripper limit switch is trigger
    the process is completed. If not, retry step 10 for a maximum of C8 times, and call for help if it still fails.

Gripper algo V2:
NOTE: 0 = not done in gripper itself, but done in advance

0.a: Find approximate location of center and rotation of carton with OpenCV
0.b: Move center of gripper to the left (with constant C1) of the carton center, taking the rotation into account
1: Move lower until central limit switch trigger (carton has been hit)
2: Move higher for a constant value C2, so the gripper hovers C2 above the carton stack
3: Move to the right until both IR sensors lose sight (so both are outside the carton), signaling each event.
   The controlling software records the coordinates of these events, which can then be used to calculate the angle
   between these two points ( = the carton angle)
4: Move to the left for a constant value of C3, so the IR sensors are approximately in the middle line (taking
   the previously calculated angle into account). The controller system will only signal readiness, once the correct
   carton angle has been calculated and set.
5: Move down until lower IR loses sight and emit event. The controlling system will record this coordinate, and use it
   to calculate the carton center and rotation. The gripper will wait for the analysis to complete.
   After the controlling system has moved the gripper to the correct center and rotation, it will signal readiness.
6: Move low for a constant value C4, so the gripper is now over the stack. If either the upper, central, or lower
   limit switches trigger, stop and realign for a maximum of C5 times, and call for help if it still fails.
7: Close the gripper, and wait for a predetermined amount of time C6. If the gripper limit switch is trigger
    the process is completed. If not, retry step 7 for a maximum of C7 times, and call for help if it still fails.
*/
