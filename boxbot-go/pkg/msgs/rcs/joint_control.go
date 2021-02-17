package rcs

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msg"
)

var JointControlTopic = msgs.Topic{
	Topic: "/joint_control",
	Msg:   &JointControl{},
}

/**
ROS equivalent JoinControl.msg:
Joint[] ctr
*/
type JointControl struct {
	msg.Package `ros:"boxbot_stepper_control"`
	Ctr         []Joint
}

/**
ROS equivalent Joint.msg:
int32 motor_id
float32 position
bool disable
*/
type Joint struct {
	msg.Package `ros:"boxbot_stepper_control"`
	MotorID     int32 `rosname:"motor_id"`
	Position    float32
	Disable     bool
}
