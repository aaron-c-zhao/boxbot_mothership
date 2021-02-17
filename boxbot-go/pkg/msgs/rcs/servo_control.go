package rcs

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msg"
)

var ServoTopic = msgs.Topic{
	Topic: "/servo_control",
	Msg:   &ServoControl{},
}

/**
ROS equivalent Joint.msg:
float64 angle
float64 speed
*/
type ServoControl struct {
	msg.Package `ros:"boxbot_servo_control"`
	Angle       float64
	Speed       float64
}
