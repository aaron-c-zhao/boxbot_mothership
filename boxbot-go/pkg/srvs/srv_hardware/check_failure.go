package srv_hardware

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const PowerCheckServiceName = "check_pwr_failure"

type PowerCheck = string

const (
	CheckInput  = "CHECK_INPUT"
	CheckMotor1 = "CHECK_MOTOR1"
	CheckMotor2 = "CHECK_MOTOR2"
	CheckMotor3 = "CHECK_MOTOR3"
	CheckServo  = "CHECK_SERVO"
)

type PowerCheckReq struct {
	Check  PowerCheck
	Failed bool
}

type PowerCheckRes struct {
	Empty std_msgs.Empty
}

type PowerCheckService struct {
	msg.Package `ros:"ras-hardware"`
	PowerCheckReq
	PowerCheckRes
}
