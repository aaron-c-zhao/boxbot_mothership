package srv_rcs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	GetMotorStatusServiceName = "get_status"
	HomeServiceName           = "home"
	CancelHomeServiceName     = "cancel_home"
	ReadyServiceName          = "motor_ready"
	MotorMoveServiceName      = "move_motor"
	MotorMoveStopServiceName  = "move_stop_motor"
	BlockMotorServiceName     = "block_motor"
)

type ErrorCode = int16

const (
	NoError          ErrorCode = 0x00
	BlockedLimit     ErrorCode = 0x01
	MotorDisabled    ErrorCode = 0x02
	ConnectionFault  ErrorCode = 0x03
	Homing           ErrorCode = 0x04
	MotorCalibrating ErrorCode = 0x05
)

type GetMotorStatusReq struct {
	Empty std_msgs.Empty
}

type GetMotorStatusRes struct {
	Motors []Status
}

type Status struct {
	Calibrating, BlockedLeft, BlockedRight, Idle, Moving, Disabled, Fault, Homing, Homed bool
	Speed                                                                                float32
	Location                                                                             int32
}

type GetMotorStatusService struct {
	msg.Package `ros:"boxbot_stepper_control"`
	GetMotorStatusReq
	GetMotorStatusRes
}

type MotorReadyReq struct {
	Empty std_msgs.Empty
}

type MotorReadyRes struct {
	Ready, Failed bool
	Message       string
}

type MotorReadyService struct {
	msg.Package `ros:"boxbot_stepper_control"`
	MotorReadyReq
	MotorReadyRes
}

type MotorMoveReq struct {
	MotorID int32 `rosname:"motor_id"`
	Left    bool
}

type MotorMoveRes struct {
	Success bool
	Message string
}

type MotorMoveService struct {
	msg.Package `ros:"boxbot_stepper_control"`
	MotorMoveReq
	MotorMoveRes
}

type MotorMoveStopReq struct {
	MotorID int32 `rosname:"motor_id"`
}

type MotorMoveStopRes struct {
	Distance    int32
	Start, Stop int32
	Success     bool
	Message     string
}

type MotorMoveStopService struct {
	msg.Package `ros:"boxbot_stepper_control"`
	MotorMoveStopReq
	MotorMoveStopRes
}

type BlockMotorReq struct {
	MotorID int32 `rosname:"motor_id"`
	Blocked bool
}

type BlockMotorRes struct {
	Success bool
	Message string
}

type BlockMotorService struct {
	msg.Package `ros:"boxbot_stepper_control"`
	BlockMotorReq
	BlockMotorRes
}
