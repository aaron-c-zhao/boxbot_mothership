package srv_hardware

import "github.com/aler9/goroslib/pkg/msg"

const RelaySetServiceName = "relay_set"

type Relay = string

const (
	RelayMotor = "REL_MOTOR"
	RelayServo = "REL_SERVO"
)

type RelaySetReq struct {
	Relay   Relay
	Enabled bool
}

type RelaySetRes struct {
	Success bool
	Error   string
}

type RelaySetService struct {
	msg.Package `ros:"ras"`
	RelaySetReq
	RelaySetRes
}
