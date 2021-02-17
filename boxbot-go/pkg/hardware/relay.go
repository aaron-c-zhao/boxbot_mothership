package hardware

import (
	"boxbot-go/pkg/srvs/srv_hardware"
	"github.com/pkg/errors"
)

type Relay int8

const (
	MotorRelay Relay = iota
	ServoRelay
)

var InvalidRelayErr = errors.New("invalid relay")

func (r Relay) toRos() (srv_hardware.Relay, error) {
	switch r {
	case MotorRelay:
		return srv_hardware.RelayMotor, nil
	case ServoRelay:
		return srv_hardware.RelayServo, nil
	default:
		return "", InvalidRelayErr
	}
}
