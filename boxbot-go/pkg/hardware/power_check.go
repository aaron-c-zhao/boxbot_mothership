package hardware

import (
	"boxbot-go/pkg/srvs/srv_hardware"
	"github.com/pkg/errors"
)

type PowerCheck int8

const (
	InputCheck PowerCheck = iota
	Motor1Check
	Motor2Check
	Motor3Check
	ServoCheck
)

var (
	matches = map[PowerCheck]Relay{
		Motor1Check: MotorRelay,
		Motor2Check: MotorRelay,
		Motor3Check: MotorRelay,
		ServoCheck:  ServoRelay,
	}
)

var InvalidPowerCheckErr = errors.New("invalid power check")

func (p PowerCheck) toRos() (srv_hardware.PowerCheck, error) {
	switch p {
	case InputCheck:
		return srv_hardware.CheckInput, nil
	case Motor1Check:
		return srv_hardware.CheckMotor1, nil
	case Motor2Check:
		return srv_hardware.CheckMotor2, nil
	case Motor3Check:
		return srv_hardware.CheckMotor3, nil
	case ServoCheck:
		return srv_hardware.CheckServo, nil
	default:
		return "", InvalidPowerCheckErr
	}
}

func (p PowerCheck) fromRos(i srv_hardware.PowerCheck) (PowerCheck, error) {
	switch i {
	case srv_hardware.CheckInput:
		return InputCheck, nil
	case srv_hardware.CheckMotor1:
		return Motor1Check, nil
	case srv_hardware.CheckMotor2:
		return Motor2Check, nil
	case srv_hardware.CheckMotor3:
		return Motor3Check, nil
	case srv_hardware.CheckServo:
		return ServoCheck, nil
	default:
		return p, InvalidPowerCheckErr
	}
}

func (p PowerCheck) MatchingRelay() Relay {
	r, ok := matches[p]
	if !ok {
		return Relay(-1)
	}

	return r
}
