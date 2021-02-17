package ui

import (
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/srvs/srv_hardware"
	"github.com/pkg/errors"
)

var InvalidButtonErr = errors.New("invalid button type")

type Button int

const (
	PowerButton Button = iota
	StartButton
	PauseButton
)

func (b Button) toRos() (ras.ButtonType, error) {
	switch b {
	case PowerButton:
		return ras.PowerButton, nil
	case StartButton:
		return ras.StartButton, nil
	case PauseButton:
		return ras.PauseButton, nil
	default:
		return "", InvalidButtonErr
	}
}

func (b Button) fromRos(btn ras.ButtonType) (Button, error) {
	switch btn {
	case ras.PowerButton:
		return PowerButton, nil
	case ras.StartButton:
		return StartButton, nil
	case ras.PauseButton:
		return PauseButton, nil
	default:
		return b, InvalidButtonErr
	}
}

var InvalidButtonEffectErr = errors.New("invalid button effect")

type ButtonEffect int

const (
	HintBlink ButtonEffect = iota
	ErrBlink
	On
	Off
)

func (b ButtonEffect) toRos() (srv_hardware.ButtonEffectType, error) {
	switch b {
	case HintBlink:
		return srv_hardware.EffectHint, nil
	case ErrBlink:
		return srv_hardware.EffectErr, nil
	case On:
		return srv_hardware.EffectOn, nil
	case Off:
		return srv_hardware.EffectOff, nil
	default:
		return "", InvalidButtonEffectErr
	}
}
