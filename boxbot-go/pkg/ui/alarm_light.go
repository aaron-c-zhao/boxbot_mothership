package ui

import (
	"boxbot-go/pkg/srvs/srv_hardware"
	"errors"
)

var InvalidColorErr = errors.New("invalid alarm light color")

type LightColor int

const (
	Red LightColor = iota
	Yellow
	Orange
	Green
)

func (l LightColor) toRos() (srv_hardware.BarColor, error) {
	switch l {
	case Red:
		return srv_hardware.BarRed, nil
	case Yellow:
		return srv_hardware.BarYellow, nil
	case Orange:
		return srv_hardware.BarOrange, nil
	case Green:
		return srv_hardware.BarGreen, nil
	default:
		return "", InvalidColorErr
	}
}

var InvalidEffectErr = errors.New("invalid alarm light effect")

type LightEffect int

const (
	Solid LightEffect = iota
	Flashing
	WaveUp
	WaveDown
)

func (l LightEffect) toRos() (srv_hardware.BarEffect, error) {
	switch l {
	case Solid:
		return srv_hardware.BarSolid, nil
	case Flashing:
		return srv_hardware.BarFlashing, nil
	case WaveUp:
		return srv_hardware.BarWaveUp, nil
	case WaveDown:
		return srv_hardware.BarWaveDown, nil
	default:
		return "", InvalidEffectErr
	}
}
