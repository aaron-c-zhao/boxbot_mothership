package srv_hardware

import (
	"boxbot-go/pkg/msgs/ras"
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	ButtonEffectServiceName = "io_button_effect"
	InterfaceSetServiceName = "io_interface_set"
	BarSetServiceName       = "io_bar_set"
)

type BarColor = string

const (
	BarRed    = "RED"
	BarYellow = "YELLOW"
	BarOrange = "ORANGE"
	BarGreen  = "GREEN"
)

type BarEffect = string

const (
	BarSolid    = "SOLID"
	BarFlashing = "FLASH"
	BarWaveUp   = "WAVE_UP"
	BarWaveDown = "WAVE_DOWN"
)

type ButtonEffectType = string

const (
	EffectHint = "INPUT_HINT" // blinks the button slowly indefinitely
	EffectErr  = "INPUT_ERR"  // blinks the button for a short amount of time
	EffectOff  = "INPUT_OFF"  // disables button light (to solid off)
	EffectOn   = "INPUT_ON"   // enabled button light (to solid on)
)

type ButtonEffectReq struct {
	Button       ras.ButtonType
	ButtonEffect ButtonEffectType
}

type ButtonEffectRes struct {
	Success bool
	Error   string
}

type ButtonEffectService struct {
	msg.Package `ros:"ras-hardware"`
	ButtonEffectReq
	ButtonEffectRes
}

type BarSetReq struct {
	Color  BarColor
	Effect BarEffect
}

type BarSetRes struct {
	Success bool
	Error   string
}

type BarSetService struct {
	msg.Package `ros:"ras-hardware"`
	BarSetReq
	BarSetRes
}

type InterfaceSetReq struct {
	Error   bool
	Message string
	Hint    string
}

type InterfaceSetRes struct {
	Success bool
	Error   string
}

type InterfaceSetService struct {
	msg.Package `ros:"ras-hardware"`
	InterfaceSetReq
	InterfaceSetRes
}
