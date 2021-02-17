package ras

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msg"
)

type ButtonType = string

const (
	PowerButton = "BTN_PWR"
	StartButton = "BTN_STR"
	PauseButton = "BTN_PSE"
)

var ButtonEventTopic = msgs.Topic{
	Topic: "io_button_event",
	Msg:   &ButtonEvent{},
}

type ButtonEvent struct {
	msg.Package `ros:"ras"`
	Type        ButtonType
}
