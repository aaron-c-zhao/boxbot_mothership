package ras

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msg"
)

type State = string

var StateChangeTopic = msgs.Topic{
	Topic: "ras_state_change",
	Msg:   &StateChange{},
}

type StateChange struct {
	msg.Package `ros:"ras"`
	State       State
}
