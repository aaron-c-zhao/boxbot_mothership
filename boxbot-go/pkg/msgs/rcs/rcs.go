package rcs

import (
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/node"
	"github.com/aler9/goroslib/pkg/msg"
)

type State = string

var StateChangeTopic = msgs.Topic{
	Topic: node.GlobalNamespace + "/rcs/rcs_state_change",
	Msg:   &StateChange{},
}

type StateChange struct {
	msg.Package `ros:"ras"`
	State       State
}
