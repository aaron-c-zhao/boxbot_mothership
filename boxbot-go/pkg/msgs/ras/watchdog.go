package ras

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msg"
)

var WatchdogTopic = msgs.Topic{
	Topic: "watchdog_alert",
	Msg:   &WatchdogAlert{},
}

type WatchdogAlert struct {
	msg.Package `ros:"ras"`
	Failed      bool
}
