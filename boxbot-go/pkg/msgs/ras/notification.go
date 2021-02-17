package ras

import (
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/node"
	"github.com/aler9/goroslib/pkg/msg"
)

var NotificationTopic = msgs.Topic{
	Topic: node.GlobalNamespace + "/notification",
	Msg:   &Notification{},
}

type Notification struct {
	msg.Package `ros:"ras"`
	Id          uint32
	Message     string
}
