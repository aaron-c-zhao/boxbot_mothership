package msgs

import (
	"boxbot-go/pkg/node"
	"github.com/aler9/goroslib/pkg/msg"
)

type LogLevel = string

const (
	TRACE LogLevel = "TRACE"
	DEBUG LogLevel = "DEBUG"
	INFO  LogLevel = "INFO"
	WARN  LogLevel = "WARN"
	ERROR LogLevel = "ERROR"
)

var LogTopic = Topic{
	Topic: node.GlobalNamespace + "/log_entry",
	Msg:   &LogEntry{},
}

type LogEntry struct {
	msg.Package `ros:"ras" json:"-"`
	Timestamp   int64    `json:"timestamp"`
	Node        string   `json:"node"`
	Level       LogLevel `json:"level"`
	Message     string   `json:"message"`
}
