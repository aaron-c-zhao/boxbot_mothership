package srv_ras

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	GetLogServiceName    = "get_logs"
	DeleteLogServiceName = "del_logs"
)

//TODO(timanema): Pagination?
type GetLogReq struct {
	From, To int64
	Nodes    []string
	Levels   []string
}

type GetLogRes struct {
	Result []msgs.LogEntry `json:"result"`
	Error  string          `json:"-"`
}

type GetLogService struct {
	msg.Package `ros:"ras"`
	GetLogReq
	GetLogRes
}

type DeleteLogReq struct {
	From, To int64
	Nodes    []string
	Levels   []string
}

type DeleteLogRes struct {
	Deleted uint32
	Error   string
}

type DeleteLogService struct {
	msg.Package `ros:"ras"`
	DeleteLogReq
	DeleteLogRes
}
