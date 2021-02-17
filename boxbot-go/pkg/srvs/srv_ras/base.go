package srv_ras

import (
	"boxbot-go/internal/ras_states"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	GetRasStateServiceName        = "get_ras_state"
	TransitionRasStateServiceName = "transition_ras_state"
)

type GetRasStateReq struct {
	Empty std_msgs.Empty
}

type GetRasStateRes struct {
	State ras_states.RasState
}

type GetRasStateService struct {
	msg.Package `ros:"ras"`
	GetRasStateReq
	GetRasStateRes
}

type TransitionRasReq struct {
	State ras_states.RasState
}

type TransitionRasRes struct {
	Success bool
	Error   string
}

type TransitionRasStateService struct {
	msg.Package `ros:"ras"`
	TransitionRasReq
	TransitionRasRes
}
