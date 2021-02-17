package boxbot_autonomous //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

const (
	RcsControlBaseServiceName     = "/rcs_control"
	RcsControlMoveServiceName     = RcsControlBaseServiceName + "/move"
	RcsControlLocationServiceName = RcsControlBaseServiceName + "/get_location"
	RcsControlHomeServiceName     = RcsControlBaseServiceName + "/home"
	RcsControlRotateServiceName   = RcsControlBaseServiceName + "/rotate"
	RcsControlSetPosServiceName   = RcsControlBaseServiceName + "/set_pos"
	RcsControlStopServiceName     = RcsControlBaseServiceName + "/stop"
)

const (
	ControlCmdReq_MOVE         int8 = 0 //nolint:golint
	ControlCmdReq_STOP         int8 = 1 //nolint:golint
	ControlCmdReq_HOME         int8 = 2 //nolint:golint
	ControlCmdReq_GET_LOCATION int8 = 3 //nolint:golint
	ControlCmdReq_ROTATE       int8 = 4 //nolint:golint
	ControlCmdReq_SET_POS      int8 = 5 //nolint:golint
)

type ControlCmdReq struct { //nolint:golint
	msg.Definitions `ros:"int8 MOVE=0,int8 STOP=1,int8 HOME=2,int8 GET_LOCATION=3,int8 ROTATE=4,int8 SET_POS=5"`
	Cmd             int8                     //nolint:golint
	Goal            geometry_msgs.Quaternion //nolint:golint
}

type ControlCmdRes struct { //nolint:golint
	Location  geometry_msgs.Quaternion //nolint:golint
	ErrorCode int8                     //nolint:golint
}

type ControlCmd struct { //nolint:golint
	msg.Package `ros:"boxbot_autonomous"`
	ControlCmdReq
	ControlCmdRes
}
