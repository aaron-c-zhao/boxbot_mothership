package srv_hardware

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const EmergencyServiceName = "emergency_flag"

type EmergencyReq struct {
	Emergency bool
}

type EmergencyRes struct {
	Success bool
	Message string
}

type EmergencyService struct {
	msg.Package `ros:"ras-hardware"`
	EmergencyReq
	EmergencyRes
}
