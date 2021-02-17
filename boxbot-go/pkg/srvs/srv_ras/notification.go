package srv_ras

import "github.com/aler9/goroslib/pkg/msg"

const CreateNotificationServiceName = "create_notification"

type CreateNotificationReq struct {
	Message string
}

type CreateNotificationRes struct {
	Id      uint32
	Success bool
	Error   string
}

type CreateNotificationService struct {
	msg.Package `ros:"ras"`
	CreateNotificationReq
	CreateNotificationRes
}
