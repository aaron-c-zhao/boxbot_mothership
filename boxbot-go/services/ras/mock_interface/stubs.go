package main

import (
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/srvs/srv_hardware"
	"boxbot-go/pkg/ws"
	"github.com/pkg/errors"
	"strings"
)

func (n *interfaceNode) setButtonEffect(req *srv_hardware.ButtonEffectReq) *srv_hardware.ButtonEffectRes {
	switch req.Button {
	case ras.StartButton:
		n.StartBtn = req.ButtonEffect
	case ras.PauseButton:
		n.PauseBtn = req.ButtonEffect
	case ras.PowerButton:
		n.PowerBtn = req.ButtonEffect
	}
	return &srv_hardware.ButtonEffectRes{
		Success: true,
	}
}

func (n *interfaceNode) setInterface(req *srv_hardware.InterfaceSetReq) *srv_hardware.InterfaceSetRes {
	n.InterfaceText = strings.Replace(req.Message, "\n", "\\n", -1)

	n.websock.Publish(&ws.DisplayState{
		Error:   req.Error,
		Message: req.Message,
		Hint:    req.Hint,
	})

	return &srv_hardware.InterfaceSetRes{
		Success: true,
	}
}

func (n *interfaceNode) setBar(req *srv_hardware.BarSetReq) *srv_hardware.BarSetRes {
	n.BarColor = req.Color
	n.BarEffect = req.Effect
	return &srv_hardware.BarSetRes{
		Success: true,
	}
}

func (n *interfaceNode) setRelay(req *srv_hardware.RelaySetReq) *srv_hardware.RelaySetRes {
	switch req.Relay {
	case srv_hardware.RelayMotor:
		n.MotorRelayActive = req.Enabled
	case srv_hardware.RelayServo:
		n.ServoRelayActive = req.Enabled
	}
	return &srv_hardware.RelaySetRes{
		Success: true,
	}
}

func (n *interfaceNode) setCheck(check srv_hardware.PowerCheck, failed bool) error {
	req := &srv_hardware.PowerCheckReq{
		Check:  check,
		Failed: failed,
	}

	err := n.checkClient.Call(req, &srv_hardware.PowerCheckRes{})
	return errors.Wrap(err, "unable to call power check service")
}

func (n *interfaceNode) toggleEmergency() error {
	n.Emergency = !n.Emergency
	req := &srv_hardware.EmergencyReq{
		Emergency: n.Emergency,
	}

	res := &srv_hardware.EmergencyRes{}
	err := n.emergencyClient.Call(req, res)

	if !res.Success {
		n.Emergency = !n.Emergency
	}

	return errors.Wrap(err, "unable to call emergency service")
}

func (n *interfaceNode) pressButton(t ras.ButtonType) {
	n.btnPub.Write(&ras.ButtonEvent{
		Type: t,
	})
}
