package hardware

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/srvs/srv_hardware"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"io"
)

type Hardware interface {
	io.Closer
	SetRelay(r Relay, enabled bool) error
	IsActive(r Relay) bool
	SetFailureCallback(cb func(PowerCheck, bool))
	SetEmergencyCallback(cb func(bool) error)
}

type hardware struct {
	logger                 log.Logger
	relayClient            *goroslib.ServiceClient
	checkSrv, emergencySrv *goroslib.ServiceProvider

	powerCb     func(PowerCheck, bool)
	emergencyCb func(bool) error

	// Internal state to keep track of the relays.
	// This assumes they start in false (disabled), which they do.
	motor, servo bool
}

func New(n *goroslib.Node, logger log.Logger) (Hardware, error) {
	var err error
	h := hardware{
		logger: logger,
	}

	// Create clients
	h.relayClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_hardware.RelaySetServiceName,
		Srv:             &srv_hardware.RelaySetService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_hardware.RelaySetServiceName)
	}

	// Create providers
	h.checkSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n,
		Name:     srv_hardware.PowerCheckServiceName,
		Srv:      &srv_hardware.PowerCheckService{},
		Callback: h.handlePowerCheckCall,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_hardware.RelaySetServiceName)
	}

	h.emergencySrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n,
		Name:     srv_hardware.EmergencyServiceName,
		Srv:      &srv_hardware.EmergencyService{},
		Callback: h.handleEmergencyCall,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_hardware.EmergencyServiceName)
	}

	return &h, nil
}

func (h *hardware) SetRelay(r Relay, enabled bool) error {
	relay, err := r.toRos()
	if err != nil {
		return errors.Wrap(err, "unable to convert to ROS types")
	}

	res := &srv_hardware.RelaySetRes{}
	err = h.relayClient.Call(&srv_hardware.RelaySetReq{
		Relay:   relay,
		Enabled: enabled,
	}, res)
	if err != nil {
		return errors.Wrap(err, "err while calling relay service")
	}

	if !res.Success {
		return errors.Errorf("failed to set relay: %v", res.Error)
	}

	// Update internal state to keep track of the relays
	if r == MotorRelay {
		h.motor = enabled
	} else if r == ServoRelay {
		h.servo = enabled
	}

	return nil
}

func (h *hardware) IsActive(r Relay) bool {
	switch r {
	case MotorRelay:
		return h.motor
	case ServoRelay:
		return h.servo
	default:
		return false
	}
}

func (h *hardware) handlePowerCheckCall(req *srv_hardware.PowerCheckReq) *srv_hardware.PowerCheckRes {
	c, err := PowerCheck(0).fromRos(req.Check)
	if err != nil {
		h.logger.Logf(msgs.WARN, "ignoring invalid power check call for %v", req.Check)
		return &srv_hardware.PowerCheckRes{}
	}

	// After ensuring the power check was valid, the callback is called
	h.powerCb(c, req.Failed)
	return &srv_hardware.PowerCheckRes{}
}

func (h *hardware) handleEmergencyCall(req *srv_hardware.EmergencyReq) *srv_hardware.EmergencyRes {
	if err := h.emergencyCb(req.Emergency); err != nil {
		return &srv_hardware.EmergencyRes{
			Success: false,
			Message: fmt.Sprintf("err while calling emergency handler: %v", err),
		}
	}

	return &srv_hardware.EmergencyRes{
		Success: true,
	}
}

func (h *hardware) SetFailureCallback(cb func(PowerCheck, bool)) {
	h.powerCb = cb
}

func (h *hardware) SetEmergencyCallback(cb func(bool) error) {
	h.emergencyCb = cb
}

func (h *hardware) Close() error {
	if err := errors.Wrapf(h.relayClient.Close(), "failed to close %s client", srv_hardware.RelaySetServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(h.checkSrv.Close(), "failed to close %s service", srv_hardware.PowerCheckServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(h.emergencySrv.Close(), "failed to close %s service", srv_hardware.EmergencyServiceName); err != nil {
		return err
	}
	return nil
}
