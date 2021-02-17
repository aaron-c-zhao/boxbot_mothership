package rcs_link

import (
	"boxbot-go/internal/rcs_states"
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	rcsmsgs "boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/srvs/srv_rcs"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"time"
)

var RCSInitTimeoutErr = errors.New("timeout has been reached while waiting for RSC to initialize")

type Callback func() error

type RCS interface {
	io.Closer

	SetStateCallbacks(stuckCb, emptyCb, errCb Callback)
	WaitForInit(timeout time.Duration) error
	Start() error
	Shutdown() error
	SetPaused(paused bool) error
	SetHalted(halted bool) error
	SetEmergency(emergency bool) error
	StackPlaced() error
	OperatorUnstuck() error
}

type rcs struct {
	startSrv                                                                           *goroslib.ServiceClient
	shutdownSrv, pausedSrv, haltedSrv, emergencySrv, placeStackSrv, operatorUnstuckSrv *goroslib.ServiceClient
	logger                                                                             log.Logger

	initCh                           chan struct{}
	stuckLatch, emptyLatch, errLatch *atomic.Bool
	stuckCb, emptyCb, errCb          Callback
	rcsSub                           *goroslib.Subscriber
}

func NewRCS(n *goroslib.Node, logger log.Logger) (RCS, error) {
	var err error
	r := &rcs{
		logger:     logger,
		stuckLatch: atomic.NewBool(false),
		emptyLatch: atomic.NewBool(false),
		errLatch:   atomic.NewBool(false),
		initCh:     make(chan struct{}),
	}

	// Create subscribers
	r.rcsSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:            n,
		Topic:           rcsmsgs.StateChangeTopic.Topic,
		Callback:        r.handleRCSStateChange,
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s subscriber", rcsmsgs.StateChangeTopic.Topic)
	}

	// Create service clients
	r.startSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsStartServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsStartServiceName)
	}

	r.shutdownSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsShutdownServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsShutdownServiceName)
	}

	r.pausedSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsSetPausedServiceName,
		Srv:             &std_srvs.SetBool{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsSetPausedServiceName)
	}

	r.haltedSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsSetHaltedServiceName,
		Srv:             &std_srvs.SetBool{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsSetHaltedServiceName)
	}

	r.emergencySrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsSetEmergencyServiceName,
		Srv:             &std_srvs.SetBool{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsSetEmergencyServiceName)
	}

	r.placeStackSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsStackPlacedServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsStackPlacedServiceName)
	}

	r.operatorUnstuckSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.RcsOperatorUnstuckServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.RcsOperatorUnstuckServiceName)
	}

	return r, nil
}

func (r *rcs) handleRCSStateChange(msg *rcsmsgs.StateChange) {
	r.logger.Logf(msgs.DEBUG, "calling rcs callback to handle rcs state change: %s", msg.State)

	switch msg.State {
	case rcs_states.StuckState:
		if r.stuckCb == nil {
			r.stuckLatch.Store(true)
		} else {
			if err := r.stuckCb(); err != nil {
				r.logger.Logf(msgs.WARN, "error while handling rcs stuck callback: %v", err)
			}
		}
	case rcs_states.EmptyState:
		if r.emptyCb == nil {
			r.emptyLatch.Store(true)
		} else {
			if err := r.emptyCb(); err != nil {
				r.logger.Logf(msgs.WARN, "error while handling rcs empty callback: %v", err)
			}
		}
	case rcs_states.ErrorState:
		if r.errCb == nil {
			r.errLatch.Store(true)
		} else {
			if err := r.errCb(); err != nil {
				r.logger.Logf(msgs.WARN, "error while handling rcs err callback: %v", err)
			}
		}
	case rcs_states.RcsReady:
		// When this state is reached the robot is ready, so close the init channel
		r.logger.Logf(msgs.INFO, "received signal that RCS is ready")
		close(r.initCh)
	}
}

func (r *rcs) SetStateCallbacks(stuckCb, emptyCb, errCb Callback) {
	r.stuckCb = stuckCb
	r.emptyCb = emptyCb
	r.errCb = errCb

	// Check if latches are set
	if r.stuckLatch.Load() {
		if err := r.stuckCb(); err != nil {
			r.logger.Logf(msgs.WARN, "error while handling rcs stuck callback: %v", err)
		}
	}

	if r.emptyLatch.Load() {
		if err := r.emptyCb(); err != nil {
			r.logger.Logf(msgs.WARN, "error while handling rcs empty callback: %v", err)
		}
	}

	if r.errLatch.Load() {
		if err := r.errCb(); err != nil {
			r.logger.Logf(msgs.WARN, "error while handling rcs err callback: %v", err)
		}
	}
}

func (r *rcs) WaitForInit(timeout time.Duration) error {
	select {
	case <-r.initCh:
	case <-time.After(timeout):
		return RCSInitTimeoutErr
	}

	return nil
}

func (r *rcs) Start() error {
	res := std_srvs.TriggerRes{}
	if err := r.startSrv.Call(&std_srvs.TriggerReq{}, &res); err != nil {
		return errors.Wrap(err, "failed to make service start call")
	}

	if !res.Success {
		return errors.Errorf("start failed: %s", res.Message)
	}

	return nil
}

func (r *rcs) Shutdown() error {
	res := std_srvs.TriggerRes{}
	if err := r.shutdownSrv.Call(&std_srvs.TriggerReq{}, &res); err != nil {
		return errors.Wrap(err, "failed to make service shutdown call")
	}

	if !res.Success {
		return errors.Errorf("shutdown failed: %s", res.Message)
	}

	return nil
}

func (r *rcs) SetPaused(paused bool) error {
	res := std_srvs.SetBoolRes{}
	if err := r.pausedSrv.Call(&std_srvs.SetBoolReq{Data: paused}, &res); err != nil {
		return errors.Wrap(err, "failed to make service set paused call")
	}

	if !res.Success {
		return errors.Errorf("set paused failed: %s", res.Message)
	}
	return nil
}

func (r *rcs) SetHalted(halted bool) error {
	res := std_srvs.SetBoolRes{}
	if err := r.haltedSrv.Call(&std_srvs.SetBoolReq{Data: halted}, &res); err != nil {
		return errors.Wrap(err, "failed to make service set halted call")
	}

	if !res.Success {
		return errors.Errorf("set halted failed: %s", res.Message)
	}
	return nil
}

func (r *rcs) SetEmergency(emergency bool) error {
	res := std_srvs.SetBoolRes{}
	if err := r.emergencySrv.Call(&std_srvs.SetBoolReq{Data: emergency}, &res); err != nil {
		return errors.Wrap(err, "failed to make service set emergency call")
	}

	if !res.Success {
		return errors.Errorf("set emergency failed: %s", res.Message)
	}
	return nil
}

func (r *rcs) StackPlaced() error {
	res := std_srvs.TriggerRes{}
	if err := r.placeStackSrv.Call(&std_srvs.TriggerReq{}, &res); err != nil {
		return errors.Wrap(err, "failed to make service stack placed call")
	}

	if !res.Success {
		return errors.Errorf("stack placed failed: %s", res.Message)
	}

	return nil
}

func (r *rcs) OperatorUnstuck() error {
	res := std_srvs.TriggerRes{}
	if err := r.operatorUnstuckSrv.Call(&std_srvs.TriggerReq{}, &res); err != nil {
		return errors.Wrap(err, "failed to make service operator unstuck call")
	}

	if !res.Success {
		return errors.Errorf("operator unstuck placed failed: %s", res.Message)
	}

	return nil
}

func (r *rcs) Close() (err error) {
	if err = errors.Wrapf(r.rcsSub.Close(), "failed to close %s subscriber", rcsmsgs.StateChangeTopic.Topic); err != nil {
		return
	}
	if err := errors.Wrapf(r.shutdownSrv.Close(), "failed to close %s service client", srv_rcs.RcsShutdownServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(r.pausedSrv.Close(), "failed to close %s service client", srv_rcs.RcsSetPausedServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(r.haltedSrv.Close(), "failed to close %s service client", srv_rcs.RcsSetHaltedServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(r.emergencySrv.Close(), "failed to close %s service client", srv_rcs.RcsSetEmergencyServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(r.placeStackSrv.Close(), "failed to close %s service client", srv_rcs.RcsStackPlacedServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(r.operatorUnstuckSrv.Close(), "failed to close %s service client", srv_rcs.RcsOperatorUnstuckServiceName); err != nil {
		return err
	}

	return
}
