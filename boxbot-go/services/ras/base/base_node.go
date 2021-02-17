package main

import (
	"boxbot-go/pkg/event"
	"boxbot-go/pkg/hardware"
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs/ras"
	rcs2 "boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/rcs_link"
	"boxbot-go/pkg/srvs/srv_ras"
	"boxbot-go/pkg/ui"
	"boxbot-go/pkg/watchdog"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/looplab/fsm"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"sync"
)

type baseNode struct {
	n                        *goroslib.Node
	getStateSrv, setStateSrv *goroslib.ServiceProvider
	rasPub                   *goroslib.Publisher

	close, done chan struct{}
	closeLock   sync.Once

	fsm *fsm.FSM
	c   Config

	logger log.Logger
	ui     ui.UI
	rcs    rcs_link.RCS
	hw     hardware.Hardware
	wd     watchdog.Watchdog

	emergencyEvent, watchdogEvent *event.Event
	emptyEvent, stuckEvent        *event.Event
	startupCompleted, stopStarted *atomic.Bool
	paused                        *atomic.Bool
}

func NewNode(c node.Config) (node.Node, error) {
	var err error
	n := &baseNode{
		close:            make(chan struct{}, 1),
		done:             make(chan struct{}),
		startupCompleted: atomic.NewBool(false),
		stopStarted:      atomic.NewBool(false),
		paused:           atomic.NewBool(false),
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "failed to read config for base node")
	}

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.RasBaseName,
		Namespace:     node.RasNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.RasBaseName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create services
	n.getStateSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.GetRasStateServiceName,
		Srv:      &srv_ras.GetRasStateService{},
		Callback: n.getState,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_ras.GetRasStateServiceName)
	}
	n.setStateSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.TransitionRasStateServiceName,
		Srv:      &srv_ras.TransitionRasStateService{},
		Callback: n.transitionState,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_ras.TransitionRasStateServiceName)
	}

	// Create publishers
	n.rasPub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n.n,
		Topic: ras.StateChangeTopic.Topic,
		Msg:   ras.StateChangeTopic.Msg,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s publisher", ras.StateChangeTopic.Topic)
	}

	// Create UI link
	if n.ui, err = ui.New(n.n, n.logger); err != nil {
		return nil, errors.Wrap(err, "failed to create link to UI")
	}
	n.ui.SetButtonCallback(n.handleButtonEvent)

	// Create hardware link
	if n.hw, err = hardware.New(n.n, n.logger); err != nil {
		return nil, errors.Wrap(err, "failed to create link to hardware")
	}
	n.hw.SetFailureCallback(n.handlePowerFailure)
	n.hw.SetEmergencyCallback(n.handleEmergencyFlag)

	// Create watchdog
	if n.wd, err = watchdog.New(n.n, n.logger); err != nil {
		return nil, errors.Wrap(err, "failed to create link with watchdog")
	}
	n.wd.SetCallback(n.handleWatchdog)

	// Create FSM
	n.createFSM()

	go node.PingSelf(n.n, n.close, n.done, node.RasBaseName)
	return n, nil
}

func (n *baseNode) getState(_ *srv_ras.GetRasStateReq) *srv_ras.GetRasStateRes {
	return &srv_ras.GetRasStateRes{State: n.fsm.Current()}
}

// TODO(timanema): Restrictions?
func (n *baseNode) transitionState(req *srv_ras.TransitionRasReq) *srv_ras.TransitionRasRes {
	if err := n.fsm.Event(req.State); err != nil {
		return &srv_ras.TransitionRasRes{
			Success: false,
			Error:   err.Error(),
		}
	}

	return &srv_ras.TransitionRasRes{
		Success: true,
	}
}

func (n *baseNode) handleRCSStateChange(_ *rcs2.StateChange) {

}

func (n *baseNode) Done() <-chan struct{} {
	return n.done
}

func (n *baseNode) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrap(n.ui.Close(), "failed to close ui client"); err != nil {
			return
		}
		if err = errors.Wrap(n.hw.Close(), "failed to close hardware client"); err != nil {
			return
		}
		if err = errors.Wrap(n.wd.Close(), "failed to close watchdog client"); err != nil {
			return
		}
		if n.rcs != nil {
			if err = errors.Wrap(n.rcs.Close(), "failed to close rcs client"); err != nil {
				return
			}
		}
		if err = errors.Wrapf(n.rasPub.Close(), "failed to close %s publisher", ras.StateChangeTopic.Topic); err != nil {
			return
		}
		if err = errors.Wrapf(n.getStateSrv.Close(), "failed to close %s service", srv_ras.GetRasStateServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.setStateSrv.Close(), "failed to close %s service", srv_ras.TransitionRasStateServiceName); err != nil {
			return
		}

		if err = errors.Wrap(n.n.Close(), "failed to close ros node"); err != nil {
			return
		}

		close(n.close)
	})

	if err != nil {
		return err
	}

	return nil
}
