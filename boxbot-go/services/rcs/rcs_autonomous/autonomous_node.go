package main

import (
	"boxbot-go/internal/rcs_states"
	"boxbot-go/pkg/carton_search"
	"boxbot-go/pkg/event"
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	rcsmsgs "boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/robot_move"
	"boxbot-go/pkg/srvs/srv_rcs"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/looplab/fsm"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"sync"
)

type autonomous struct {
	n                                                                                  *goroslib.Node
	statePub                                                                           *goroslib.Publisher
	startSrv                                                                           *goroslib.ServiceProvider
	shutdownSrv, pausedSrv, haltedSrv, emergencySrv, placeStackSrv, operatorUnstuckSrv *goroslib.ServiceProvider

	logger log.Logger

	close, done chan struct{}
	closeLock   sync.Once

	fsm *fsm.FSM
	c   Config

	move    robot_move.RobotMove
	search  carton_search.CartonSearcher
	gripper *gripper

	cartonLocations       []carton_search.Coordinate
	currentCarton         *atomic.Int32
	currentCartonLocation geometry_msgs.Quaternion
	currentLayer          *atomic.Int32

	// TODO(timanema): Maybe add extra checks to leave event based states, as there is a race condition
	// between the source - calibration - state
	emergencyEvent, haltEvent, pauseEvent *event.Event
	homed, stuck, empty                   *atomic.Bool
	cartonsAdded, stuckFixed              *atomic.Bool
	rcsStarted, stopStarted               *atomic.Bool
	currentRunState                       rcs_states.RcsState
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &autonomous{
		close:           make(chan struct{}, 1),
		done:            make(chan struct{}),
		currentCarton:   atomic.NewInt32(0),
		currentLayer:    atomic.NewInt32(0),
		homed:           atomic.NewBool(false),
		stuck:           atomic.NewBool(false),
		empty:           atomic.NewBool(false),
		rcsStarted:      atomic.NewBool(false),
		stopStarted:     atomic.NewBool(false),
		cartonsAdded:    atomic.NewBool(false),
		stuckFixed:      atomic.NewBool(false),
		currentRunState: rcs_states.InitState,
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "unable to read config")
	}

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.RcsAutonomousName,
		Namespace:     node.RcsNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.RcsAutonomousName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create state publisher
	n.statePub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n.n,
		Topic: rcsmsgs.StateChangeTopic.Topic,
		Msg:   rcsmsgs.StateChangeTopic.Msg,
		Latch: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s publisher", rcsmsgs.StateChangeTopic.Topic)
	}

	// Create services
	n.startSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsStartServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.handleStart,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsStartServiceName)
	}

	n.shutdownSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsShutdownServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.handleShutdown,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsShutdownServiceName)
	}

	n.pausedSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsSetPausedServiceName,
		Srv:      &std_srvs.SetBool{},
		Callback: n.handleSetPaused,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsSetPausedServiceName)
	}

	n.haltedSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsSetHaltedServiceName,
		Srv:      &std_srvs.SetBool{},
		Callback: n.handleSetHalted,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsSetHaltedServiceName)
	}

	n.emergencySrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsSetEmergencyServiceName,
		Srv:      &std_srvs.SetBool{},
		Callback: n.handleSetEmergency,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsSetEmergencyServiceName)
	}

	n.placeStackSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsStackPlacedServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.handlePlaceStack,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsStackPlacedServiceName)
	}

	n.operatorUnstuckSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.RcsOperatorUnstuckServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.handleOperatorUnstuck,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.RcsOperatorUnstuckServiceName)
	}

	// Create interfaces with robot
	n.move, err = robot_move.New(n.n, n.logger)
	if err != nil {
		return nil, errors.Wrap(err, "failed to create move link")
	}

	n.gripper = NewGripper(n.c, n.logger, n.move)
	n.search = carton_search.New()

	// If the node is started in gripper test mode, do not start the fsm, but instead simply start the gripping process
	if n.c.EnableGripperTest {
		n.logger.Logf(msgs.WARN, "RCS FSM IS STARTING IN GRIPPER TEST MODE")

		// Gripper close test
		n.logger.Logf(msgs.WARN, "OPENING GRIPPER")
		fmt.Println(n.gripper.ToggleGripper(true))

		n.logger.Logf(msgs.WARN, "CLOSING GRIPPER")
		fmt.Println(n.gripper.ToggleGripper(false))
		fmt.Println(n.gripper.WaitForClose())

		n.logger.Logf(msgs.WARN, "OPENING GRIPPER")
		fmt.Println(n.gripper.ToggleGripper(true))

		if n.c.EnableGripperTestHome {
			n.logger.Logf(msgs.WARN, "GRIPPER TEST HOME")
			err := n.move.Home(func(err error) {
				if err != nil {
					n.logger.Logf(msgs.ERROR, "failed to home system for test: %v", err)
					return
				}

				fmt.Println("HOMING DONE")
				//TODO(timanema): Remove test statement
				loc, err := n.move.GetLocation()
				fmt.Printf("location after homing: %v (%v)\n", loc, err)

				go n.gripperTest()
			})
			if err != nil {
				n.logger.Logf(msgs.ERROR, "failed to home system: %v", err)
			}
		} else {
			go n.gripperTest()
		}
	} else {
		// Create FSM
		n.createFSM()
	}

	go node.PingSelf(n.n, n.close, n.done, node.RcsAutonomousName)

	return n, err
}

func (n *autonomous) gripperTest() {
	n.currentCartonLocation = geometry_msgs.Quaternion{
		X: -0.3275,
		Y: 0.899,
		Z: 1.18,
		W: -0.069813,
	}
	err := n.move.Move(n.currentCartonLocation, func(err error) {
		if err != nil {
			n.logger.Logf(msgs.ERROR, "error while moving to test location: %v", err)
			return
		}
		if e := n.grip(); e != "" {
			n.logger.Logf(msgs.INFO, "FSM would have made change to %s\n", e)
		}
	})
	if err != nil {
		n.logger.Logf(msgs.ERROR, "unable to move gripper to basic test location: %v", err)
	}
}

//-0.32349057895908206, 0.44421033131416154, 0.27993636066076677
func (n *autonomous) handleStart(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	n.rcsStarted.Store(true)
	n.canTransition(rcs_states.RcsStart)

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *autonomous) handleShutdown(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	n.stopStarted.Store(true)
	n.canTransition(rcs_states.StopRcs)

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *autonomous) handleSet(e *event.Event, active bool, name string) error {
	if active {
		return errors.Wrapf(e.Active(), "error setting %s event to active", name)
	} else {
		return errors.Wrapf(e.Inactive(), "error setting %s event to inactive", name)
	}
}

func (n *autonomous) handleSetPaused(req *std_srvs.SetBoolReq) *std_srvs.SetBoolRes {
	if err := n.handleSet(n.pauseEvent, req.Data, "pause"); err != nil {
		return &std_srvs.SetBoolRes{
			Success: false,
			Message: err.Error(),
		}
	}

	return &std_srvs.SetBoolRes{
		Success: true,
	}
}

func (n *autonomous) handleSetHalted(req *std_srvs.SetBoolReq) *std_srvs.SetBoolRes {
	if err := n.handleSet(n.haltEvent, req.Data, "halt"); err != nil {
		return &std_srvs.SetBoolRes{
			Success: false,
			Message: err.Error(),
		}
	}

	return &std_srvs.SetBoolRes{
		Success: true,
	}
}

func (n *autonomous) handleSetEmergency(req *std_srvs.SetBoolReq) *std_srvs.SetBoolRes {
	if err := n.handleSet(n.emergencyEvent, req.Data, "emergency"); err != nil {
		return &std_srvs.SetBoolRes{
			Success: false,
			Message: err.Error(),
		}
	}

	return &std_srvs.SetBoolRes{
		Success: true,
	}
}

func (n *autonomous) handlePlaceStack(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	n.cartonsAdded.Store(true)
	n.canTransition(rcs_states.OperatorAddedCartons)

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *autonomous) handleOperatorUnstuck(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	n.stuckFixed.Store(true)
	n.canTransition(rcs_states.OperatorFixedStuck)

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *autonomous) Done() <-chan struct{} {
	return n.done
}

func (n *autonomous) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrapf(n.statePub.Close(), "failed to close %s publisher", rcsmsgs.StateChangeTopic.Topic); err != nil {
			return
		}
		if err = errors.Wrapf(n.startSrv.Close(), "failed to close %s service", srv_rcs.RcsStartServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.shutdownSrv.Close(), "failed to close %s service", srv_rcs.RcsShutdownServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.pausedSrv.Close(), "failed to close %s service", srv_rcs.RcsSetPausedServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.haltedSrv.Close(), "failed to close %s service", srv_rcs.RcsSetHaltedServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.emergencySrv.Close(), "failed to close %s service", srv_rcs.RcsSetEmergencyServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.placeStackSrv.Close(), "failed to close %s service", srv_rcs.RcsStackPlacedServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.operatorUnstuckSrv.Close(), "failed to close %s service", srv_rcs.RcsOperatorUnstuckServiceName); err != nil {
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
