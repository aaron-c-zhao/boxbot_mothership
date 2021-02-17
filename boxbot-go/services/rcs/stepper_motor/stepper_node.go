package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/srvs/srv_rcs"
	"boxbot-go/pkg/stepper_serial"
	"context"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"golang.org/x/sync/errgroup"
	"math"
	"sync"
	"time"
)

const (
	reconnectDelay                 = time.Second
	moveIndefinitelyDistance int32 = 50000
)

var TargetNotReachedErr = errors.New("unable to reach target, since speed is zero")

type stepper struct {
	n                                                   *goroslib.Node
	getMotorStatusSrv, homeSrv, cancelHomeSrv, readySrv *goroslib.ServiceProvider
	moveSrv, moveStopSrv, blockSrv                      *goroslib.ServiceProvider
	motorSub                                            *goroslib.Subscriber

	baseMotor, armMotor, towerMotor                *stepper_serial.SerialStepper
	baseTarget, armTarget, towerTarget             int32
	baseIndefStart, armIndefStart, towerIndefStart int32

	c Config

	logger log.Logger

	close, done chan struct{}
	closeLock   sync.Once
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &stepper{
		close: make(chan struct{}, 5),
		done:  make(chan struct{}),
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "failed to read config for stepper node")
	}

	// Create motors
	n.baseMotor = stepper_serial.New(n.c.BaseMotorDevice)
	n.armMotor = stepper_serial.New(n.c.ArmMotorDevice)
	n.towerMotor = stepper_serial.New(n.c.TowerMotorDevice)

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.StepperDriverName,
		Namespace:     node.RcsNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.StepperDriverName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create subscribers
	n.motorSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n.n,
		Topic:    rcs.JointControlTopic.Topic,
		Callback: n.setMotors,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s subscriber", rcs.JointControlTopic.Topic)
	}

	// Create services
	n.getMotorStatusSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.GetMotorStatusServiceName,
		Srv:      &srv_rcs.GetMotorStatusService{},
		Callback: n.getMotorStatus,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.GetMotorStatusServiceName)
	}
	n.homeSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.HomeServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.home,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.HomeServiceName)
	}
	n.cancelHomeSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.CancelHomeServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.cancelHome,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.CancelHomeServiceName)
	}
	n.readySrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.ReadyServiceName,
		Srv:      &srv_rcs.MotorReadyService{},
		Callback: n.motorReady,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.ReadyServiceName)
	}

	n.moveSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.MotorMoveServiceName,
		Srv:      &srv_rcs.MotorMoveService{},
		Callback: n.moveIndefinitelyHandler,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.MotorMoveServiceName)
	}
	n.moveStopSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.MotorMoveStopServiceName,
		Srv:      &srv_rcs.MotorMoveStopService{},
		Callback: n.stopMoveIndefinitelyHandler,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.MotorMoveStopServiceName)
	}
	n.blockSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.BlockMotorServiceName,
		Srv:      &srv_rcs.BlockMotorService{},
		Callback: n.blockMotor,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.BlockMotorServiceName)
	}

	go node.PingSelf(n.n, n.close, n.done, node.StepperDriverName)

	// Open connections to all motors
	if n.c.BaseMotorEnabled {
		go n.start(n.baseMotor, "base")
	}
	if n.c.TowerMotorEnabled {
		go n.start(n.towerMotor, "tower")
	}
	if n.c.ArmMotorEnabled {
		go n.start(n.armMotor, "arm")
	}

	return n, err
}

func (n *stepper) start(motor *stepper_serial.SerialStepper, name string) {
	var err error
	ticker := time.NewTicker(reconnectDelay)

	for range ticker.C {
		if err = motor.Start(); err != nil {
			n.logger.Logf(msgs.ERROR, "unable to start serial communication with stepper controller for %s"+
				" at %s: %v", name, motor.Port, err)
		} else {
			break
		}
	}
}

func (n *stepper) moveIndefinitely(motor *stepper_serial.SerialStepper, name string, speed float32, reverse, left bool) (*srv_rcs.MotorMoveRes, int32) {
	if !motor.Blocked() {
		return &srv_rcs.MotorMoveRes{
			Success: false,
			Message: fmt.Sprintf("%s motor blocked", name),
		}, 0
	}

	// Block motor and stop it
	motor.Block(true)
	if err := motor.Stop(); err != nil {
		return &srv_rcs.MotorMoveRes{
			Success: false,
			Message: errors.Wrapf(err, "unable to stop %s motor", name).Error(),
		}, 0
	}

	// Save location and move to a position far away
	status, err := motor.GetStatus()
	if err != nil {
		return &srv_rcs.MotorMoveRes{
			Success: false,
			Message: errors.Wrapf(err, "unable to get status of %s motor", name).Error(),
		}, 0
	}

	start := status.Location
	target := moveIndefinitelyDistance

	// Check if the motor is working in reverse
	if reverse {
		target *= -1
	}

	// Check if the motor needs to go to high (left) or low (right) side
	if !left {
		target *= -1
	}

	// Finally set the actual target position
	err, _ = motor.IgnoreBlockSetMotor(&rcs.Joint{
		Position: float32(target),
	}, speed, reverse, true, true)
	if err != nil {
		return &srv_rcs.MotorMoveRes{
			Success: false,
			Message: errors.Wrapf(err, "unable to set target position for %s motor", name).Error(),
		}, 0
	}

	return &srv_rcs.MotorMoveRes{
		Success: true,
	}, start
}

func (n *stepper) moveIndefinitelyHandler(req *srv_rcs.MotorMoveReq) *srv_rcs.MotorMoveRes {
	switch req.MotorID {
	case n.c.BaseMotorID:
		if !n.c.BaseMotorEnabled {
			return &srv_rcs.MotorMoveRes{
				Success: true,
			}
		}

		res, start := n.moveIndefinitely(n.baseMotor, "base", n.c.BaseMotorIndefiniteSpeed,
			n.c.BaseMotorReversePosition, req.Left)
		if res.Success {
			n.baseIndefStart = start
		}

		return res
	case n.c.TowerMotorID:
		if !n.c.TowerMotorEnabled {
			return &srv_rcs.MotorMoveRes{
				Success: true,
			}
		}

		res, start := n.moveIndefinitely(n.towerMotor, "tower", n.c.TowerMotorIndefiniteSpeed,
			n.c.TowerMotorReversePosition, req.Left)
		if res.Success {
			n.towerIndefStart = start
		}

		return res
	case n.c.ArmMotorID:
		if !n.c.ArmMotorEnabled {
			return &srv_rcs.MotorMoveRes{
				Success: true,
			}
		}

		res, start := n.moveIndefinitely(n.armMotor, "arm", n.c.ArmMotorIndefiniteSpeed,
			n.c.ArmMotorReversePosition, req.Left)
		if res.Success {
			n.armIndefStart = start
		}

		return res
	}

	return nil
}

func (n *stepper) stopMoveIndefinitely(motor *stepper_serial.SerialStepper, name string) (int32, error) {
	if err := motor.Stop(); err != nil {
		return 0, errors.Wrapf(err, "failed to stop %s motor", name)
	}

	status, err := motor.GetStatus()
	if err != nil {
		return 0, errors.Wrapf(err, "failed to get status of %s motor", name)
	}

	return status.Location, nil
}

func (n *stepper) stopMoveIndefinitelyHandler(req *srv_rcs.MotorMoveStopReq) *srv_rcs.MotorMoveStopRes {
	switch req.MotorID {
	case n.c.BaseMotorID:
		if !n.c.BaseMotorEnabled {
			return &srv_rcs.MotorMoveStopRes{
				Success: true,
			}
		}

		end, err := n.stopMoveIndefinitely(n.baseMotor, "base")
		if err != nil {
			return &srv_rcs.MotorMoveStopRes{
				Success: false,
				Message: errors.Wrap(err, "failed to stop the indefinite move").Error(),
			}
		}

		return &srv_rcs.MotorMoveStopRes{
			Distance: int32(math.Abs(math.Abs(float64(n.baseIndefStart)) - math.Abs(float64(end)))),
			Start:    n.baseIndefStart,
			Stop:     end,
			Success:  true,
		}
	case n.c.TowerMotorID:
		if !n.c.TowerMotorEnabled {
			return &srv_rcs.MotorMoveStopRes{
				Success: true,
			}
		}

		end, err := n.stopMoveIndefinitely(n.towerMotor, "tower")
		if err != nil {
			return &srv_rcs.MotorMoveStopRes{
				Success: false,
				Message: errors.Wrap(err, "failed to stop the indefinite move").Error(),
			}
		}

		return &srv_rcs.MotorMoveStopRes{
			Distance: int32(math.Abs(math.Abs(float64(n.towerIndefStart)) - math.Abs(float64(end)))),
			Start:    n.towerIndefStart,
			Stop:     end,
			Success:  true,
		}
	case n.c.ArmMotorID:
		if !n.c.ArmMotorEnabled {
			return &srv_rcs.MotorMoveStopRes{
				Success: true,
			}
		}

		end, err := n.stopMoveIndefinitely(n.armMotor, "arm")
		if err != nil {
			return &srv_rcs.MotorMoveStopRes{
				Success: false,
				Message: errors.Wrap(err, "failed to stop the indefinite move").Error(),
			}
		}

		return &srv_rcs.MotorMoveStopRes{
			Distance: int32(math.Abs(math.Abs(float64(n.armIndefStart)) - math.Abs(float64(end)))),
			Start:    n.armIndefStart,
			Stop:     end,
			Success:  true,
		}
	default:
		return &srv_rcs.MotorMoveStopRes{
			Success: false,
			Message: fmt.Sprintf("unknown motor id %v", req.MotorID),
		}
	}
}

func (n *stepper) blockMotor(req *srv_rcs.BlockMotorReq) *srv_rcs.BlockMotorRes {
	switch req.MotorID {
	case n.c.BaseMotorID:
		n.baseMotor.Block(req.Blocked)
	case n.c.TowerMotorID:
		n.towerMotor.Block(req.Blocked)
	case n.c.ArmMotorID:
		n.armMotor.Block(req.Blocked)
	default:
		return &srv_rcs.BlockMotorRes{
			Success: false,
			Message: fmt.Sprintf("unknown motor id %v", req.MotorID),
		}
	}

	return &srv_rcs.BlockMotorRes{
		Success: true,
	}
}

func (n *stepper) motorReady(_ *srv_rcs.MotorReadyReq) *srv_rcs.MotorReadyRes {
	errGr, _ := errgroup.WithContext(context.TODO())

	var base, tower, arm bool

	// Wait for all motors
	errGr.Go(func() error {
		if !n.c.BaseMotorEnabled {
			base = true
			return nil
		}

		status, err := n.baseMotor.GetStatus()
		if err != nil {
			return errors.Wrap(err, "failed to retrieve status for base motor")
		}

		base = status.Location == n.baseTarget

		// If speed is zero while location has not been reached, cancel
		if !base && status.Speed == 0 {
			return TargetNotReachedErr
		}
		return nil
	})

	errGr.Go(func() error {
		if !n.c.TowerMotorEnabled {
			tower = true
			return nil
		}

		status, err := n.towerMotor.GetStatus()
		if err != nil {
			return errors.Wrap(err, "failed to retrieve status for tower motor")
		}

		tower = status.Location == n.towerTarget

		// If speed is zero while location has not been reached, cancel
		if !tower && status.Speed == 0 {
			return TargetNotReachedErr
		}
		return nil
	})

	errGr.Go(func() error {
		if !n.c.ArmMotorEnabled {
			arm = true
			return nil
		}

		status, err := n.armMotor.GetStatus()
		if err != nil {
			return errors.Wrap(err, "failed to retrieve status for arm motor")
		}

		arm = status.Location == n.armTarget

		// If speed is zero while location has not been reached, cancel
		if !arm && status.Speed == 0 {
			return TargetNotReachedErr
		}
		return nil
	})

	if err := errGr.Wait(); err != nil {
		return &srv_rcs.MotorReadyRes{
			Ready:   false,
			Failed:  true,
			Message: errors.Wrap(err, "unable to wait for all motors").Error(),
		}
	}

	return &srv_rcs.MotorReadyRes{
		Ready: base && tower && arm,
	}
}

func (n *stepper) setMotors(req *rcs.JointControl) {
	//n.logger.LogLocalf(msgs.DEBUG, "received set motor req: %v", req)

	for _, ctr := range req.Ctr {
		switch ctr.MotorID {
		case n.c.BaseMotorID:
			if !n.c.BaseMotorEnabled {
				continue
			}
			speed := n.c.BaseMotorCruiseSpeed

			// Cap the speed if applicable
			if speed > n.c.BaseMotorMaxSpeed {
				n.logger.Logf(msgs.WARN, "received speed %v (steps/s) for base motor, is more than max speed (%v), "+
					"capping the speed", speed, n.c.BaseMotorMaxSpeed)
				speed = n.c.BaseMotorMaxSpeed
			}

			err, target := n.baseMotor.SetMotor(&ctr, speed, n.c.BaseMotorReversePosition)
			if err != nil {
				n.logger.Logf(msgs.WARN, "connecting to base motor failed: %v", err)
			} else {
				n.baseTarget = target
			}
		case n.c.TowerMotorID:
			if !n.c.TowerMotorEnabled {
				continue
			}
			speed := n.c.TowerMotorCruiseSpeed

			// Cap the speed if applicable
			if speed > n.c.TowerMotorMaxSpeed {
				n.logger.Logf(msgs.WARN, "received speed %v (steps/s) for tower motor, is more than max speed (%v), "+
					"capping the speed", speed, n.c.TowerMotorMaxSpeed)
				speed = n.c.TowerMotorCruiseSpeed
			}

			err, target := n.towerMotor.SetMotor(&ctr, speed, n.c.TowerMotorReversePosition)
			if err != nil {
				n.logger.Logf(msgs.WARN, "connecting to tower motor failed: %v", err)
			} else {
				n.towerTarget = target
			}
		case n.c.ArmMotorID:
			if !n.c.ArmMotorEnabled {
				continue
			}
			speed := n.c.ArmMotorCruiseSpeed

			// Cap the speed if applicable
			if speed > n.c.ArmMotorMaxSpeed {
				n.logger.Logf(msgs.WARN, "received speed %v (steps/s) for arm motor, is more than max speed (%v), "+
					"capping the speed", speed, n.c.ArmMotorMaxSpeed)
				speed = n.c.ArmMotorMaxSpeed
			}

			err, target := n.armMotor.SetMotor(&ctr, speed, n.c.ArmMotorReversePosition)
			if err != nil {
				n.logger.Logf(msgs.WARN, "connecting to arm motor failed: %v", err)
			} else {
				n.armTarget = target
			}
		}
	}
}

func (n *stepper) getMotorStatus(_ *srv_rcs.GetMotorStatusReq) *srv_rcs.GetMotorStatusRes {
	var err error
	res := &srv_rcs.GetMotorStatusRes{
		Motors: make([]srv_rcs.Status, 3),
	}

	// Load base motor status
	if n.c.BaseMotorEnabled {
		res.Motors[0], err = n.baseMotor.GetStatus()
		if err != nil {
			res.Motors[0] = srv_rcs.Status{
				Idle:  true,
				Fault: true,
			}
		}
	}

	// Load tower motor status
	if n.c.TowerMotorEnabled {
		res.Motors[1], err = n.towerMotor.GetStatus()
		if err != nil {
			res.Motors[1] = srv_rcs.Status{
				Idle:  true,
				Fault: true,
			}
		}
	}

	// Load arm motor status
	if n.c.ArmMotorEnabled {
		res.Motors[2], err = n.armMotor.GetStatus()
		if err != nil {
			res.Motors[2] = srv_rcs.Status{
				Idle:  true,
				Fault: true,
			}
		}
	}

	return res
}

func (n *stepper) home(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	n.logger.Logf(msgs.INFO, "homing motors")
	errGr, _ := errgroup.WithContext(context.TODO())

	defer func() {
		// Unblock all motors when this function returns
		n.baseMotor.Block(false)
		n.towerMotor.Block(false)
		n.armMotor.Block(false)
	}()

	// Home base motor
	errGr.Go(func() error {
		if !n.c.BaseMotorEnabled {
			return nil
		}

		// Block motor
		n.baseMotor.Block(true)

		if err := n.baseMotor.StartHoming(n.c.BaseMotorHomingSpeed); err != nil {
			return errors.Wrap(err, "unable to home base motor")
		}

		n.logger.Logf(msgs.DEBUG, "base motor homed, moving to offset at %v", n.c.BaseMotorHomingOffset)

		// Move base motor to its home offset
		if err, _ := n.baseMotor.IgnoreBlockSetMotor(&rcs.Joint{
			Position: n.c.BaseMotorHomingOffset,
		}, n.c.BaseMotorCruiseSpeed, n.c.BaseMotorReversePosition, true, true); err != nil {
			n.logger.Logf(msgs.WARN, "unable to move base motor to home offset: %v", err)
			return errors.Wrap(err, "unable to move base motor to home offset")
		}

		// Wait until the offset has been reached
		target := n.c.BaseMotorHomingOffset
		if n.c.BaseMotorReversePosition {
			target *= -1
		}

		if err := n.baseMotor.WaitForPosition(int32(target), false); err != nil {
			n.logger.Logf(msgs.WARN, "failed to wait for base motor to move to home offset: %v", err)
			return errors.Wrap(err, "failed to wait for base motor to move to home offset")
		}

		// Zero the motor
		if err := n.baseMotor.Zero(); err != nil {
			n.logger.Logf(msgs.WARN, "unable to set zero position for base motor: %v", err)
			return errors.Wrap(err, "unable to set zero position for base motor")
		}

		return nil
	})

	// Home tower motor
	errGr.Go(func() error {
		if !n.c.TowerMotorEnabled {
			return nil
		}

		n.towerMotor.Block(true)

		return errors.Wrap(n.towerMotor.StartHoming(n.c.TowerMotorHomingSpeed), "failed to home tower motor")
	})

	// Home arm motor
	errGr.Go(func() error {
		if !n.c.ArmMotorEnabled {
			return nil
		}

		n.armMotor.Block(true)

		return errors.Wrap(n.armMotor.StartHoming(n.c.ArmMotorHomingSpeed), "failed to home arm motor")
	})

	if err := errGr.Wait(); err != nil {
		return &std_srvs.TriggerRes{
			Success: false,
			Message: errors.Wrap(err, "failed to home motors").Error(),
		}
	}

	n.logger.Logf(msgs.INFO, "motors homed")

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *stepper) cancelHome(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	errGr, _ := errgroup.WithContext(context.TODO())

	// Cancel home base motor
	errGr.Go(func() error {
		if !n.c.BaseMotorEnabled {
			return nil
		}

		n.logger.Logf(msgs.INFO, "cancelling base motor home")
		return errors.Wrap(n.baseMotor.StopHoming(), "unable to cancel home base motor")
	})

	// Cancel home tower motor
	errGr.Go(func() error {
		if !n.c.TowerMotorEnabled {
			return nil
		}

		n.logger.Logf(msgs.INFO, "cancelling tower motor home")
		return errors.Wrap(n.towerMotor.StopHoming(), "failed to cancel home tower motor")
	})

	// Cancel home arm motor
	errGr.Go(func() error {
		if !n.c.ArmMotorEnabled {
			return nil
		}

		n.logger.Logf(msgs.INFO, "cancelling arm motor home")
		return errors.Wrap(n.armMotor.StopHoming(), "failed to cancel home arm motor")
	})

	if err := errGr.Wait(); err != nil {
		return &std_srvs.TriggerRes{
			Success: false,
			Message: errors.Wrap(err, "failed to cancel home motors").Error(),
		}
	}

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *stepper) Done() <-chan struct{} {
	return n.done
}

func (n *stepper) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrap(n.baseMotor.Close(), "failed to close connection to base motor"); err != nil {
			return
		}
		if err = errors.Wrap(n.towerMotor.Close(), "failed to close connection to tower motor"); err != nil {
			return
		}
		if err = errors.Wrap(n.armMotor.Close(), "failed to close connection to arm motor"); err != nil {
			return
		}
		if err = errors.Wrap(n.motorSub.Close(), "failed to close motor subscriber"); err != nil {
			return
		}
		if err = errors.Wrap(n.getMotorStatusSrv.Close(), "failed to close motor service"); err != nil {
			return
		}
		if err = errors.Wrap(n.homeSrv.Close(), "failed to close home service"); err != nil {
			return
		}
		if err = errors.Wrap(n.cancelHomeSrv.Close(), "failed to close ready service"); err != nil {
			return
		}
		if err = errors.Wrap(n.readySrv.Close(), "failed to close cancel home service"); err != nil {
			return
		}
		if err = errors.Wrap(n.moveSrv.Close(), "failed to close cancel move indefinitely service"); err != nil {
			return
		}
		if err = errors.Wrap(n.moveStopSrv.Close(), "failed to close cancel stop move indefinitely service"); err != nil {
			return
		}
		if err = errors.Wrap(n.blockSrv.Close(), "failed to close cancel stop motor block service"); err != nil {
			return
		}

		if err = errors.Wrap(n.n.Close(), "failed to close ros node"); err != nil {
			return
		}
	})

	if err != nil {
		return err
	}

	n.close <- struct{}{}

	return nil
}
