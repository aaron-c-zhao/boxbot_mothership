package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/servo_serial"
	"boxbot-go/pkg/srvs/srv_rcs"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"math"
	"sync"
	"time"
)

const (
	reconnectDelay = time.Second
	readyPollDelay = time.Second
)

type stepper struct {
	n        *goroslib.Node
	servoSub *goroslib.Subscriber

	homeSrv, waitSrv *goroslib.ServiceProvider

	servo   *servo_serial.SerialServo
	blocked *atomic.Bool

	c Config

	logger log.Logger

	close, done chan struct{}
	closeLock   sync.Once
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &stepper{
		close:   make(chan struct{}, 5),
		blocked: atomic.NewBool(false),
		done:    make(chan struct{}),
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "failed to read config for servo node")
	}

	// Create motors
	n.servo = servo_serial.New(n.c.ServoDevice)

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.ServoDriverName,
		Namespace:     node.RcsNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.ServoDriverName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create subscribers
	n.servoSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n.n,
		Topic:    rcs.JointControlTopic.Topic,
		Callback: n.setServo,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s subscriber", rcs.ServoTopic.Topic)
	}

	// Create services
	n.homeSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.ServoHomeServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.home,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.ServoHomeServiceName)
	}
	n.waitSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_rcs.ServoWaitServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.waitForServo,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_rcs.ServoWaitServiceName)
	}

	go node.PingSelf(n.n, n.close, n.done, node.ServoDriverName)

	// Open connections to both motors
	go n.start(n.servo)

	return n, err
}

func (n *stepper) home(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	n.blocked.Store(true)
	defer n.blocked.Store(false)

	// Set position
	if err := n.servo.SetPosition(n.c.ServoHomeLocation, n.c.ServoSpeed); err != nil {
		n.logger.Logf(msgs.WARN, "failed to set servo position to home position: %v", err)
		return &std_srvs.TriggerRes{
			Success: false,
			Message: errors.Wrap(err, "failed to home servo").Error(),
		}
	}

	// Give the servo some time to continue
	time.Sleep(time.Millisecond * 500)

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *stepper) waitForServo(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	ticker := time.NewTicker(readyPollDelay)

	for {
		select {
		case <-n.Done():
			return &std_srvs.TriggerRes{
				Success: false,
				Message: "servo node stopping",
			}
		case <-ticker.C:
		}

		ready, err := n.servo.AtTarget()
		if err != nil {
			return &std_srvs.TriggerRes{
				Success: false,
				Message: errors.Wrap(err, "unable to retrieve servo status").Error(),
			}
		}

		if ready {
			return &std_srvs.TriggerRes{
				Success: true,
			}
		}
	}
}

func (n *stepper) start(motor *servo_serial.SerialServo) {
	var err error
	ticker := time.NewTicker(reconnectDelay)

	for range ticker.C {
		select {
		case <-n.Done():
			return
		default:
		}

		if err = motor.Start(); err != nil {
			n.logger.Logf(msgs.ERROR, "unable to start serial communication with servo controller at %s: %v", motor.Port, err)
		} else {
			break
		}
	}
}

func (n *stepper) setServo(req *rcs.JointControl) {
	//n.logger.LogLocalf(msgs.DEBUG, "received set motor req: %v", req)

	for _, ctr := range req.Ctr {
		if ctr.MotorID == n.c.ServoMotorID && !n.blocked.Load() {
			if err := n.servo.SetPosition(float64(ctr.Position)*(180.0/math.Pi), n.c.ServoSpeed); err != nil {
				n.logger.Logf(msgs.WARN, "failed to set servo position: %v", err)
			}
		}
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
		if err = errors.Wrap(n.servo.Close(), "failed to close connection to servo"); err != nil {
			return
		}
		if err = errors.Wrap(n.servoSub.Close(), "failed to close servo subscriber"); err != nil {
			return
		}
		if err = errors.Wrap(n.homeSrv.Close(), "failed to close home service"); err != nil {
			return
		}
		if err = errors.Wrap(n.waitSrv.Close(), "failed to close wait service"); err != nil {
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
