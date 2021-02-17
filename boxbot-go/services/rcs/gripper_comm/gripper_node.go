package main

import (
	gripper_serial "boxbot-go/pkg/gripper_serial_v2"
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/node"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"sync"
)

type gripper struct {
	n                 *goroslib.Node
	getMotorStatusSrv *goroslib.ServiceProvider
	motorSub          *goroslib.Subscriber

	gripper *gripper_serial.SerialGripper

	c Config

	logger log.Logger

	close, done chan struct{}
	closeLock   sync.Once
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &gripper{
		close: make(chan struct{}, 1),
		done:  make(chan struct{}),
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "failed to read config for stepper node")
	}

	// Create link to gripper
	n.gripper = gripper_serial.New(n.c.GripperDevice, n.handleGripSuccess, n.handleGripFailed,
		n.handleHit, n.handleStartHigher, n.handleRightEdgeFound, n.handleHighLowEdgeFound)

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.GripperDriverName,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.GripperDriverName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	go node.PingSelf(n.n, n.close, n.done, node.GripperDriverName)

	return n, nil
}

func (g *gripper) handleGripSuccess() {

}

func (g *gripper) handleGripFailed() {

}

func (g *gripper) handleHit(sensor gripper_serial.Sensor) {

}

func (g *gripper) handleStartHigher() {

}

func (g *gripper) handleRightEdgeFound() {

}

func (g *gripper) handleHighLowEdgeFound() {

}

func (g *gripper) Done() <-chan struct{} {
	return g.done
}

func (g *gripper) Close() error {
	var err error
	g.closeLock.Do(func() {
		if err = errors.Wrap(g.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrap(g.n.Close(), "failed to close ros node"); err != nil {
			return
		}
	})

	if err != nil {
		return err
	}

	g.close <- struct{}{}

	return nil
}
