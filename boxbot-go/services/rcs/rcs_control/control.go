package main

import (
	"boxbot-go/pkg/ik_solver"
	"boxbot-go/pkg/path_solver"
	"fmt"
	"github.com/pkg/errors"
	"time"
)

const (
	loopFrequencyHz   = 50
	actionGracePeriod = path_solver.CreepDuration + 0.5
)

type SimpleControl struct {
	currentState  ik_solver.JointState
	currentTarget *path_solver.SimplePath

	transmissions *TransmissionMap

	targetCh chan *path_solver.SimplePath
	stopCh   <-chan struct{}
	doneCh   chan struct{}
}

func NewSimpleControl(initialState ik_solver.JointState, stopCh <-chan struct{}, transmissions *TransmissionMap) *SimpleControl {
	transmissions.Verify()

	return &SimpleControl{
		currentState:  initialState,
		targetCh:      make(chan *path_solver.SimplePath, 1),
		stopCh:        stopCh,
		doneCh:        make(chan struct{}),
		transmissions: transmissions,
	}
}

//TODO(timanema): Use forward kinematics to provide a better error message
func (c *SimpleControl) SetTarget(target ik_solver.Coordinate) error {
	start := time.Now()

	// Calculate the joint states
	s, err := ik_solver.FindSolution(target)
	if err != nil {
		return errors.Wrapf(err, "unable to find a set of joint states which result in %v", target)
	}

	// Find a simple path
	path, err := path_solver.FindSimplePath(c.currentState, s)
	if err != nil {
		return errors.Wrapf(err, "unable to find a path from %v to %v", c.currentState, target)
	}

	c.targetCh <- &path
	fmt.Printf("found a solution from %v to %v in %v\n", c.currentState, target, time.Now().Sub(start))
	return nil
}

func (c *SimpleControl) Moving() bool {
	return c.currentTarget != nil
}

func (c *SimpleControl) Done() <-chan struct{} {
	return c.doneCh
}

func (c *SimpleControl) Start() {
	delay := time.Second / time.Duration(loopFrequencyHz)
	rate := 1.0 / float64(loopFrequencyHz)

	tm := time.NewTicker(delay)
	t := time.Now()
	prev := t
	diff := 0.0

	actionTime := 0.0
	actionLength := 1.0

	for {
		select {
		case now := <-tm.C:
			diff = float64(now.Sub(prev)-delay) / float64(time.Second)
			prev = now
		case target := <-c.targetCh:
			//TODO(timanema): Probably have to get position before switching target
			c.currentTarget = target
			actionTime = 0.0
			actionLength = target.Duration
			fmt.Printf("starting task with duration %fs\n", actionLength)
		case <-c.stopCh:
			fmt.Println("stopping")
			_ = c.Close()
			return
		}

		// If no target, wait
		if c.currentTarget == nil {
			continue
		}

		// If action completed, remove it
		if actionTime > actionLength+actionGracePeriod {
			c.currentState = c.currentTarget.Target
			c.currentTarget = nil
			fmt.Println("action done, removing")
			continue
		}

		actionTime += rate + diff
		fmt.Println("----------")
		c.SetMotor(ArmMotor, c.currentTarget.ArmMap.GetVel(actionTime), c.currentTarget.Target.ArmLength)
		c.SetMotor(BaseMotor, c.currentTarget.BaseMap.GetVel(actionTime), c.currentTarget.Target.BaseAngleRad)
		c.SetMotor(GripperServo, c.currentTarget.ServoMap.GetVel(actionTime), c.currentTarget.Target.ServoAngleDegrees)
		c.SetMotor(TowerMotor, c.currentTarget.TowerMap.GetVel(actionTime), c.currentTarget.Target.TowerHeight)
		fmt.Println("----------")
	}
}

func (c *SimpleControl) SetMotor(m MotorType, speed, position float64) {
	fmt.Printf("set motor %d: %f, %f\n", m, speed, position)
}

func (c *SimpleControl) Close() error {
	close(c.doneCh)
	return nil
}
