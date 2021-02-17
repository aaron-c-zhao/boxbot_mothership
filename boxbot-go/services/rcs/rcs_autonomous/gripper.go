package main

import (
	gripperserial "boxbot-go/pkg/gripper_serial_v2"
	"boxbot-go/pkg/gripper_solver"
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/robot_move"
	"fmt"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"time"
)

var GripperStoppedErr = errors.New("gripper was stopped")
var GripFailedErr = errors.New("gripping cartons failed")

const (
	closePollDelay = time.Second
)

//TODO(timanema): add way to cancel grip sequence
type gripper struct {
	gripper *gripperserial.SerialGripper
	move    robot_move.RobotMove

	searchCh chan error
	gripCh   chan error

	startLocation                                                               geometry_msgs.Quaternion
	latestUpperSensorLocation, latestLowerSensorLocation, latestHighLowLocation geometry_msgs.Quaternion

	rightEdgeFound, highLowEdgeFound, lowLayer *atomic.Bool

	c      Config
	logger log.Logger

	done chan struct{}
}

func NewGripper(c Config, logger log.Logger, move robot_move.RobotMove) *gripper {
	n := gripper{
		move:             move,
		c:                c,
		logger:           logger,
		searchCh:         make(chan error),
		gripCh:           make(chan error),
		rightEdgeFound:   atomic.NewBool(false),
		highLowEdgeFound: atomic.NewBool(false),
		lowLayer:         atomic.NewBool(false),
		done:             make(chan struct{}),
	}

	// Create link to gripper
	n.gripper = gripperserial.New(n.c.GripperDevice, n.handleGripSuccess, n.handleGripFailed,
		n.handleHit, n.handleStartHigher, n.handleRightEdgeFound, n.handleHighLowEdgeFound)

	// Start gripper comms
	go func() {
		if err := n.gripper.Start(); err != nil {
			n.logger.Logf(msgs.ERROR, "unable to start communication with gripper: %v", err)
		}
	}()

	return &n
}

//TODO(timanema): Implement later
func (g *gripper) ToggleGripper(open bool) error {
	fmt.Printf("setting gripper to opened = %v\n", open)
	return nil
}

//TODO(timanema): Implement later
func (g *gripper) Stop() error {
	return nil
}

func (g *gripper) WaitForClose() error {
	ticker := time.NewTicker(closePollDelay)

	for {
		select {
		case <-g.done:
			return errors.New("gripper connection is closing while waiting for close")
		case <-ticker.C:
		}

		open, err := g.gripper.IsOpen()
		if err != nil {
			return errors.Wrap(err, "failed to get open status from gripper")
		}

		if !open {
			return nil
		}
	}
}

func (g *gripper) StartSearch(start geometry_msgs.Quaternion, lowLayer bool) error {
	// Close previous channels
	close(g.gripCh)
	close(g.searchCh)

	// Init to starting values
	g.gripCh = make(chan error)
	g.searchCh = make(chan error)
	g.rightEdgeFound.Store(false)
	g.highLowEdgeFound.Store(false)
	g.lowLayer.Store(lowLayer)
	g.startLocation = start

	g.logger.Logf(msgs.INFO, "starting with searching at %v", start)

	if err := g.gripper.Reset(); err != nil {
		return errors.Wrap(err, "unable to reset gripper")
	}

	if err := g.gripper.StartSearch(); err != nil {
		return errors.Wrap(err, "unable to send start search command")
	}

	g.logger.Logf(msgs.DEBUG, "moving lower to find carton with switches")
	if err := g.move.DescendZ(true); err != nil {
		return errors.Wrap(err, "unable to send start descending of gripper")
	}

	return errors.Wrap(<-g.searchCh, "failed to search stack")
}

func (g *gripper) StartGrip() error {
	g.logger.Logf(msgs.DEBUG, "moving lower for %v to grip carton", g.c.GripHeight)

	// Move lower to grip stack
	if err := g.moveRelative(0, 0, -g.c.GripHeight, 0); err != nil {
		return errors.Wrap(err, "failed to move lower to grip stack")
	}

	// Signal gripper that it can start gripping
	if err := g.gripper.CommandCompleted(); err != nil {
		return errors.Wrap(err, "failed to signal gripper to start gripping")
	}

	err := errors.Wrap(<-g.gripCh, "failed to grip stack")

	// Stop the motors if the gripping failed
	if err != nil {
		if stopErr := g.move.Stop(); stopErr != nil {
			return errors.Wrapf(stopErr, "failed to stop motor after gripping failed (%v)", err)
		}
	}

	return nil
}

func (g *gripper) handleGripSuccess() {
	g.gripCh <- nil
}

func (g *gripper) handleGripFailed() {
	g.gripCh <- GripFailedErr
}

func (g *gripper) handleHit(sensor gripperserial.Sensor) {
	l, err := g.move.GetLocation()
	if err != nil {
		g.searchCh <- errors.Wrapf(err, "unable to get location for sensor %v", sensor)
		return
	}

	if sensor != gripperserial.LowerIR && sensor != gripperserial.UpperIR {
		g.logger.Logf(msgs.WARN, "ignoring invalid sensor hit for sensor %v", sensor)
		return
	}

	lower := sensor == gripperserial.LowerIR

	//TODO(timanema): Get phi and psi

	// Calculate sensor location
	loc := gripper_solver.FindCoordinateIrSensor(gripper_solver.Point{
		X: l.X,
		Y: l.Y,
	}, l.W, lower)

	// Reuse l struct
	l.X = loc.X
	l.Y = loc.Y

	g.logger.Logf(msgs.DEBUG, "sensor hit at %v (sensor = %v)", l, sensor)

	// Update position
	if !g.rightEdgeFound.Load() {
		if lower {
			g.logger.Logf(msgs.DEBUG, "lower sensor location set %v", l)
			g.latestLowerSensorLocation = l
		} else {
			g.logger.Logf(msgs.DEBUG, "higher sensor location set %v", l)
			g.latestUpperSensorLocation = l
		}
	} else if !g.highLowEdgeFound.Load() {
		g.logger.Logf(msgs.DEBUG, "high/low sensor location set %v", l)
		g.latestHighLowLocation = l
	} else {
		g.logger.Logf(msgs.WARN, "ignoring incorrectly timed sensor hit for sensor %v", sensor)
	}
}

func (g *gripper) handleStartHigher() {
	// Stop the descend of the gripper
	g.logger.Logf(msgs.DEBUG, "hit carton, stop moving lower")
	if err := g.move.DescendZ(false); err != nil {
		g.searchCh <- errors.Wrap(err, "unable to send start descending of gripper")
		return
	}

	// Increase Z-height to hover above carton
	g.logger.Logf(msgs.DEBUG, "moving high for %v to hover above carton", g.c.GripperHoverCartonHeight)
	if err := g.moveRelative(0, 0, g.c.GripperHoverCartonHeight, 0); err != nil {
		g.searchCh <- errors.Wrap(err, "failed to move to hover height for search")
		return
	}

	// Send signal to MCU to start looking
	if err := g.gripper.CommandCompleted(); err != nil {
		g.searchCh <- errors.Wrap(err, "failed to send command completed cmd")
		return
	}

	// Keep moving right until the right edge is found
	for !g.rightEdgeFound.Load() {
		g.logger.Logf(msgs.DEBUG, "moving right, since right edge has not yet been found")
		if err := g.moveDirection(RIGHT); err != nil && errors.Cause(err) != robot_move.MoveCancelledErr {
			g.searchCh <- errors.Wrap(err, "failed to move to the right for search IR sensors")
			return
		}
	}
}

func (g *gripper) handleRightEdgeFound() {
	// Set flag so the gripper stops moving right
	g.logger.Logf(msgs.DEBUG, "right edge has been found")
	g.rightEdgeFound.Store(true)

	// Stop previous movement
	g.logger.Logf(msgs.DEBUG, "stopping old movement to the right")
	//if err := g.move.Stop(); err != nil {
	//	g.searchCh <- errors.Wrap(err, "failed to stop moving to the right after right edge was found")
	//	return
	//}

	// Move back to starting position
	g.logger.Logf(msgs.DEBUG, "moving to the start location %v", g.startLocation)
	if err := g.moveTo(g.startLocation); err != nil {
		g.searchCh <- errors.Wrap(err, "failed to move back to the starting position after the right edge "+
			"has been found")
		return
	}

	// Send signal to MCU to start looking
	if err := g.gripper.CommandCompleted(); err != nil {
		g.searchCh <- errors.Wrap(err, "failed to send command completed cmd")
		return
	}

	// Move up/down in y (depending on low vs high stack row, resp) until sensor hit
	dir := UP
	if !g.lowLayer.Load() {
		dir = DOWN
	}

	for !g.highLowEdgeFound.Load() {
		g.logger.Logf(msgs.DEBUG, "moving up/down (up = %v), since high/low edge has not yet been found", g.lowLayer.Load())
		if err := g.moveDirection(dir); err != nil && errors.Cause(err) != robot_move.MoveCancelledErr {
			g.searchCh <- errors.Wrap(err, "failed to move up/down to determine high/low edge with IR sensors")
			return
		}
	}
}

func (g *gripper) handleHighLowEdgeFound() {
	g.logger.Logf(msgs.DEBUG, "high low edge found")
	l, err := g.move.GetLocation()
	if err != nil {
		g.searchCh <- errors.Wrapf(err, "unable to get location to calculate final target point")
		return
	}

	A, B, C := gripper_solver.Point{
		X: g.latestHighLowLocation.X,
		Y: g.latestHighLowLocation.Y,
	}, gripper_solver.Point{
		X: g.latestLowerSensorLocation.X,
		Y: g.latestLowerSensorLocation.Y,
	}, gripper_solver.Point{
		X: g.latestUpperSensorLocation.X,
		Y: g.latestUpperSensorLocation.Y,
	}
	g.logger.Logf(msgs.DEBUG, "calculating target point based on these points (A, B, C): %v %v %v", A, B, C)

	// Find the corner location
	corner, R := gripper_solver.FindCorner(A, B, C)

	// Find the middle location
	middle, angle := gripper_solver.FindMiddle(R, corner)

	// Find the middle with offset
	effectorMiddle := gripper_solver.FindOffsetMiddle(R, middle, B, C)

	// Transform into other struct
	l.X = effectorMiddle.X
	l.Y = effectorMiddle.Y
	l.W = angle

	// Log intermediate results
	g.logger.Logf(msgs.DEBUG, "R: %v", R)
	g.logger.Logf(msgs.DEBUG, "corner: %v", corner)
	g.logger.Logf(msgs.DEBUG, "carton middle: %v", middle)
	g.logger.Logf(msgs.DEBUG, "target 2d: %v", effectorMiddle)
	g.logger.Logf(msgs.DEBUG, "target angle: %v", angle)
	g.logger.Logf(msgs.DEBUG, "final target: %v", l)

	// Move to calculated position
	g.logger.Logf(msgs.DEBUG, "moving to calculated target: %v", l)
	if err := g.moveTo(l); err != nil {
		g.searchCh <- errors.Wrap(err, "failed to move to calculated position")
		return
	}

	// Done with search, properly aligned now
	g.searchCh <- nil
}

func (g *gripper) Close() error {
	if err := errors.Wrap(g.gripper.Close(), "failed to close gripper link"); err != nil {
		return err
	}

	close(g.done)
	return nil
}
