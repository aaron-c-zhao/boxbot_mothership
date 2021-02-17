package main

//TODO(timanema): Refactor if there is time
import (
	"boxbot-go/internal/rcs_states"
	"boxbot-go/pkg/carton_search"
	"boxbot-go/pkg/event"
	"boxbot-go/pkg/lang"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/robot_move"
	"fmt"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/looplab/fsm"
	"github.com/pkg/errors"
	"time"
)

//TODO(timanema): Error handling

func (n *autonomous) transition(e string, args ...interface{}) error {
	cur := n.fsm.Current()
	n.logger.Logf(msgs.DEBUG, "transitioning rcs autonomous FSM from %s (%s)", cur, e)

	if err := n.fsm.Event(e, args...); err != nil {
		n.logger.Logf(msgs.DEBUG, "illegal state transition %s (from current state %s): %v", e, n.fsm.Current(), err)
		return err
	}

	n.statePub.Write(&rcs.StateChange{
		State: n.fsm.Current(),
	})
	n.logger.Logf(msgs.DEBUG, "transitioned rcs autonomous FSM: %s -> %s (%s)", cur, n.fsm.Current(), e)
	return nil
}

func (n *autonomous) canTransition(e string, args ...interface{}) {
	go func() {
		_ = n.transition(e, args...)
	}()
}

func (n *autonomous) mustTransition(e string, args ...interface{}) {
	go func() {
		if err := n.transition(e, args...); err != nil {
			_ = n.fsm.Event(rcs_states.GenericErr, lang.InternalError, lang.RestartSystem, err)
			n.statePub.Write(&rcs.StateChange{
				State: rcs_states.GenericErr,
			})
		}
	}()
}

func (n *autonomous) createFSM() {
	n.fsm = fsm.NewFSM(rcs_states.InitState, fsm.Events{
		// Special state, only used to enter RCS initialization
		{Name: rcs_states.StartInit, Src: []string{rcs_states.InitState}, Dst: rcs_states.InitRcsState},

		// First map all normal transitions
		{Name: rcs_states.RcsInit, Src: []string{rcs_states.InitRcsState}, Dst: rcs_states.RcsReady},
		{Name: rcs_states.RcsStart, Src: []string{rcs_states.RcsReady}, Dst: rcs_states.CalibratingState},

		{Name: rcs_states.StartHoming, Src: []string{rcs_states.CalibratingState}, Dst: rcs_states.HomingState},
		{Name: rcs_states.HomingDone, Src: []string{rcs_states.HomingState}, Dst: rcs_states.CalibratingState},
		{Name: rcs_states.MoveOutsideCameraRange, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.MovingOutsideCameraRangeState},

		{Name: rcs_states.MoveOutsideCameraRangeDone, Src: []string{rcs_states.MovingOutsideCameraRangeState},
			Dst: rcs_states.SearchState},
		{Name: rcs_states.NoCartonsFound, Src: []string{rcs_states.SearchState},
			Dst: rcs_states.EmptyState},
		{Name: rcs_states.OperatorAddedCartons, Src: []string{rcs_states.EmptyState},
			Dst: rcs_states.CalibratingState},

		{Name: rcs_states.SearchCartonsDone, Src: []string{rcs_states.SearchState}, Dst: rcs_states.MovingToCartonState},
		{Name: rcs_states.MovedToCarton, Src: []string{rcs_states.MovingToCartonState},
			Dst: rcs_states.GrippingCartonState},

		{Name: rcs_states.CartonGripped, Src: []string{rcs_states.GrippingCartonState},
			Dst: rcs_states.MovingToConveyorState},

		{Name: rcs_states.MoveToConveyorDone, Src: []string{rcs_states.MovingToConveyorState},
			Dst: rcs_states.DroppingAtConveyorState},
		{Name: rcs_states.CartonsDroppedAtConveyor, Src: []string{rcs_states.DroppingAtConveyorState},
			Dst: rcs_states.MovingToCartonState},
		{Name: rcs_states.AllCartonsMoved, Src: []string{rcs_states.MovingToCartonState},
			Dst: rcs_states.EmptyState},

		//TODO(timanema): Add all appropriate source states for these generic transitions
		{Name: rcs_states.StopRcs, Src: []string{rcs_states.CalibratingState, rcs_states.HomingState,
			rcs_states.EmergencyState, rcs_states.PausedState, rcs_states.MovingToConveyorState,
			rcs_states.DroppingAtConveyorState, rcs_states.MovingOutsideCameraRangeState, rcs_states.SearchState,
			rcs_states.MovingToCartonState, rcs_states.GrippingCartonState, rcs_states.EmptyState,
			rcs_states.StuckState, rcs_states.HaltState, rcs_states.RcsReady},
			Dst: rcs_states.StopRcsState},

		{Name: rcs_states.StartEmergency, Src: []string{rcs_states.CalibratingState, rcs_states.HomingState,
			rcs_states.PausedState, rcs_states.MovingToConveyorState,
			rcs_states.DroppingAtConveyorState, rcs_states.MovingOutsideCameraRangeState, rcs_states.SearchState,
			rcs_states.MovingToCartonState, rcs_states.GrippingCartonState, rcs_states.EmptyState,
			rcs_states.StuckState, rcs_states.HaltState, rcs_states.RcsReady},
			Dst: rcs_states.EmergencyState},
		{Name: rcs_states.StopEmergency, Src: []string{rcs_states.EmergencyState},
			Dst: rcs_states.CalibratingState},

		{Name: rcs_states.StartHalt, Src: []string{rcs_states.CalibratingState, rcs_states.HomingState,
			rcs_states.PausedState, rcs_states.MovingToConveyorState,
			rcs_states.DroppingAtConveyorState, rcs_states.MovingOutsideCameraRangeState, rcs_states.SearchState,
			rcs_states.MovingToCartonState, rcs_states.GrippingCartonState, rcs_states.EmptyState,
			rcs_states.StuckState, rcs_states.RcsReady},
			Dst: rcs_states.HaltState},
		{Name: rcs_states.StopHalt, Src: []string{rcs_states.HaltState},
			Dst: rcs_states.CalibratingState},

		{Name: rcs_states.StartPause, Src: []string{rcs_states.CalibratingState, rcs_states.HomingState,
			rcs_states.MovingToConveyorState, rcs_states.DroppingAtConveyorState,
			rcs_states.MovingOutsideCameraRangeState, rcs_states.SearchState, rcs_states.MovingToCartonState,
			rcs_states.GrippingCartonState, rcs_states.EmptyState, rcs_states.RcsReady},
			Dst: rcs_states.PausedState},
		{Name: rcs_states.StopPause, Src: []string{rcs_states.PausedState},
			Dst: rcs_states.CalibratingState},

		// Now map all restoring transitions. These will be used when the calibrating state restores an old state,
		// which was previously overruled
		{Name: rcs_states.StartSearchCartons, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.SearchState},
		{Name: rcs_states.MoveToCarton, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.MovingToCartonState},
		{Name: rcs_states.StartGrippingCarton, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.GrippingCartonState},

		{Name: rcs_states.MoveToConveyor, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.MovingToConveyorState},
		{Name: rcs_states.DropCartonsAtConveyor, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.DroppingAtConveyorState},

		{Name: rcs_states.MoveOutsideCameraRange, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.MovingOutsideCameraRangeState},

		{Name: rcs_states.StartStuck, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.StuckState},

		{Name: rcs_states.RcsWait, Src: []string{rcs_states.CalibratingState},
			Dst: rcs_states.RcsReady},

		// Now map all transitions that lead to the machine being stuck, and needing help from
		// an operator
		{Name: rcs_states.FailedToDropCartons, Src: []string{rcs_states.DroppingAtConveyorState},
			Dst: rcs_states.StuckState},
		{Name: rcs_states.MoveToConveyorFailed, Src: []string{rcs_states.MovingToConveyorState},
			Dst: rcs_states.StuckState},
		{Name: rcs_states.MoveOutsideCameraRangeFailed, Src: []string{rcs_states.MovingOutsideCameraRangeState},
			Dst: rcs_states.StuckState},
		{Name: rcs_states.MoveToCartonFailed, Src: []string{rcs_states.MovingToCartonState},
			Dst: rcs_states.StuckState},
		{Name: rcs_states.GrippingCartonFailed, Src: []string{rcs_states.GrippingCartonState},
			Dst: rcs_states.StuckState},
		{Name: rcs_states.HomingFailed, Src: []string{rcs_states.HomingState},
			Dst: rcs_states.StuckState},

		{Name: rcs_states.OperatorFixedStuck, Src: []string{rcs_states.StuckState},
			Dst: rcs_states.CalibratingState},

		// Now map all exceptional transitions (error transitions)
		{Name: rcs_states.RcsInitFailed, Src: []string{rcs_states.InitRcsState}, Dst: rcs_states.ErrorState},
		{Name: rcs_states.RcsStopFailed, Src: []string{rcs_states.StopRcsState}, Dst: rcs_states.ErrorState},

		{Name: rcs_states.GenericErr, Src: []string{rcs_states.InitRcsState, rcs_states.CalibratingState,
			rcs_states.HomingState, rcs_states.EmergencyState, rcs_states.PausedState, rcs_states.MovingToConveyorState,
			rcs_states.DroppingAtConveyorState, rcs_states.MovingOutsideCameraRangeState, rcs_states.SearchState,
			rcs_states.MovingToCartonState, rcs_states.GrippingCartonState, rcs_states.EmptyState,
			rcs_states.StuckState, rcs_states.HaltState, rcs_states.StopRcsState}, Dst: rcs_states.ErrorState},
	}, fsm.Callbacks{
		rcs_states.InitRcsState:                  n.enterInitRcs,
		rcs_states.RcsReady:                      n.enterRcsReady,
		rcs_states.CalibratingState:              n.enterCalibrating,
		rcs_states.HomingState:                   n.enterHoming,
		rcs_states.EmergencyState:                n.enterEmergency,
		rcs_states.PausedState:                   n.enterPaused,
		rcs_states.MovingToConveyorState:         n.enterMovingToConveyor,
		rcs_states.DroppingAtConveyorState:       n.enterDroppingAtConveyor,
		rcs_states.MovingOutsideCameraRangeState: n.enterMovingOutsideCameraRange,
		rcs_states.SearchState:                   n.enterSearch,
		rcs_states.MovingToCartonState:           n.enterMovingToCarton,
		rcs_states.GrippingCartonState:           n.enterGrippingCarton,
		rcs_states.EmptyState:                    n.enterEmpty,
		rcs_states.StuckState:                    n.enterStuck,
		rcs_states.HaltState:                     n.enterHalt,
		rcs_states.ErrorState:                    n.enterError,
		rcs_states.StopRcsState:                  n.enterStopRcs,

		"leave_" + rcs_states.HomingState:             n.leaveHoming,
		"leave_" + rcs_states.StuckState:              n.leaveStuck,
		"leave_" + rcs_states.EmptyState:              n.leaveEmpty,
		"leave_" + rcs_states.GrippingCartonState:     n.leaveGrippingCarton,
		"leave_" + rcs_states.DroppingAtConveyorState: n.leaveDroppingAtConveyor,
	})

	//TODO(timanema): Remove later
	fmt.Printf("fsm: \n%s\n", fsm.Visualize(n.fsm))
	n.mustTransition(rcs_states.StartInit)
}

func (n *autonomous) enterInitRcs(_ *fsm.Event) {
	// Setup detection for emergency event
	n.emergencyEvent = event.NewEvent(func() error {
		// Emergency button pressed
		return n.transition(rcs_states.StartEmergency)
	}, func() error {
		// Emergency button released
		return n.transition(rcs_states.StopEmergency)
	})

	// Setup detection for halt event
	n.haltEvent = event.NewEvent(func() error {
		// System halted, could be because of watchdog or other recoverable error state in RAS
		return n.transition(rcs_states.StartEmergency)
	}, func() error {
		// System (RAS) recovered from error state
		return n.transition(rcs_states.StopEmergency)
	})

	// Setup detection for pause event
	n.pauseEvent = event.NewEvent(func() error {
		// System (RAS) entered
		return n.transition(rcs_states.StartPause)
	}, func() error {
		// System (RAS) resumed from paused state
		return n.transition(rcs_states.StopPause)
	})

	//TODO(timanema): Wait for all RCS nodes

	n.canTransition(rcs_states.RcsInit)
}

func (n *autonomous) enterCalibrating(_ *fsm.Event) {
	// check the following
	// - homed (6)
	// - emergency (1)
	// - halt (3)
	// - paused (5)
	// - stop started (2)
	// - stuck (4)
	// - prev run state (7):
	//		- dropping at conveyor
	// 		- moving to conveyor
	// 		- moving outside cam range <- fallback
	// 		- searching
	//		- moving to carton
	//		- gripping carton

	// Check if emergency is happening
	if n.emergencyEvent.IsActive() {
		n.canTransition(rcs_states.StartEmergency)
		return
	}

	// Check if the system is stopping
	if n.stopStarted.Load() {
		n.canTransition(rcs_states.StopRcs)
		return
	}

	// Check if the system is halted by RAS
	if n.haltEvent.IsActive() {
		n.canTransition(rcs_states.StartHalt)
		return
	}

	// Check if the system is waiting on operator intervention
	if n.stuck.Load() {
		n.canTransition(rcs_states.StartStuck)
		return
	}

	// Check if the system is paused
	if n.pauseEvent.IsActive() {
		n.canTransition(rcs_states.StartPause)
		return
	}

	if !n.rcsStarted.Load() {
		n.canTransition(rcs_states.RcsWait)
		return
	}

	// Check the system is homed
	if !n.homed.Load() {
		n.canTransition(rcs_states.StartHoming)
		return
	}

	// Finally, check the previous running state, and continue there if applicable
	switch n.currentRunState {
	case rcs_states.MovingToConveyorState:
		n.canTransition(rcs_states.MoveToConveyor)
	case rcs_states.DroppingAtConveyorState:
		n.canTransition(rcs_states.DropCartonsAtConveyor)
	case rcs_states.MovingOutsideCameraRangeState:
		n.canTransition(rcs_states.MoveOutsideCameraRange)
	case rcs_states.MovingToCartonState:
		n.canTransition(rcs_states.MoveToCarton)
	case rcs_states.GrippingCartonState:
		n.canTransition(rcs_states.StartGrippingCarton)
	case rcs_states.SearchState:
		n.canTransition(rcs_states.StartSearchCartons)
	default:
		n.canTransition(rcs_states.MoveOutsideCameraRange)
	}
}

func (n *autonomous) enterRcsReady(_ *fsm.Event) {
	n.logger.Logf(msgs.INFO, "waiting for start signal from RAS")
}

func (n *autonomous) enterHoming(_ *fsm.Event) {
	// start homing
	// done callback -> leave homing

	// Probably superfluous set, but better safe than sorry
	n.homed.Store(false)

	// Start homing
	err := n.move.Home(func(err error) {
		if err == nil {
			n.canTransition(rcs_states.HomingDone)
		} else {
			n.canTransition(rcs_states.HomingFailed)
		}
	})
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) leaveHoming(e *fsm.Event) {
	// if done homing, homed = true

	if e.Event == rcs_states.HomingDone {
		n.homed.Store(true)
	}
}

func (n *autonomous) enterEmergency(_ *fsm.Event) {
	// stop motors
	// homed = false

	n.homed.Store(false)
	if err := n.move.Stop(); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterPaused(_ *fsm.Event) {
	// stop motors

	if err := n.move.Stop(); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterMovingToConveyor(_ *fsm.Event) {
	// update current running state
	// move to position in config
	// done callback -> event MoveToConveyorDone

	n.currentRunState = rcs_states.MovingToConveyorState
	err := n.move.Move(geometry_msgs.Quaternion{
		X: -10,
		Y: -10,
		Z: -10,
		W: -10,
	}, func(err error) {
		// Ignore cancel
		if err == robot_move.MoveCancelledErr {
			return
		}

		if err == nil {
			n.canTransition(rcs_states.MoveToConveyorDone)
		} else {
			n.canTransition(rcs_states.MoveToConveyorFailed)
		}
	})
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterDroppingAtConveyor(_ *fsm.Event) {
	// update current running state
	// open gripper
	// done callback -> move up slightly
	// else -> event FailedToDropCartons
	// done callback 2 -> update current stack
	// done callback 2 -> event CartonsDroppedAtConveyor
	// else -> event FailedToDropCartons

	n.currentRunState = rcs_states.DroppingAtConveyorState

	if err := n.gripper.ToggleGripper(true); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
		return
	}

	err := n.move.MoveDelay(geometry_msgs.Quaternion{
		X: 0,
		Y: 0,
		Z: 1,
		W: 0,
	}, func(err error) {
		// Ignore cancel
		if err == robot_move.MoveCancelledErr {
			return
		}

		if err == nil {
			n.canTransition(rcs_states.CartonsDroppedAtConveyor)
		} else {
			n.canTransition(rcs_states.FailedToDropCartons)
		}
	}, time.Second*2)
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) leaveDroppingAtConveyor(e *fsm.Event) {
	if e.Event == rcs_states.CartonsDroppedAtConveyor {
		//TODO(timanema): Change current stack, or go to empty state if
		// the entire stack has been emptied

		next := n.currentCarton.Load() + 1
		if next >= 6 {
			next = 0
			n.currentLayer.Add(1)
			fmt.Printf("moving to layer %d\n", n.currentLayer.Load())
		}
		n.currentCarton.Store(next)

		if n.currentLayer.Load() >= 2 {
			fmt.Println("all cartons are moved")
			n.canTransition(rcs_states.AllCartonsMoved)
		}
	}
}

func (n *autonomous) enterMovingOutsideCameraRange(_ *fsm.Event) {
	// update current running state
	// move to position in config
	// done callback -> event MoveOutsideCameraRangeDone
	// else -> event MoveOutsideCameraRangeFailed

	n.currentRunState = rcs_states.MovingOutsideCameraRangeState
	err := n.move.Move(
		geometry_msgs.Quaternion{
			X: -5,
			Y: -5,
			Z: -5,
			W: -5,
		}, func(err error) {
			// Ignore cancel
			if err == robot_move.MoveCancelledErr {
				return
			}

			if err == nil {
				n.canTransition(rcs_states.MoveOutsideCameraRangeDone)
			} else {
				n.canTransition(rcs_states.MoveOutsideCameraRangeFailed)
			}
		})
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterSearch(_ *fsm.Event) {
	// update current running state
	// get cartons from opencv
	// if success -> update carton locations and set current stack
	// if success -> event SearchCartonsDone
	// else -> event NoCartonsFound

	n.currentRunState = rcs_states.SearchState
	if err := n.move.Stop(); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
		return
	}

	err := n.search.Search(func(coordinates []carton_search.Coordinate, err error) {
		if err == carton_search.NoCartonsFound {
			n.canTransition(rcs_states.NoCartonsFound)
			return
		}

		if err != nil {
			n.canTransition(rcs_states.GenericErr, err)
			return
		}

		n.cartonLocations = coordinates
		n.currentCarton.Store(0)
		n.canTransition(rcs_states.SearchCartonsDone)
	})
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterMovingToCarton(_ *fsm.Event) {
	// update current running state
	// get current stack location, and move to it
	// done callback -> event MovedToCarton
	// else -> event MoveToCartonFailed

	//TODO(timanema): Guestimate the Z coordinate
	n.currentRunState = rcs_states.MovingToCartonState
	coord := n.cartonLocations[n.currentCarton.Load()]
	n.currentCartonLocation = geometry_msgs.Quaternion{
		X: coord.X,
		Y: coord.Y,
		Z: coord.Z,
		W: coord.Theta,
	}

	fmt.Printf("moving to carton %d\n", n.currentCarton.Load())
	err := n.move.Move(n.currentCartonLocation, func(err error) {
		// Ignore cancel
		if err == robot_move.MoveCancelledErr {
			return
		}

		if err == nil {
			n.canTransition(rcs_states.MovedToCarton)
		} else {
			n.canTransition(rcs_states.MoveToCartonFailed)
		}
	})
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

//TODO(timanema): Priority refactor
func (n *autonomous) enterGrippingCarton(_ *fsm.Event) {
	//TODO(timanema): think about what happens when the gripper is moving down when the state gets interrupted
	// while its moving down. Restoring state will probably start search again, which will destroy the tower

	// update current running state
	// start gripping search program
	// done callback -> start gripping grip program
	// else -> event GrippingCartonFailed
	// done grip callback -> move slightly up
	// done grip callback -> event CartonGripped
	// else -> event GrippingCartonFailed

	n.currentRunState = rcs_states.GrippingCartonState
	go func() {
		if e := n.grip(); e != "" {
			n.canTransition(e)
		}
	}()
}

func (n *autonomous) grip() string {
	//TODO(timanema): Account for low layers
	n.logger.Logf(msgs.DEBUG, "starting carton search")
	err := n.gripper.StartSearch(n.currentCartonLocation, false)
	// If the gripper was stopped, stop execution
	if errors.Cause(err) == GripperStoppedErr {
		n.logger.Logf(msgs.DEBUG, "gripper was stopped during search")
		return ""
	}

	// Check if the search failed
	if err != nil {
		n.logger.Logf(msgs.DEBUG, "gripper search failed: %v", err)
		return rcs_states.GrippingCartonFailed
	}

	n.logger.Logf(msgs.DEBUG, "starting carton grip")
	err = n.gripper.StartGrip()
	// If the gripper was stopped, stop execution
	if errors.Cause(err) == GripperStoppedErr {
		n.logger.Logf(msgs.DEBUG, "gripper was stopped during grip")
		return ""
	}

	// Check if the grip failed
	if err != nil {
		n.logger.Logf(msgs.DEBUG, "gripper grip failed: %v", err)
		return rcs_states.GrippingCartonFailed
	}

	// Move higher with gripped stack
	n.logger.Logf(msgs.DEBUG, "gripped stack, moving higher %v to safely move carton", n.c.GripHeight)
	if err := n.gripper.moveRelative(0, 0, n.c.GripHeight, 0); err != nil {
		// Ignore if preempted somewhere
		if err == robot_move.MoveCancelledErr {
			n.logger.Logf(msgs.DEBUG, "gripper while moving gripped cartons up")
			return ""
		}

		return rcs_states.GrippingCartonFailed
	}

	// Carton is now gripper
	return rcs_states.CartonGripped
}

func (n *autonomous) leaveGrippingCarton(_ *fsm.Event) {
	if err := n.gripper.Stop(); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterEmpty(_ *fsm.Event) {
	// empty = true
	// move to safe position

	n.empty.Store(true)
	err := n.move.Move(geometry_msgs.Quaternion{
		X: -2,
		Y: -2,
		Z: -2,
		W: -2,
	}, func(err error) {
		// Ignore cancel
		if err == robot_move.MoveCancelledErr {
			return
		}

		if err != nil {
			n.canTransition(rcs_states.GenericErr, errors.New("failed to move to safe empty location"))
			return
		}
	})
	if err != nil {
		n.canTransition(rcs_states.GenericErr, err)
		return
	}

	// Check if cartons were added in the mean time
	if n.cartonsAdded.Load() {
		n.canTransition(rcs_states.OperatorAddedCartons)
	}
}

func (n *autonomous) leaveEmpty(e *fsm.Event) {
	// if  OperatorAddedCartons -> empty = false

	if e.Event == rcs_states.OperatorAddedCartons {
		n.empty.Store(false)
		n.currentRunState = rcs_states.InitState
		n.currentCarton.Store(0)
		n.currentLayer.Store(0)
		n.cartonsAdded.Store(false)
	}
}

func (n *autonomous) enterStuck(_ *fsm.Event) {
	// stuck = true
	// stop motors

	n.stuck.Store(true)
	if err := n.move.Stop(); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
		return
	}

	// Check if stuck was fixed in the mean time
	if n.stuckFixed.Load() {
		n.canTransition(rcs_states.OperatorFixedStuck)
	}
}

func (n *autonomous) leaveStuck(e *fsm.Event) {
	// if OperatorFixedStuck -> stuck = false

	if e.Event == rcs_states.OperatorFixedStuck {
		n.stuck.Store(false)
		n.stuckFixed.Store(false)
	}
}

func (n *autonomous) enterHalt(_ *fsm.Event) {
	// stop motors
	// homed = false
	n.homed.Store(false)
	if err := n.move.Stop(); err != nil {
		n.canTransition(rcs_states.GenericErr, err)
	}
}

func (n *autonomous) enterError(_ *fsm.Event) {
	// stop motors
	// homed = false
	n.homed.Store(false)
	if err := n.move.Stop(); err != nil {
		n.logger.Logf(msgs.ERROR, "unable to stop motors in error state")
	}
}

func (n *autonomous) enterStopRcs(_ *fsm.Event) {
	// stop motors
	// stop relevant nodes

	// shutdown computer, if applicable

	if err := n.move.Stop(); err != nil {
		n.logger.Logf(msgs.ERROR, "unable to stop motors while shutting down")
	}
}
