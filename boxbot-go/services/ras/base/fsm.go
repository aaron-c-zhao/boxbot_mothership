package main

import (
	"boxbot-go/internal/ras_states"
	"boxbot-go/pkg/event"
	"boxbot-go/pkg/hardware"
	"boxbot-go/pkg/lang"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/rcs_link"
	"boxbot-go/pkg/ui"
	"errors"
	"fmt"
	"github.com/looplab/fsm"
	"time"
)

func (n *baseNode) transition(e string, args ...interface{}) error {
	cur := n.fsm.Current()
	n.logger.Logf(msgs.DEBUG, "transitioning base FSM from %s (%s)", cur, e)

	if err := n.fsm.Event(e, args...); err != nil {
		n.logger.Logf(msgs.DEBUG, "illegal state transition %s (from current state %s): %v", e, n.fsm.Current(), err)
		return err
	}

	n.logger.Logf(msgs.DEBUG, "transitioned base FSM: %s -> %s (%s)", cur, n.fsm.Current(), e)
	return nil
}

func (n *baseNode) canTransition(e string, args ...interface{}) {
	go func() {
		_ = n.transition(e, args...)
	}()
}

func (n *baseNode) mustTransition(e string, args ...interface{}) {
	go func() {
		if err := n.transition(e, args...); err != nil {
			_ = n.fsm.Event(ras_states.GenericErr, lang.InternalError, lang.RestartSystem, err)
		}
	}()
}

func (n *baseNode) createFSM() {
	n.fsm = fsm.NewFSM(ras_states.InitState, fsm.Events{
		// Special state, only used to enter RAS initialization
		{Name: ras_states.StartInit, Src: []string{ras_states.InitState}, Dst: ras_states.InitRasState},

		// First map all normal transitions
		{Name: ras_states.RasInit, Src: []string{ras_states.InitRasState}, Dst: ras_states.InitRcsState},
		{Name: ras_states.RcsInit, Src: []string{ras_states.InitRcsState}, Dst: ras_states.CalibrateState},

		{Name: ras_states.NoPallet, Src: []string{ras_states.RunningState}, Dst: ras_states.EmptyState},
		{Name: ras_states.EmptyPallet, Src: []string{ras_states.RunningState}, Dst: ras_states.EmptyState},
		{Name: ras_states.PalletPlaced, Src: []string{ras_states.EmptyState}, Dst: ras_states.WaitingState},
		{Name: ras_states.StartButton, Src: []string{ras_states.WaitingState}, Dst: ras_states.RunningState},

		{Name: ras_states.PauseButton, Src: []string{ras_states.RunningState}, Dst: ras_states.PausedState},
		{Name: ras_states.StartButton, Src: []string{ras_states.PausedState, ras_states.StuckState}, Dst: ras_states.CalibrateState},

		{Name: ras_states.RcsStuck, Src: []string{ras_states.RunningState}, Dst: ras_states.StuckState},

		{Name: ras_states.EmergencyFlagHigh, Src: []string{ras_states.InitRasState, ras_states.InitRcsState, ras_states.RunningState,
			ras_states.EmptyState, ras_states.WaitingState, ras_states.PausedState, ras_states.StuckState,
			ras_states.WatchdogState, ras_states.CalibrateState,
			ras_states.StopRcsState}, Dst: ras_states.EmergencyState},
		{Name: ras_states.EmergencyFlagLow, Src: []string{ras_states.EmergencyState}, Dst: ras_states.CalibrateState},

		{Name: ras_states.WatchdogAlertHigh, Src: []string{ras_states.InitRasState, ras_states.InitRcsState, ras_states.RunningState,
			ras_states.EmptyState, ras_states.WaitingState, ras_states.PausedState, ras_states.StuckState,
			ras_states.CalibrateState, ras_states.StopRcsState},
			Dst: ras_states.WatchdogState},
		{Name: ras_states.WatchdogAlertLow, Src: []string{ras_states.WatchdogState}, Dst: ras_states.CalibrateState},

		{Name: ras_states.Calibrated, Src: []string{ras_states.CalibrateState}, Dst: ras_states.RunningState},

		{Name: ras_states.RcsStop, Src: []string{ras_states.StopRcsState}, Dst: ras_states.StopRasState},

		{Name: ras_states.StopSystem, Src: []string{ras_states.RunningState, ras_states.EmptyState, ras_states.WaitingState,
			ras_states.CalibrateState, ras_states.PausedState, ras_states.StuckState, ras_states.EmergencyState,
			ras_states.WatchdogState, ras_states.ErrorState}, Dst: ras_states.StopRcsState},

		// Now map all restoring transitions. These will be used when the calibrating state restores an old state,
		// which was previously overrules.
		{Name: ras_states.RasInit, Src: []string{ras_states.CalibrateState}, Dst: ras_states.InitRcsState},
		{Name: ras_states.PauseButton, Src: []string{ras_states.CalibrateState}, Dst: ras_states.PausedState},
		{Name: ras_states.RcsStuck, Src: []string{ras_states.CalibrateState}, Dst: ras_states.StuckState},
		{Name: ras_states.NoPallet, Src: []string{ras_states.CalibrateState}, Dst: ras_states.EmptyState},

		// Now map all exceptional transitions (error transitions)
		{Name: ras_states.RasInitFailed, Src: []string{ras_states.InitRasState}, Dst: ras_states.ErrorState},
		{Name: ras_states.RcsInitFailed, Src: []string{ras_states.InitRcsState}, Dst: ras_states.ErrorState},

		{Name: ras_states.GenericErr, Src: []string{ras_states.InitRasState, ras_states.InitRcsState, ras_states.RunningState,
			ras_states.EmergencyState, ras_states.WaitingState, ras_states.CalibrateState, ras_states.PausedState,
			ras_states.StuckState, ras_states.EmergencyState, ras_states.WatchdogState, ras_states.StopRcsState},
			Dst: ras_states.ErrorState},

		{Name: ras_states.RcsStopFailed, Src: []string{ras_states.StopRcsState}, Dst: ras_states.ErrorState},
	}, fsm.Callbacks{
		ras_states.InitRasState:   n.enterInitRas,
		ras_states.InitRcsState:   n.enterInitRcs,
		ras_states.RunningState:   n.enterRunning,
		ras_states.EmptyState:     n.enterEmpty,
		ras_states.WaitingState:   n.enterWaiting,
		ras_states.PausedState:    n.enterPaused,
		ras_states.EmergencyState: n.enterEmergency,
		ras_states.WatchdogState:  n.enterWatchdog,
		ras_states.CalibrateState: n.enterCalibrate,
		ras_states.ErrorState:     n.enterError,
		ras_states.StopRcsState:   n.enterStopRcs,
		ras_states.StopRasState:   n.enterStopRas,
		ras_states.StuckState:     n.enterStuck,

		"leave_" + ras_states.WaitingState:   n.leaveWaiting,
		"leave_" + ras_states.PausedState:    n.leavePaused,
		"leave_" + ras_states.EmergencyState: n.leaveEmergency,
		"leave_" + ras_states.WatchdogState:  n.leaveWatchdog,

		"before_" + ras_states.RcsInit: n.afterInitRcs,
	})

	//TODO(timanema): Remove later
	fmt.Printf("fsm: \n%s\n", fsm.Visualize(n.fsm))
	n.mustTransition(ras_states.StartInit)
}

func (n *baseNode) enterInitRas(_ *fsm.Event) {
	n.mustSetAlarmLight(ui.Yellow, ui.Solid)
	n.mustSetDisplay(false, lang.StartingMessage, "")

	//TODO(timanema): Check for edge cases (starting with emergency button pressed for example)

	// Setup detection for emergency button
	n.emergencyEvent = event.NewEvent(func() error {
		// Emergency button pressed
		return n.transition(ras_states.EmergencyFlagHigh)
	}, func() error {
		// Emergency button released
		return n.transition(ras_states.EmergencyFlagLow)
	})

	// Setup detection for watchdog
	n.watchdogEvent = event.NewEvent(func() error {
		// Active
		n.canTransition(ras_states.WatchdogAlertHigh)
		return nil
	}, func() error {
		// Inactive
		n.canTransition(ras_states.WatchdogAlertLow)
		return nil
	})

	// Setup detection for RCS stuck
	n.stuckEvent = event.NewEvent(func() error {
		n.canTransition(ras_states.RcsStuck)
		return nil
	}, func() error {
		return nil
	})

	// Setup detection for RCS empty
	n.emptyEvent = event.NewEvent(func() error {
		n.canTransition(ras_states.NoPallet)
		return nil
	}, func() error {
		return nil
	})

	// RCS should be starting in parallel, this will just be checking its state to see if it's done
	// If n.rcs is not nil, the node is already subscribed to rcs events, so only try to init once.
	if n.rcs == nil {
		var err error
		if n.rcs, err = rcs_link.NewRCS(n.n, n.logger); err != nil {
			n.logger.Logf(msgs.ERROR, "unable to connect to RCS: %v", err)

			n.mustTransition(ras_states.RcsInitFailed, lang.RCSCreationFailed, lang.MaintenanceRequired, err)
			return
		}
	}

	// Wait for all RAS nodes to be active, then continue with initialized RCS
	go func() {
		n.logger.Logf(msgs.INFO, "waiting for other RAS nodes")
		if err := node.WaitForNodes(n.n, n.close, node.RasNodes); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to wait for all RAS nodes to come online: %v", err)
			n.mustTransition(ras_states.RasInitFailed, lang.RASInitTimeout, lang.MaintenanceRequired, err)
			return
		}

		// Transition to InitRCSState
		if err := n.fsm.Event(ras_states.RasInit); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to transition to RCS init state: %v", err)
			n.mustTransition(ras_states.RasInitFailed, lang.RASInitFail, lang.MaintenanceRequired, err)
		}
	}()
}

func (n *baseNode) enterInitRcs(_ *fsm.Event) {
	// Enable power to the motors for RCS to start
	if err := n.hw.SetRelay(hardware.MotorRelay, true); err != nil {
		n.logger.Logf(msgs.ERROR, "setting motor relay to true failed when entering RCS init state: %v", err)
		n.mustTransition(ras_states.RcsInitFailed, lang.RCSInitMotorRelayFail, lang.MaintenanceRequired, err)
		return
	}
	if err := n.hw.SetRelay(hardware.ServoRelay, true); err != nil {
		n.logger.Logf(msgs.ERROR, "setting servo relay to true failed when entering RCS init state: %v", err)
		n.mustTransition(ras_states.RcsInitFailed, lang.RCSInitServoRelayFail, lang.MaintenanceRequired, err)
		return
	}

	n.logger.Logf(msgs.INFO, "waiting for RCS nodes")
	//TODO(timanema): Actually wait for RCS nodes
	time.Sleep(time.Second * 3)

	n.logger.Logf(msgs.INFO, "connecting with RCS")

	// Set callbacks
	n.rcs.SetStateCallbacks(n.stuckEvent.Active, n.emptyEvent.Active, func() error {
		n.mustTransition(ras_states.GenericErr, lang.RCSGenericError, lang.MaintenanceRequired,
			errors.New("generic rcs error"))
		return nil
	})

	go func() {
		// Wait for rcs to start up
		if err := n.rcs.WaitForInit(time.Second * time.Duration(n.c.RCSInitTimeoutSeconds)); err != nil {
			n.logger.Logf(msgs.ERROR, "unable to init RCS: %v", err)
			n.canTransition(ras_states.RcsInitFailed, lang.RCSInitFailed, lang.MaintenanceRequired, err)
			return
		}

		n.logger.Logf(msgs.DEBUG, "done with waiting for RCS init")

		// When it has started, just allow it to work
		if err := n.rcs.Start(); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to start RCS: %v", err)
			n.canTransition(ras_states.RcsInitFailed, lang.RCSInitFailed, lang.MaintenanceRequired, err)
			return
		}

		n.canTransition(ras_states.RcsInit)
	}()

	// Change alert light
	n.mustSetAlarmLight(ui.Yellow, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(false, lang.StartingMessage, "")
}

func (n *baseNode) afterInitRcs(_ *fsm.Event) {
	n.startupCompleted.Store(true)

	n.logger.Logf(msgs.INFO, "starting monitoring systems")

	if err := n.wd.EnableWatchdog(); err != nil {
		n.logger.Logf(msgs.ERROR, "failed to enable watchdog after RCS init: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.WatchdogInitFailed, lang.MaintenanceRequired, err)
		return
	}
}

func (n *baseNode) enterRunning(_ *fsm.Event) {
	// Change alert light
	n.mustSetAlarmLight(ui.Green, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(false, lang.RunningMessage, "")

	n.mustSetButtonEffect(ui.StartButton, ui.On)
	n.mustSetButtonEffect(ui.PauseButton, ui.On)
	n.mustSetButtonEffect(ui.PowerButton, ui.On)
}

func (n *baseNode) enterEmpty(_ *fsm.Event) {
	// Notify operator
	n.mustSendNotification(lang.EmptyNotification)

	// Change alert light
	n.mustSetAlarmLight(ui.Orange, ui.Flashing)

	// Change screen hint
	n.mustSetDisplay(true, lang.EmptyMessage, lang.EmptyHint)

	//TODO(timanema): Do we want actual limit switches here?
	go func() {
		time.Sleep(time.Second * 5)
		n.canTransition(ras_states.PalletPlaced)
	}()
}

func (n *baseNode) enterWaiting(_ *fsm.Event) {
	// Start 'start' button hint
	n.mustSetButtonEffect(ui.StartButton, ui.HintBlink)

	// Change alert light
	n.mustSetAlarmLight(ui.Orange, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(true, lang.WaitingMessage, lang.WaitingHint)
}

func (n *baseNode) leaveWaiting(e *fsm.Event) {
	// Stop 'start' button hint
	n.mustSetButtonEffect(ui.StartButton, ui.On)

	if e.Event == ras_states.StartButton {
		_ = n.emptyEvent.Inactive()

		// Attempt to restart rcs from empty
		if err := n.rcs.StackPlaced(); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to resume RCS after empty: %v", err)
			n.mustTransition(ras_states.GenericErr, lang.RCSResumeEmptyFailure, lang.RestartSystem, err)
			return
		}
	}
}

func (n *baseNode) enterPaused(_ *fsm.Event) {
	n.paused.Store(true)

	// Pause rcs
	if err := n.rcs.SetPaused(true); err != nil {
		n.logger.Logf(msgs.ERROR, "failed to pause RCS when pausing: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.RCSPauseFailure, lang.RestartSystem, err)
		return
	}

	// Start 'start' button hint
	n.mustSetButtonEffect(ui.StartButton, ui.HintBlink)

	// Change alert light
	n.mustSetAlarmLight(ui.Orange, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(true, lang.PausedMessage, "")
}

func (n *baseNode) leavePaused(e *fsm.Event) {
	// Stop 'start' button hint
	n.mustSetButtonEffect(ui.StartButton, ui.On)

	// Update state if normal leave
	if e.Event == ras_states.StartButton {
		// Start rcs
		if err := n.rcs.SetPaused(false); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to resume RCS after pause: %v", err)
			n.mustTransition(ras_states.GenericErr, lang.RCSResumeFailure, lang.RestartSystem, err)
			return
		}

		n.paused.Store(false)
	}
}

func (n *baseNode) enterStuck(_ *fsm.Event) {
	// Notify operator
	n.mustSendNotification(lang.StuckNotification)

	// Start 'start' button hint
	n.mustSetButtonEffect(ui.StartButton, ui.HintBlink)

	// Change alert light
	n.mustSetAlarmLight(ui.Red, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(true, lang.StuckMessage, lang.StuckHint)
}

func (n *baseNode) leaveStuck(e *fsm.Event) {
	// Stop 'start' button hint
	n.mustSetButtonEffect(ui.StartButton, ui.On)

	// Update state if normal leave
	if e.Event == ras_states.StartButton {
		_ = n.stuckEvent.Inactive()

		// Attempt to restart rcs from stuck
		if err := n.rcs.OperatorUnstuck(); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to resume RCS after stuck: %v", err)
			n.mustTransition(ras_states.GenericErr, lang.RCSResumeStuckFailure, lang.RestartSystem, err)
			return
		}
	}
}

func (n *baseNode) enterEmergency(_ *fsm.Event) {
	n.logger.Logf(msgs.WARN, "system entering emergency mode")

	// Notify operator
	n.mustSendNotification(lang.EmergencyNotification)

	// Change alert light
	n.mustSetAlarmLight(ui.Red, ui.Flashing)

	// Change screen hint
	n.mustSetDisplay(true, lang.EmergencyMessage, "")

	// Set system to a paused state, so it won't immediately start after the emergency button is released
	n.paused.Store(true)

	// Disable power to the motors during the emergency
	if err := n.hw.SetRelay(hardware.MotorRelay, false); err != nil {
		n.logger.Logf(msgs.ERROR, "setting motor relay to false failed when entering emergency state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.EmergencyMotorDisableRelayFail, lang.MaintenanceRequired, err)
		return
	}
	if err := n.hw.SetRelay(hardware.ServoRelay, false); err != nil {
		n.logger.Logf(msgs.ERROR, "setting servo relay to false failed when entering emergency state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.EmergencyServoDisableRelayFail, lang.MaintenanceRequired, err)
		return
	}

	// Stop rcs
	if err := n.rcs.SetEmergency(true); err != nil {
		n.logger.Logf(msgs.ERROR, "failed to stop RCS after emergency: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.RCSStartEmergencyFailure, lang.RestartSystem, err)
		return
	}
}

func (n *baseNode) leaveEmergency(_ *fsm.Event) {
	n.logger.Logf(msgs.INFO, "system leaving emergency mode")

	// Enable power to the motors after the emergency
	if err := n.hw.SetRelay(hardware.MotorRelay, true); err != nil {
		n.logger.Logf(msgs.ERROR, "setting motor relay to true failed when leaving emergency state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.EmergencyMotorEnableRelayFail, lang.MaintenanceRequired, err)
		return
	}
	if err := n.hw.SetRelay(hardware.ServoRelay, true); err != nil {
		n.logger.Logf(msgs.ERROR, "setting servo relay to true failed when leaving emergency state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.EmergencyServoEnableRelayFail, lang.MaintenanceRequired, err)
		return
	}

	// Start rcs
	if err := n.rcs.SetEmergency(false); err != nil {
		n.logger.Logf(msgs.ERROR, "failed to resume RCS after emergency: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.RCSStopEmergencyFailure, lang.RestartSystem, err)
		return
	}
}

func (n *baseNode) enterWatchdog(_ *fsm.Event) {
	n.logger.Logf(msgs.WARN, "system halted by watchdog")

	// Stop rcs
	if err := n.rcs.SetHalted(true); err != nil {
		n.logger.Logf(msgs.ERROR, "failed to stop RCS after halt: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.RCSStartHaltFailure, lang.RestartSystem, err)
		return
	}

	// Notify operator
	n.mustSendNotification(lang.WatchdogNotification)

	// Change alert light
	n.mustSetAlarmLight(ui.Red, ui.Flashing)

	// Change screen hint
	n.mustSetDisplay(true, lang.WatchdogMessage, lang.RestartSystem)

	// Set system to a paused state, so it won't immediately start after the watchdog alert is fixed
	n.paused.Store(true)
}

func (n *baseNode) leaveWatchdog(_ *fsm.Event) {
	n.logger.Logf(msgs.INFO, "system resumed by watchdog")

	// Start rcs
	if err := n.rcs.SetHalted(false); err != nil {
		n.logger.Logf(msgs.ERROR, "failed to resume RCS after halt: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.RCSStopHaltFailure, lang.RestartSystem, err)
		return
	}
}

func (n *baseNode) enterCalibrate(_ *fsm.Event) {
	//TODO(timanema): check which transitions can happen from this state
	// 1: check if emergency is happening
	// 2: check if watchdog is active
	// 3: check if rcs was initialized
	// 4: check if paused
	// 5: check if stopping

	// Change alert light
	n.mustSetAlarmLight(ui.Orange, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(false, lang.CalibratingMessage, "")

	// Check for active states
	if n.emergencyEvent.IsActive() {
		n.canTransition(ras_states.EmergencyFlagHigh)
		return
	}

	if n.watchdogEvent.IsActive() {
		n.canTransition(ras_states.WatchdogAlertHigh)
		return
	}

	if !n.startupCompleted.Load() {
		n.canTransition(ras_states.RasInit)
		return
	}

	if n.stuckEvent.IsActive() {
		n.canTransition(ras_states.RcsStuck)
		return
	}

	if n.emptyEvent.IsActive() {
		n.canTransition(ras_states.NoPallet)
		return
	}

	if n.paused.Load() {
		n.canTransition(ras_states.PauseButton)
		return
	}

	if n.stopStarted.Load() {
		n.canTransition(ras_states.StopSystem)
		return
	}

	// Check for RCS

	// Finally, everything is OK, transition to running
	n.canTransition(ras_states.Calibrated)
}

func (n *baseNode) enterError(e *fsm.Event) {
	n.logger.Logf(msgs.ERROR, "system entering fault state")

	// Stop rcs
	if n.rcs != nil {
		// Stop rcs
		if err := n.rcs.SetHalted(true); err != nil {
			n.logger.Logf(msgs.ERROR, "failed to stop RCS after error: %v", err)
		}
	}

	if len(e.Args) < 3 {
		n.logger.Logf(msgs.ERROR, "system entered fault state, but due to an internal bug not enough information was provided for this message")
		n.logger.Logf(msgs.ERROR, "please check previous logging for information. This is everything sent to the handler: %v", e.Args)
	} else {
		n.logger.Logf(msgs.ERROR, "error (\"%s\") occurred with underlying reason: %v", e.Args[0], e.Args[2])
	}

	// Notify operator
	n.mustSendNotification(lang.ErrorNotification)

	// Change alert light
	n.mustSetAlarmLight(ui.Red, ui.Flashing)

	// Change screen hint
	if len(e.Args) < 2 {
		n.mustSetDisplay(true, "Internal error, check logging", "")
	} else {
		n.mustSetDisplay(true, fmt.Sprintf("%v", e.Args[0]), fmt.Sprintf("%v", e.Args[1]))
	}

	// Disable power to the motors when the system enters a fault state
	if err := n.hw.SetRelay(hardware.MotorRelay, false); err != nil {
		n.logger.Logf(msgs.ERROR, "setting motor relay to false failed when entering err state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.ErrMotorDisableRelayFail, lang.MaintenanceRequired, err)
		return
	}
	if err := n.hw.SetRelay(hardware.ServoRelay, false); err != nil {
		n.logger.Logf(msgs.ERROR, "setting servo relay to false failed when entering err state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.ErrServoDisableRelayFail, lang.MaintenanceRequired, err)
		return
	}
}

func (n *baseNode) enterStopRcs(_ *fsm.Event) {
	n.logger.Logf(msgs.INFO, "system stopping")
	n.stopStarted.Store(true)

	// Disable watchdog, as nodes will go down soon
	n.logger.Logf(msgs.INFO, "stopping monitoring")
	if err := n.wd.DisableWatchdog(); err != nil {
		n.mustTransition(ras_states.RcsStopFailed, lang.WatchdogStopFailed, lang.MaintenanceRequired, err)
		return
	}

	n.logger.Logf(msgs.INFO, "stopping RCS")

	// Change alert light
	n.mustSetAlarmLight(ui.Orange, ui.Solid)

	// Change screen hint
	n.mustSetDisplay(false, lang.StoppingMessage, "")

	// Start 'power' button hint, and disable the others
	n.mustSetButtonEffect(ui.PowerButton, ui.HintBlink)
	n.mustSetButtonEffect(ui.StartButton, ui.Off)
	n.mustSetButtonEffect(ui.PauseButton, ui.Off)

	// Stop rcs
	go func() {
		if n.rcs != nil {
			if err := n.rcs.Shutdown(); err != nil {
				//TODO(timanema): Handle this
				return
			}
		}

		n.mustTransition(ras_states.RcsStop)
	}()

	// Disable power to the motors when the system is stopping
	if err := n.hw.SetRelay(hardware.MotorRelay, false); err != nil {
		n.logger.Logf(msgs.ERROR, "setting motor relay to false failed when entering stop state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.StopMotorDisableRelayFail, lang.MaintenanceRequired, err)
		return
	}
	if err := n.hw.SetRelay(hardware.ServoRelay, false); err != nil {
		n.logger.Logf(msgs.ERROR, "setting servo relay to false failed when entering stop state: %v", err)
		n.mustTransition(ras_states.GenericErr, lang.StopServoDisableRelayFail, lang.MaintenanceRequired, err)
		return
	}
}

func (n *baseNode) enterStopRas(_ *fsm.Event) {
	n.logger.Logf(msgs.INFO, "stopping RAS")

	// Stop relevant RAS nodes
	if err := node.KillNodes(n.n, node.RasKillNodes); err != nil {
		n.logger.Logf(msgs.ERROR, "error occurred while stopping RAS nodes: %v. Ignoring and continue shutdown", err)
	}

	// Finally shutdown the computer
	if n.c.EnableShutdown {
		n.logger.Logf(msgs.INFO, "shutting down host")
		//TODO(timanema): Actually shutdown the computer
	} else {
		n.logger.Logf(msgs.INFO, "shutting down host disabled, killing all nodes instead")
		_ = n.n.KillNode(node.WatchdogName)
		_ = n.n.KillNode(node.RasBaseName)
	}
}
