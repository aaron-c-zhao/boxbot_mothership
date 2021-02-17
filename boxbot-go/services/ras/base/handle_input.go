package main

import (
	"boxbot-go/internal/ras_states"
	"boxbot-go/pkg/hardware"
	"boxbot-go/pkg/lang"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/ui"
	"github.com/pkg/errors"
)

func (n *baseNode) handleButtonEvent(btn ui.Button) error {
	transition, err := transitionForButton(btn)
	if err != nil {
		return errors.Wrap(err, "unable to convert button to transition")
	}

	if err := n.fsm.Event(transition); err != nil {
		return errors.Wrap(err, "unable to transition fsm with the given button")
	}

	return nil
}

func (n *baseNode) handlePowerFailure(p hardware.PowerCheck, failed bool) {
	n.logger.LogLocalf(msgs.DEBUG, "handling power failure of type %v for failure=%v", p, failed)

	// If the current failed check matches a relay that is currently inactive, ignore this call as the check
	// will fail by disabling a relay, which is not an error condition
	if p != hardware.InputCheck && !n.hw.IsActive(p.MatchingRelay()) {
		// TODO(timanema): This behaviour might result in problems when a check actually fails during a period
		// of downtime for a relay, since the interface only calls the service on changes. Should add a check for this
		// somewhere, not sure if already in the prototype or just in document.
		// Can be solved by repeatedly calling the service (ugly), or calling the service on changes AND when a relay
		// becomes active (better).
		n.logger.LogLocalf(msgs.DEBUG, "ignoring power failure for type %v, as the matching relay is "+
			"currently disabled", p)
		return
	}

	// Only handle failed inputs for now, as we will not support restoring conditions in this version of the prototype
	if failed {
		switch p {
		case hardware.InputCheck:
			n.mustTransition(ras_states.GenericErr, lang.PowerCheckInputFailed, lang.MaintenanceRequired, "power failure")
		case hardware.Motor1Check:
			n.mustTransition(ras_states.GenericErr, lang.PowerCheckMotor1Failed, lang.MaintenanceRequired, "power failure")
		case hardware.Motor2Check:
			n.mustTransition(ras_states.GenericErr, lang.PowerCheckMotor2Failed, lang.MaintenanceRequired, "power failure")
		case hardware.Motor3Check:
			n.mustTransition(ras_states.GenericErr, lang.PowerCheckMotor3Failed, lang.MaintenanceRequired, "power failure")
		case hardware.ServoCheck:
			n.mustTransition(ras_states.GenericErr, lang.PowerCheckServoFailed, lang.MaintenanceRequired, "power failure")
		}
	}
}

func (n *baseNode) handleEmergencyFlag(emergency bool) error {
	n.logger.LogLocalf(msgs.DEBUG, "handling emergency flag set to %v", emergency)

	if emergency {
		return n.emergencyEvent.Active()
	} else {
		return n.emergencyEvent.Inactive()
	}
}

func (n *baseNode) handleWatchdog(failed bool) {
	n.logger.LogLocalf(msgs.DEBUG, "handling watchdog alert (high: %v)", failed)

	if failed {
		_ = n.watchdogEvent.Active()
	} else {
		_ = n.watchdogEvent.Inactive()
	}
}

func transitionForButton(btn ui.Button) (ras_states.RasTransition, error) {
	switch btn {
	case ui.PowerButton:
		return ras_states.StopSystem, nil
	case ui.StartButton:
		return ras_states.StartButton, nil
	case ui.PauseButton:
		return ras_states.PauseButton, nil
	default:
		return "", ui.InvalidButtonEffectErr
	}
}
