package main

import (
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/pins"
	"boxbot-go/pkg/srvs/srv_hardware"
	"github.com/pkg/errors"
	"github.com/stianeikeland/go-rpio/v4"
	"os"
	"syscall"
	"time"
)

func (n *interfaceNode) startHardware() error {
	n.buttons = pins.NewPinHandler()

	if err := n.buttons.PrepareInterface(); err != nil {
		if err, ok := errors.Cause(err).(*os.PathError); ok && err.Err == syscall.ENOENT {
			//HACK(timanema): goroslib retrieves the subs async, so this will be executed before the logging service
			// had to change to subscribe. Very very ugly, but a quick fix. Need to think of something better
			time.Sleep(time.Second)

			n.logger.Logf(msgs.ERROR, "unable to start button handlers, starting interface without IO capability: %v", err)
			return nil
		}

		return errors.Wrap(err, "failed to prepare button handlers")
	}

	// Interface buttons
	go n.buttons.DetectEdge(pins.PowerButton, rpio.FallEdge, func() {
		n.handleButtonInput(ras.PowerButton)
	}, pins.PowerButtonLed)
	go n.buttons.DetectEdge(pins.StartButton, rpio.FallEdge, func() {
		n.handleButtonInput(ras.StartButton)
	}, pins.StartButtonLed)
	go n.buttons.DetectEdge(pins.PauseButton, rpio.FallEdge, func() {
		n.handleButtonInput(ras.PauseButton)
	}, pins.PauseButtonLed)

	// Power checks
	go n.buttons.DetectEdge(pins.InputVerifyIn, rpio.AnyEdge, func() {
		n.handlePowerCheck(srv_hardware.CheckInput, n.buttons.GetPin(pins.InputVerifyIn) == rpio.Low)
	})
	go n.buttons.DetectEdge(pins.Motor1VerifyIn, rpio.AnyEdge, func() {
		n.handlePowerCheck(srv_hardware.CheckMotor1, n.buttons.GetPin(pins.Motor1VerifyIn) == rpio.Low)
	})
	go n.buttons.DetectEdge(pins.Motor2VerifyIn, rpio.AnyEdge, func() {
		n.handlePowerCheck(srv_hardware.CheckMotor2, n.buttons.GetPin(pins.Motor2VerifyIn) == rpio.Low)
	})
	go n.buttons.DetectEdge(pins.Motor3VerifyIn, rpio.AnyEdge, func() {
		n.handlePowerCheck(srv_hardware.CheckMotor3, n.buttons.GetPin(pins.Motor3VerifyIn) == rpio.Low)
	})
	go n.buttons.DetectEdge(pins.ServoVerifyIn, rpio.AnyEdge, func() {
		n.handlePowerCheck(srv_hardware.CheckServo, n.buttons.GetPin(pins.ServoVerifyIn) == rpio.Low)
	})

	// Emergency check
	go n.buttons.DetectEdge(pins.EmergencyFlag, rpio.AnyEdge, func() {
		n.handleEmergency(n.buttons.GetPin(pins.EmergencyFlag) == rpio.Low)
	})

	return nil
}

func (n *interfaceNode) handleButtonInput(btn ras.ButtonType) {
	n.logger.Logf(msgs.DEBUG, "publishing button input for %s", btn)
	n.btnPub.Write(&ras.ButtonEvent{
		Type: btn,
	})
}

// TODO(timanema): Figure out why service call sometimes results in EOF
func (n *interfaceNode) handlePowerCheck(check srv_hardware.PowerCheck, failed bool) {
	n.logger.Logf(msgs.DEBUG, "handling power check failure for %s (failed: %v)", check, failed)

	err := n.checkClient.Call(&srv_hardware.PowerCheckReq{
		Check:  check,
		Failed: failed,
	}, &srv_hardware.PowerCheckRes{})

	if err != nil {
		n.logger.Logf(msgs.ERROR, "unable to call power check service for %s (failed: %v): %v", check, failed, err)
	}
}

func (n *interfaceNode) handleEmergency(emergency bool) {
	n.logger.Logf(msgs.DEBUG, "handling emergency to %v", emergency)

	res := &srv_hardware.EmergencyRes{}
	err := n.emergencyClient.Call(&srv_hardware.EmergencyReq{
		Emergency: emergency,
	}, res)

	if err != nil {
		n.logger.Logf(msgs.ERROR, "unable to call emergency service to %v: %v", emergency, err)
	}

	if !res.Success {
		n.logger.Logf(msgs.ERROR, "emergency service reported failure, retrying in %v seconds: %v", n.c.EmergencyRetryDelaySeconds, res.Message)

		go func() {
			time.Sleep(time.Second * time.Duration(n.c.EmergencyRetryDelaySeconds))

			// Check if emergency is no longer applicable
			if (emergency && n.buttons.GetPin(pins.EmergencyFlag) == rpio.High) ||
				(!emergency && n.buttons.GetPin(pins.EmergencyFlag) == rpio.Low) {
				return
			}

			n.handleEmergency(emergency)
		}()
	}
}
