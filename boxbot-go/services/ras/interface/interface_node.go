package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/pins"
	"boxbot-go/pkg/srvs/srv_hardware"
	"boxbot-go/pkg/ws"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"github.com/stianeikeland/go-rpio/v4"
	"os"
	"sync"
	"time"
)

type interfaceNode struct {
	n                                          *goroslib.Node
	buttonInputSrv, interfaceSetSrv, barSetSrv *goroslib.ServiceProvider
	relaySetSrv                                *goroslib.ServiceProvider
	watchdogResetSrv                           *goroslib.ServiceProvider
	btnPub                                     *goroslib.Publisher
	checkClient, emergencyClient               *goroslib.ServiceClient

	logger log.Logger

	c Config

	close, done chan struct{}
	closeLock   sync.Once

	buttons *pins.PinHandler
	websock *ws.WebSocketServer
}

//TODO(timanema): Add connection checking to RAS
//TODO(timanema): Add error handling to PinHandler
func NewNode(c node.Config) (_ node.Node, err error) {
	n := &interfaceNode{
		close: make(chan struct{}, 1),
		done:  make(chan struct{}),
	}

	// Read config
	n.c, err = ReadConfig()
	if err != nil {
		return nil, errors.Wrap(err, "unable to read config")
	}

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.InterfaceName,
		Namespace:     node.RasNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.InterfaceName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create services
	n.buttonInputSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_hardware.ButtonEffectServiceName,
		Srv:      &srv_hardware.ButtonEffectService{},
		Callback: n.setButtonEffect,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.ButtonEffectServiceName)
	}
	n.interfaceSetSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_hardware.InterfaceSetServiceName,
		Srv:      &srv_hardware.InterfaceSetService{},
		Callback: n.setInterface,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.InterfaceSetServiceName)
	}
	n.barSetSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_hardware.BarSetServiceName,
		Srv:      &srv_hardware.BarSetService{},
		Callback: n.setBar,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.BarSetServiceName)
	}
	n.relaySetSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_hardware.RelaySetServiceName,
		Srv:      &srv_hardware.RelaySetService{},
		Callback: n.setRelay,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.RelaySetServiceName)
	}
	n.watchdogResetSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_hardware.PulseWatchdogPinServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.resetWatchdog,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.PulseWatchdogPinServiceName)
	}

	// Create publishers
	n.btnPub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n.n,
		Topic: ras.ButtonEventTopic.Topic,
		Msg:   ras.ButtonEventTopic.Msg,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s publisher", ras.ButtonEventTopic.Topic)
	}

	// Create clients
	n.checkClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n.n,
		Name:            srv_hardware.PowerCheckServiceName,
		Srv:             &srv_hardware.PowerCheckService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.PowerCheckServiceName)
	}
	n.emergencyClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n.n,
		Name:            srv_hardware.EmergencyServiceName,
		Srv:             &srv_hardware.EmergencyService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_hardware.EmergencyServiceName)
	}

	// Create websocket server
	n.websock = ws.New(n.logger, &ws.DisplayState{
		Message: "System is starting",
	})
	go func() {
		if err := n.websock.ListenAndServe(fmt.Sprintf("%s:%v", n.c.WebsocketHost, n.c.WebsocketPort)); err != nil {
			_ = n.Close()
			n.logger.Logf(msgs.ERROR, "unable to start websocket server")

			// TODO(timanema): Maybe this is a bit too rough
			os.Exit(1)
		}
	}()

	go node.PingSelf(n.n, n.close, n.done, node.InterfaceName)
	go n.runBar()

	if err := n.startHardware(); err != nil {
		return nil, err
	}

	return n, err
}

func (n *interfaceNode) runBar() {
	//TODO(timanema): Do something with bar
}

func (n *interfaceNode) resetWatchdog(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	go n.buttons.PulsePin(pins.WatchdogResetPin, true, pins.WatchdogPulseDelay)

	return &std_srvs.TriggerRes{
		Success: true,
	}
}

func (n *interfaceNode) setButtonEffect(req *srv_hardware.ButtonEffectReq) *srv_hardware.ButtonEffectRes {
	n.logger.Logf(msgs.DEBUG, "handling button effect '%v' for button %v", req.ButtonEffect, req.Button)

	var pin rpio.Pin

	switch req.Button {
	case ras.PowerButton:
		pin = pins.PowerButtonLed
	case ras.StartButton:
		pin = pins.StartButtonLed
	case ras.PauseButton:
		pin = pins.PauseButtonLed
	default:
		return &srv_hardware.ButtonEffectRes{
			Success: false,
			Error:   fmt.Sprintf("unknown button type %v", req.Button),
		}
	}

	switch req.ButtonEffect {
	case srv_hardware.EffectHint:
		go n.buttons.BlinkPinIndeterminate(pin, pins.HintInterval)
	case srv_hardware.EffectErr:
		go n.buttons.BlinkPin(pin, pins.ErrInterval, pins.ErrDuration)
	case srv_hardware.EffectOff:
		n.buttons.SetPin(pin, rpio.Low)
	case srv_hardware.EffectOn:
		n.buttons.SetPin(pin, rpio.High)
	default:
		return &srv_hardware.ButtonEffectRes{
			Success: false,
			Error:   fmt.Sprintf("unknown button effect %v", req.ButtonEffect),
		}
	}

	return &srv_hardware.ButtonEffectRes{
		Success: true,
	}
}

func (n *interfaceNode) setInterface(req *srv_hardware.InterfaceSetReq) *srv_hardware.InterfaceSetRes {
	n.logger.Logf(msgs.DEBUG, "handling set interface to '%s'", req.Message)

	n.websock.Publish(&ws.DisplayState{
		Error:   req.Error,
		Message: req.Message,
		Hint:    req.Hint,
	})
	return &srv_hardware.InterfaceSetRes{
		Success: true,
	}
}

func (n *interfaceNode) setBar(req *srv_hardware.BarSetReq) *srv_hardware.BarSetRes {
	n.logger.Logf(msgs.DEBUG, "handling set bar to color '%v' and effect '%v'", req.Color, req.Effect)

	// TODO(timanema): Set bar effects
	return &srv_hardware.BarSetRes{
		Success: true,
	}
}

func (n *interfaceNode) setRelay(req *srv_hardware.RelaySetReq) *srv_hardware.RelaySetRes {
	n.logger.Logf(msgs.DEBUG, "handling set relay: %v to %v", req.Relay, req.Enabled)

	var pin rpio.Pin
	switch req.Relay {
	case srv_hardware.RelayMotor:
		pin = pins.MotorRelay
	case srv_hardware.RelayServo:
		pin = pins.ServoRelay
	default:
		return &srv_hardware.RelaySetRes{
			Success: false,
			Error:   fmt.Sprintf("invalid relay %s", req.Relay),
		}
	}

	if req.Enabled {
		n.buttons.SetPin(pin, rpio.High)

		// Force a power check update after the given amount of time
		go func() {
			time.Sleep(time.Second * time.Duration(n.c.RelayGracePeriodSeconds))

			n.handlePowerCheck(srv_hardware.CheckInput, n.buttons.GetPin(pins.InputVerifyIn) == rpio.Low)
			n.handlePowerCheck(srv_hardware.CheckMotor1, n.buttons.GetPin(pins.Motor1VerifyIn) == rpio.Low)
			n.handlePowerCheck(srv_hardware.CheckMotor2, n.buttons.GetPin(pins.Motor2VerifyIn) == rpio.Low)
			n.handlePowerCheck(srv_hardware.CheckMotor3, n.buttons.GetPin(pins.Motor3VerifyIn) == rpio.Low)
			n.handlePowerCheck(srv_hardware.CheckServo, n.buttons.GetPin(pins.ServoVerifyIn) == rpio.Low)
		}()
	} else {
		n.buttons.SetPin(pin, rpio.Low)
	}

	return &srv_hardware.RelaySetRes{
		Success: true,
	}
}

func (n *interfaceNode) Done() <-chan struct{} {
	return n.done
}

func (n *interfaceNode) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrapf(n.btnPub.Close(), "failed to close %s publisher", ras.ButtonEventTopic.Topic); err != nil {
			return
		}
		if err = errors.Wrapf(n.buttonInputSrv.Close(), "failed to close %s service", srv_hardware.ButtonEffectServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.interfaceSetSrv.Close(), "failed to close %s service", srv_hardware.InterfaceSetServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.barSetSrv.Close(), "failed to close %s service", srv_hardware.BarSetServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.relaySetSrv.Close(), "failed to close %s service", srv_hardware.RelaySetServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.checkClient.Close(), "failed to close %s client", srv_hardware.PowerCheckServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.emergencyClient.Close(), "failed to close %s client", srv_hardware.EmergencyServiceName); err != nil {
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
