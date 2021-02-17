package ui

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/srvs/srv_hardware"
	"boxbot-go/pkg/srvs/srv_ras"
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"io"
)

type ButtonCallback func(buttonType Button) error

type UI interface {
	io.Closer
	SetButtonCallback(cb ButtonCallback)
	SetButtonEffect(btn Button, effect ButtonEffect) error
	SetAlarmLight(color LightColor, effect LightEffect) error
	SetDisplay(error bool, msg, hint string) error
	SendNotification(msg string) (int, error)
}

type ui struct {
	logger                                              log.Logger
	notificationClient                                  *goroslib.ServiceClient
	interfaceSetClient, buttonInputClient, barSetClient *goroslib.ServiceClient
	buttonEventSub                                      *goroslib.Subscriber

	buttonCallback ButtonCallback
}

func New(n *goroslib.Node, logger log.Logger) (UI, error) {
	var err error
	ui := &ui{
		logger: logger,
	}

	// Create service clients
	ui.notificationClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_ras.CreateNotificationServiceName,
		Srv:             &srv_ras.CreateNotificationService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_ras.CreateNotificationServiceName)
	}
	ui.interfaceSetClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_hardware.InterfaceSetServiceName,
		Srv:             &srv_hardware.InterfaceSetService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_hardware.InterfaceSetServiceName)
	}
	ui.buttonInputClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_hardware.ButtonEffectServiceName,
		Srv:             &srv_hardware.ButtonEffectService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_hardware.ButtonEffectServiceName)
	}
	ui.barSetClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_hardware.BarSetServiceName,
		Srv:             &srv_hardware.BarSetService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_hardware.BarSetServiceName)
	}

	// Create subscriber
	ui.buttonEventSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    ras.ButtonEventTopic.Topic,
		Callback: ui.handleButtonEvent,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s subscriber", ras.ButtonEventTopic.Topic)
	}

	return ui, nil
}

func (ui *ui) handleButtonEvent(e *ras.ButtonEvent) {
	ui.logger.Logf(msgs.DEBUG, "handling button event %s", e.Type)

	btn, err := Button(0).fromRos(e.Type)
	if err != nil {
		ui.logger.Logf(msgs.WARN, "ignoring invalid button event %s", e.Type)
		return
	}

	if err := ui.buttonCallback(btn); err != nil {
		ui.logger.Logf(msgs.DEBUG, "handling button event error (%s): %v", e.Type, err)

		if err := ui.SetButtonEffect(btn, ErrBlink); err != nil {
			ui.logger.Logf(msgs.WARN, "unable to blink error light (%s): %v", e.Type, err)
		}
	}
}

func (ui *ui) SetButtonCallback(cb ButtonCallback) {
	ui.buttonCallback = cb
}

func (ui *ui) SetButtonEffect(btn Button, effect ButtonEffect) error {
	var err error
	req := srv_hardware.ButtonEffectReq{}
	res := srv_hardware.ButtonEffectRes{}

	if req.Button, err = btn.toRos(); err != nil {
		return errors.Wrap(err, "unable to create button effect request")
	}
	if req.ButtonEffect, err = effect.toRos(); err != nil {
		return errors.Wrap(err, "unable to create button effect request")
	}

	if err := ui.buttonInputClient.Call(&req, &res); err != nil {
		return errors.Wrap(err, "unable to call button input service")
	}

	if !res.Success {
		return errors.Errorf("error response from button effect service: %s", res.Error)
	}

	return nil
}

func (ui *ui) SetAlarmLight(color LightColor, effect LightEffect) error {
	var err error
	req := srv_hardware.BarSetReq{}
	res := srv_hardware.BarSetRes{}

	if req.Color, err = color.toRos(); err != nil {
		return errors.Wrap(err, "unable to create set bar request")
	}
	if req.Effect, err = effect.toRos(); err != nil {
		return errors.Wrap(err, "unable to create set bar  request")
	}

	if err := ui.barSetClient.Call(&req, &res); err != nil {
		return errors.Wrap(err, "unable to call set bar service")
	}

	if !res.Success {
		return errors.Errorf("error response from alarm light service: %s", res.Error)
	}

	return nil
}

func (ui *ui) SetDisplay(error bool, msg, hint string) error {
	req := srv_hardware.InterfaceSetReq{
		Error:   error,
		Message: msg,
		Hint:    hint,
	}
	res := srv_hardware.InterfaceSetRes{}

	if err := ui.interfaceSetClient.Call(&req, &res); err != nil {
		return errors.Wrap(err, "unable to call set display service")
	}

	if !res.Success {
		return errors.Errorf("error response from interface service: %s", res.Error)
	}

	return nil
}

func (ui *ui) SendNotification(msg string) (int, error) {
	req := srv_ras.CreateNotificationReq{
		Message: msg,
	}
	res := srv_ras.CreateNotificationRes{}

	if err := ui.notificationClient.Call(&req, &res); err != nil {
		return 0, errors.Wrap(err, "unable to call notification service")
	}

	if !res.Success {
		return 0, errors.Errorf("error response from notification service: %s", res.Error)
	}

	return int(res.Id), nil
}

func (ui *ui) Close() error {
	if err := errors.Wrapf(ui.buttonEventSub.Close(), "failed to close %s subscriber", ras.ButtonEventTopic.Topic); err != nil {
		return err
	}
	if err := errors.Wrapf(ui.notificationClient.Close(), "failed to close %s service client", srv_ras.CreateNotificationServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(ui.buttonInputClient.Close(), "failed to close %s service client", srv_hardware.ButtonEffectServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(ui.interfaceSetClient.Close(), "failed to close %s service client", srv_hardware.InterfaceSetServiceName); err != nil {
		return err
	}

	return nil
}
