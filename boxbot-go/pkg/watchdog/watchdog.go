package watchdog

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/srvs/srv_ras"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"io"
)

type Watchdog interface {
	io.Closer
	EnableWatchdog() error
	DisableWatchdog() error
	SetCallback(func(bool))
}

type watchdog struct {
	logger                  log.Logger
	startClient, stopClient *goroslib.ServiceClient
	watchdogSub             *goroslib.Subscriber

	cb func(bool)
}

func New(n *goroslib.Node, logger log.Logger) (Watchdog, error) {
	var err error
	w := &watchdog{
		logger: logger,
	}

	// Create clients
	w.startClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_ras.StartWatchdogServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s client", srv_ras.StartWatchdogServiceName)
	}
	w.stopClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_ras.StopWatchdogServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s client", srv_ras.StopWatchdogServiceName)
	}

	// Create subscribers
	w.watchdogSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    ras.WatchdogTopic.Topic,
		Callback: w.handleWatchdogAlert,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s subscriber", ras.WatchdogTopic.Topic)
	}

	return w, err
}

func (w *watchdog) handleWatchdogAlert(alert *ras.WatchdogAlert) {
	w.cb(alert.Failed)
}

func (w *watchdog) EnableWatchdog() error {
	res := std_srvs.TriggerRes{}

	err := w.startClient.Call(&std_srvs.TriggerReq{}, &res)
	if err != nil {
		return errors.Wrap(err, "unable to call watchdog start service")
	}

	if !res.Success {
		return errors.Errorf("starting watchdog failed, result from start service call: %s", res.Message)
	}

	return nil
}

func (w *watchdog) DisableWatchdog() error {
	res := std_srvs.TriggerRes{}

	err := w.stopClient.Call(&std_srvs.TriggerReq{}, &res)
	if err != nil {
		return errors.Wrap(err, "unable to call watchdog stop service")
	}

	if !res.Success {
		return errors.Errorf("starting watchdog failed, result from stop service call: %s", res.Message)
	}

	return nil
}

func (w *watchdog) SetCallback(cb func(bool)) {
	w.cb = cb
}

func (w *watchdog) Close() error {
	if err := errors.Wrapf(w.startClient.Close(), "failed to close %s service client", srv_ras.StartWatchdogServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(w.stopClient.Close(), "failed to close %s service client", srv_ras.StopWatchdogServiceName); err != nil {
		return err
	}
	if err := errors.Wrapf(w.watchdogSub.Close(), "failed to close %s subscriber", ras.WatchdogTopic.Topic); err != nil {
		return err
	}

	return nil
}
