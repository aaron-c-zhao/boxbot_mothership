package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/srvs/srv_hardware"
	"boxbot-go/pkg/srvs/srv_ras"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"sync"
)

var PingFailedErr = errors.New("ping to node failed")

type watchdog struct {
	n                 *goroslib.Node
	pub               *goroslib.Publisher
	startSrv, stopSrv *goroslib.ServiceProvider
	pinResetClient    *goroslib.ServiceClient

	logger log.Logger

	c Config

	close, done chan struct{}
	closeLock   sync.Once

	timerReset bool
	timerLock  sync.Mutex

	runLock  sync.Once
	stopLock sync.Once
	runClose chan struct{}

	failed bool
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &watchdog{
		close:      make(chan struct{}, 1),
		done:       make(chan struct{}),
		timerReset: true,
		runClose:   make(chan struct{}, 1),
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "unable to read config")
	}

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.WatchdogName,
		Namespace:     node.RasNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.WatchdogName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create publishers
	n.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n.n,
		Topic: ras.WatchdogTopic.Topic,
		Msg:   ras.WatchdogTopic.Msg,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s publisher", ras.WatchdogTopic.Topic)
	}

	// Create service
	n.startSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.StartWatchdogServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.startWatchdog,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service provider", srv_ras.StartWatchdogServiceName)
	}
	n.stopSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.StopWatchdogServiceName,
		Srv:      &std_srvs.Trigger{},
		Callback: n.stopWatchdog,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service provider", srv_ras.StopWatchdogServiceName)
	}

	// Create clients
	n.pinResetClient, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n.n,
		Name:            srv_hardware.PulseWatchdogPinServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s client", srv_hardware.PulseWatchdogPinServiceName)
	}

	go node.PingSelf(n.n, n.close, n.done, node.WatchdogName)

	return n, err
}

func (n *watchdog) startWatchdog(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	success := false

	n.runLock.Do(func() {
		go n.run()
		go n.runResetTimer()
		n.logger.Logf(msgs.INFO, "watchdog received signal to start")
		success = true
	})

	return &std_srvs.TriggerRes{
		Success: success,
	}
}

func (n *watchdog) stopWatchdog(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	success := false

	n.stopLock.Do(func() {
		close(n.runClose)
		n.logger.Logf(msgs.INFO, "Watchdog received signal to stop")
		success = true
	})

	return &std_srvs.TriggerRes{
		Success: success,
	}
}

func (n *watchdog) toggleFailed() {
	n.failed = !n.failed
	n.pub.Write(&ras.WatchdogAlert{
		Failed: n.failed,
	})

	// Log appropriate message
	if n.failed {
		n.logger.Logf(msgs.ERROR, "Watchdog triggered. New failed state: %v", n.failed)
	} else {
		n.logger.Logf(msgs.INFO, "Watchdog restored. New failed state: %v", n.failed)
	}
}

func (n *watchdog) Done() <-chan struct{} {
	return n.done
}

func (n *watchdog) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrapf(n.pub.Close(), "failed to close %s publisher", ras.WatchdogTopic.Topic); err != nil {
			return
		}
		if err = errors.Wrapf(n.startSrv.Close(), "failed to close %s service", srv_ras.StartWatchdogServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.stopSrv.Close(), "failed to close %s service", srv_ras.StopWatchdogServiceName); err != nil {
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
