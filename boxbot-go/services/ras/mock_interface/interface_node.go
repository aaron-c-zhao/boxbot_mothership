package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/srvs/srv_hardware"
	"boxbot-go/pkg/ws"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"os"
	"sync"
)

type interfaceNode struct {
	n                                          *goroslib.Node
	buttonInputSrv, interfaceSetSrv, barSetSrv *goroslib.ServiceProvider
	relaySetSrv                                *goroslib.ServiceProvider
	btnPub                                     *goroslib.Publisher
	checkClient, emergencyClient               *goroslib.ServiceClient

	logger log.Logger

	close, done chan struct{}
	closeLock   sync.Once

	websock *ws.WebSocketServer

	// Used for CLI
	StartBtn, PauseBtn, PowerBtn srv_hardware.ButtonEffectType
	InterfaceText                string
	BarColor                     srv_hardware.BarColor
	BarEffect                    srv_hardware.BarEffect

	InputPower, Motor1Power, Motor2Power, Motor3Power, ServoPower bool
	Emergency                                                     bool
	MotorRelayActive, ServoRelayActive                            bool
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &interfaceNode{
		close: make(chan struct{}, 1),
		done:  make(chan struct{}),
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
		if err := n.websock.ListenAndServe("localhost:8082"); err != nil {
			_ = n.Close()
			n.logger.Logf(msgs.ERROR, "unable to start websocket server")
			fmt.Println("unable to start websocket server")
			os.Exit(1)
		}
	}()

	go node.PingSelf(n.n, n.close, n.done, node.InterfaceName)

	return n, err
}

func (n *interfaceNode) Done() <-chan struct{} {
	return n.done
}

func (n *interfaceNode) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.websock.Close(), "failed to close web socket server"); err != nil {
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
