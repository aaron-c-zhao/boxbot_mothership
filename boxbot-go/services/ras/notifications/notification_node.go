package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/srvs/srv_ras"
	"boxbot-go/pkg/ws"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"math/rand"
	"os"
	"sync"
)

type notifications struct {
	n   *goroslib.Node
	srv *goroslib.ServiceProvider
	pub *goroslib.Publisher

	logger log.Logger

	c Config

	websock *ws.WebSocketServer

	close, done chan struct{}
	closeLock   sync.Once
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &notifications{
		close: make(chan struct{}, 1),
		done:  make(chan struct{}),
	}

	// Read config
	if n.c, err = ReadConfig(); err != nil {
		return nil, errors.Wrap(err, "unable to read config")
	}

	// Create node
	n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.NotificationServiceName,
		Namespace:     node.RasNamespace,
	})
	if err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.NotificationServiceName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create service
	n.srv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.CreateNotificationServiceName,
		Srv:      &srv_ras.CreateNotificationService{},
		Callback: n.createNotification,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_ras.CreateNotificationServiceName)
	}

	// Create publishers
	n.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n.n,
		Topic: ras.NotificationTopic.Topic,
		Msg:   ras.NotificationTopic.Msg,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s publisher", ras.NotificationTopic.Topic)
	}

	// Create websocket server
	n.websock = ws.New(n.logger, nil)
	go func() {
		if err := n.websock.ListenAndServe(fmt.Sprintf("%s:%d", n.c.WebsocketHost, n.c.WebsocketPort)); err != nil {
			_ = n.Close()
			n.logger.Logf(msgs.ERROR, "unable to start websocket server")
			fmt.Println("unable to start websocket server")
			os.Exit(1)
		}
	}()

	go node.PingSelf(n.n, n.close, n.done, node.NotificationServiceName)

	return n, err
}

func (n *notifications) createNotification(req *srv_ras.CreateNotificationReq) *srv_ras.CreateNotificationRes {
	notification := &ras.Notification{
		Id:      rand.Uint32(),
		Message: req.Message,
	}

	n.pub.Write(notification)

	n.websock.Publish(&ws.Notification{
		Id:      notification.Id,
		Message: notification.Message,
	})

	n.logger.Logf(msgs.INFO, "Published notification with message (ID: %v): '%v'", notification.Id, notification.Message)

	return &srv_ras.CreateNotificationRes{
		Id:      notification.Id,
		Success: true,
	}
}

func (n *notifications) Done() <-chan struct{} {
	return n.done
}

func (n *notifications) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrapf(n.pub.Close(), "failed to close %s publisher", ras.NotificationTopic.Topic); err != nil {
			return
		}
		if err = errors.Wrapf(n.srv.Close(), "failed to close %s service", srv_ras.CreateNotificationServiceName); err != nil {
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
