package node

import (
	"errors"
	"github.com/aler9/goroslib"
	"time"
)

const (
	pingDelay   = 500 * time.Millisecond
	pingLimit   = 4
	waitTimeout = 15 * time.Second
)

var WaitTimeoutErr = errors.New("timeout while waiting for nodes to come online")
var WaitClosedErr = errors.New("node closed while waiting for nodes to come online")

func PingSelf(n *goroslib.Node, cl <-chan struct{}, done chan<- struct{}, name string) {
	ticker := time.NewTicker(pingDelay)
	cnt := 0

	for range ticker.C {
		select {
		case <-cl:
			close(done)
			return
		default:
		}

		if _, err := n.PingNode(name); err != nil {
			cnt += 1
		} else {
			cnt = 0
		}

		if cnt > pingLimit {
			close(done)
			return
		}
	}
}

func WaitForNodes(n *goroslib.Node, cl <-chan struct{}, names []string) error {
	done := make(chan struct{})

	go waitForNodes(n, cl, names, done)

	select {
	case <-done:
		return nil
	case <-cl:
		return WaitClosedErr
	case <-time.After(waitTimeout):
		return WaitTimeoutErr
	}
}

func waitForNodes(n *goroslib.Node, cl <-chan struct{}, names []string, done chan<- struct{}) {
	ticker := time.NewTicker(pingDelay)
delay:
	for range ticker.C {
		select {
		case <-cl:
			return
		default:
		}

		for _, name := range names {
			if _, err := n.PingNode(name); err != nil {
				continue delay
			}
		}

		done <- struct{}{}
	}
}
