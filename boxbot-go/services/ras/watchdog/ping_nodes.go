package main

import (
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/node"
	"time"
)

func (n *watchdog) run() {
	ticker := time.NewTicker(time.Second)

	for range ticker.C {
		select {
		case <-n.runClose:
			return
		case <-n.close:
			return
		default:
		}

		//TODO(timanema): Change to all nodes later
		if err := n.pingNodes([]string{}, node.RasNodes); err != nil && !n.failed {
			n.toggleFailed()
		} else if err == nil && n.failed {
			n.toggleFailed()
		}
	}
}

func (n *watchdog) pingNodes(normalNodes, fatalNodes []string) error {
	for _, name := range normalNodes {
		if err := n.ping(name); err != nil {
			n.logger.Logf(msgs.ERROR, "node %s failed, notifying operators")
			return err
		}
	}

	for _, name := range fatalNodes {
		if err := n.ping(name); err != nil {
			n.logger.Logf(msgs.ERROR, "fatal node %s failed, stopping hardware timer reset", name)
			n.setHardwareReset(false)

			return err
		}
	}

	n.setHardwareReset(true)
	return nil
}

func (n *watchdog) ping(name string) error {
	delay := time.Second
	for i := 0; i < n.c.FailureLimit; i++ {
		//TODO(timanema): Look for something better later
		select {
		case <-n.close:
			return nil
		default:
		}

		if _, err := n.n.PingNode(name); err != nil {
			n.logger.Logf(msgs.DEBUG, "ping failed for node %s, retrying in %ds", name, delay/time.Second)
			time.Sleep(delay)
			delay *= 2
			continue
		}

		return nil
	}

	return PingFailedErr
}
