package main

import (
	"boxbot-go/pkg/msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"time"
)

func (n *watchdog) runResetTimer() {
	ticker := time.NewTicker(time.Second * 3)

	for range ticker.C {
		select {
		case <-n.close:
			return
		default:
		}

		n.timerLock.Lock()
		if n.timerReset {
			if err := n.resetHardwareTimer(); err != nil {
				//TODO(timanema): Add retry logic?
				n.logger.Logf(msgs.ERROR, "failed to reset hardware timer: %v", err)
			}
		}
		n.timerLock.Unlock()
	}
}

func (n *watchdog) resetHardwareTimer() error {
	n.logger.Logf(msgs.TRACE, "resetting hardware timer")

	res := std_srvs.TriggerRes{}
	if err := n.pinResetClient.Call(&std_srvs.TriggerReq{}, &res); err != nil {
		return errors.Wrap(err, "unable to call pin reset service")
	}

	if !res.Success {
		return errors.Errorf("pin reset service reported failure: %s", res.Message)
	}

	return nil
}

func (n *watchdog) setHardwareReset(enabled bool) {
	n.timerLock.Lock()
	defer n.timerLock.Unlock()

	n.timerReset = enabled
}
