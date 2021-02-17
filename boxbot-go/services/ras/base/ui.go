package main

import (
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/ui"
)

func (n *baseNode) mustSetButtonEffect(btn ui.Button, effect ui.ButtonEffect) {
	if err := n.ui.SetButtonEffect(btn, effect); err != nil {
		n.logger.Logf(msgs.WARN, "unable to set button effect for UI (%v, %v): %v", btn, effect, err)
	}

	n.logger.Logf(msgs.DEBUG, "set button effect: %v, %v", btn, effect)
}

func (n *baseNode) mustSetAlarmLight(color ui.LightColor, effect ui.LightEffect) {
	if err := n.ui.SetAlarmLight(color, effect); err != nil {
		n.logger.Logf(msgs.WARN, "unable to set alarm light for UI (%v, %v): %v", color, effect, err)
	}

	n.logger.Logf(msgs.DEBUG, "set alarm light: %v, %v", color, effect)
}

func (n *baseNode) mustSetDisplay(error bool, msg, hint string) {
	if err := n.ui.SetDisplay(error, msg, hint); err != nil {
		n.logger.Logf(msgs.WARN, "unable to set display for UI (%s): %v", msg, err)
	}

	n.logger.Logf(msgs.DEBUG, "set display: %s", msg)
}

func (n *baseNode) mustSendNotification(msg string) {
	id, err := n.ui.SendNotification(msg)

	if err != nil {
		n.logger.Logf(msgs.WARN, "unable to send notification (%s): %v", msg, err)
		return
	}

	n.logger.Logf(msgs.DEBUG, "sent notification (%v): %s", id, msg)
}
