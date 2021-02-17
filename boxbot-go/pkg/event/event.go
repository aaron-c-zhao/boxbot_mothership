package event

import (
	"go.uber.org/atomic"
)

type Callback func() error

type Event struct {
	activeCb, inActive Callback
	active             *atomic.Bool
}

func NewEvent(activeCb, inactiveCb Callback) *Event {
	return &Event{
		activeCb: activeCb,
		inActive: inactiveCb,
		active:   atomic.NewBool(false),
	}
}

func (e *Event) Active() error {
	if !e.IsActive() {
		e.active.Store(true)
		return e.activeCb()
	}

	return nil
}

func (e *Event) Inactive() error {
	if e.IsActive() {
		e.active.Store(false)
		return e.inActive()
	}

	return nil
}

func (e *Event) IsActive() bool {
	return e.active.Load()
}
