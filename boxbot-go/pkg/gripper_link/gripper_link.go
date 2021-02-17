package gripper_link

import (
	"errors"
	"fmt"
	"go.uber.org/atomic"
	"io"
	"math/rand"
	"time"
)

var GrippingFailedErr = errors.New("failed to grip carton")
var PreemptedErr = errors.New("gripper action was preempted")

type Gripper interface {
	io.Closer
	OpenGripper() error
	StartSearch(cb func(err error)) error
	StartGrip(cb func(err error)) error
	Stop() error
}

//TODO(timanema): Implement
type gripper struct {
	curId *atomic.Int32
	cb    func(err error)
}

func New() Gripper {
	return &gripper{
		curId: atomic.NewInt32(-1),
	}
}

func (g *gripper) OpenGripper() error {
	return nil
}

func (g *gripper) StartSearch(cb func(err error)) error {
	id := rand.Int31()
	g.curId.Store(id)
	g.cb = cb

	go func() {
		time.Sleep(time.Second * 2)
		if g.curId.Load() == id {
			fmt.Println("searched")
			cb(nil)
		}
	}()

	return nil
}

func (g *gripper) StartGrip(cb func(err error)) error {
	id := rand.Int31()
	g.curId.Store(id)
	g.cb = cb

	go func() {
		time.Sleep(time.Second * 2)
		if g.curId.Load() == id {
			fmt.Println("gripped")
			cb(nil)
		}
	}()

	return nil
}

func (g *gripper) Stop() error {
	g.curId.Store(-1)
	g.cb(PreemptedErr)
	return nil
}

func (g *gripper) Close() error {
	return nil
}
