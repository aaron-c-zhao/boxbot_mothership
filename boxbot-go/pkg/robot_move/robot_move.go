package robot_move

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/srvs/boxbot_autonomous"
	"boxbot-go/pkg/srvs/srv_rcs"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"math/rand"
	"sync"
	"time"
)

var MoveCancelledErr = errors.New("move cancelled")

type RobotMove interface {
	io.Closer
	Move(q geometry_msgs.Quaternion, cb func(err error)) error
	MoveDelay(q geometry_msgs.Quaternion, cb func(err error), delay time.Duration) error
	DescendZ(start bool) error
	Stop() error
	GetLocation() (geometry_msgs.Quaternion, error)
	Home(cb func(err error)) error
}

//TODO(timanema): Implement this
type robotMove struct {
	curId *atomic.Int32
	cb    func(err error)

	l log.Logger

	curHomeId *atomic.Int32
	homeCb    func(err error)

	locationLock                                                   sync.Mutex
	moveSrv, stopSrv, locationSrv, setHomeSrv, rotateSrv, readySrv *goroslib.ServiceClient
	homeSrv, cancelHomeSrv                                         *goroslib.ServiceClient

	done chan struct{}
}

func New(n *goroslib.Node, logger log.Logger) (RobotMove, error) {
	r := &robotMove{
		curId:     atomic.NewInt32(-1),
		curHomeId: atomic.NewInt32(-1),
		done:      make(chan struct{}),
		l:         logger,
	}
	var err error

	// Create services
	r.moveSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            boxbot_autonomous.RcsControlMoveServiceName,
		Srv:             &boxbot_autonomous.ControlCmd{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", boxbot_autonomous.RcsControlMoveServiceName)
	}
	r.stopSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            boxbot_autonomous.RcsControlStopServiceName,
		Srv:             &boxbot_autonomous.ControlCmd{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", boxbot_autonomous.RcsControlStopServiceName)
	}
	r.locationSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            boxbot_autonomous.RcsControlLocationServiceName,
		Srv:             &boxbot_autonomous.ControlCmd{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", boxbot_autonomous.RcsControlLocationServiceName)
	}
	r.setHomeSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            boxbot_autonomous.RcsControlHomeServiceName,
		Srv:             &boxbot_autonomous.ControlCmd{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", boxbot_autonomous.RcsControlHomeServiceName)
	}
	r.rotateSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            boxbot_autonomous.RcsControlRotateServiceName,
		Srv:             &boxbot_autonomous.ControlCmd{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", boxbot_autonomous.RcsControlRotateServiceName)
	}

	r.homeSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.HomeServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.HomeServiceName)
	}
	r.cancelHomeSrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.CancelHomeServiceName,
		Srv:             &std_srvs.Trigger{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.CancelHomeServiceName)
	}
	r.readySrv, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:            n,
		Name:            srv_rcs.ReadyServiceName,
		Srv:             &srv_rcs.MotorReadyService{},
		EnableKeepAlive: true,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service client", srv_rcs.ReadyServiceName)
	}

	return r, nil
}

func (r *robotMove) Move(q geometry_msgs.Quaternion, cb func(err error)) error {
	return r.MoveDelay(q, cb, time.Millisecond)
}

func (r *robotMove) MoveDelay(q geometry_msgs.Quaternion, cb func(err error), delay time.Duration) error {
	id := rand.Int31()
	r.curId.Store(id)
	r.cb = cb

	go func() {
		time.Sleep(delay)

		select {
		case <-r.done:
			return
		default:
		}

		fmt.Printf("moving to %v\n", q)

		// Move gripper
		req := &boxbot_autonomous.ControlCmdReq{
			Goal: q,
		}
		res := &boxbot_autonomous.ControlCmdRes{}
		err := errors.Wrap(r.moveSrv.Call(req, res), "failed to call move service")
		if err == nil && res.ErrorCode != 0 {
			err = errors.Errorf("move service return error code: %v", res.ErrorCode)
		}

		// Wait for stepper motors to move there
		if err == nil {
			ready := false

			for !ready {
				readyRes := srv_rcs.MotorReadyRes{}
				err = errors.Wrap(r.readySrv.Call(&srv_rcs.MotorReadyReq{}, &readyRes), "failed to wait for motors")

				if err != nil {
					break
				}

				if readyRes.Failed {
					err = errors.Errorf("ready service returned failure status: %v", readyRes.Message)
					break
				}

				if readyRes.Ready {
					ready = true
					break
				}

				//TODO(timanema): Remove test print
				fmt.Println("wait for motor")

				// Wait before calling the service again
				time.Sleep(time.Second)
			}
		}

		// Rotate gripper
		if err == nil {
			req = &boxbot_autonomous.ControlCmdReq{
				Goal: q,
			}
			res = &boxbot_autonomous.ControlCmdRes{}
			err = errors.Wrap(r.rotateSrv.Call(req, res), "failed to call rotate service")
			if err == nil && res.ErrorCode != 0 {
				err = errors.Errorf("rotate service for rotate return error code: %v", res.ErrorCode)
			}

			//TODO(timanema): For now sleep for a constant amount of time, maybe change into something better later
			r.l.Logf(msgs.DEBUG, "rotating call done, waiting for 5 second to give the servo some time to rotate")
			time.Sleep(time.Second * 5)
			r.l.Logf(msgs.DEBUG, "rotating done (after hardcoded delay)")
		}

		if r.curId.Load() == id {
			fmt.Printf("moved to %v with err: %v\n", q, err)
			cb(err)
			r.cb = nil
		}
	}()

	return nil
}

//TODO(timanema): Implement
func (r *robotMove) DescendZ(start bool) error {
	r.l.Logf(msgs.DEBUG, "descending = %v\n", start)
	return nil
}

//TODO(timanema): Refactor
func (r *robotMove) Stop() error {
	r.curId.Store(-1)
	r.curHomeId.Store(-1)

	var err error

	// Cancel home
	if r.homeCb != nil {
		r.homeCb(MoveCancelledErr)
		r.homeCb = nil

		r.l.Logf(msgs.DEBUG, "cancel home")
		res := &std_srvs.TriggerRes{}
		err = errors.Wrap(r.cancelHomeSrv.Call(&std_srvs.TriggerReq{}, res), "unable to cancel home")
	}

	// Cancel move
	if r.cb != nil {
		r.cb(MoveCancelledErr)
		r.cb = nil

		r.l.Logf(msgs.DEBUG, "cancel regular move")
		err2 := r.stopSrv.Call(&boxbot_autonomous.ControlCmdReq{}, &boxbot_autonomous.ControlCmdRes{})
		if err == nil {
			err = err2
		} else {
			err = errors.Errorf("home err: '%v' & move err: '%v'", err, err2)
		}
	}

	return errors.Wrap(err, "unable to call all stop services")
}

func (r *robotMove) GetLocation() (geometry_msgs.Quaternion, error) {
	//TODO(timanema): Move to cached pub/sub?
	r.locationLock.Lock()
	defer r.locationLock.Unlock()

	res := &boxbot_autonomous.ControlCmdRes{}
	if err := r.locationSrv.Call(&boxbot_autonomous.ControlCmdReq{}, res); err != nil {
		return geometry_msgs.Quaternion{}, errors.Wrap(err, "unable to call get location service")
	}

	return res.Location, nil
}

func (r *robotMove) Home(cb func(err error)) error {
	id := rand.Int31()
	r.curHomeId.Store(id)
	r.homeCb = cb

	go func() {
		r.l.Logf(msgs.DEBUG, "homing motors")
		homeRes := &std_srvs.TriggerRes{}
		err := errors.Wrap(r.homeSrv.Call(&std_srvs.TriggerReq{}, homeRes), "failed to call home service")

		if err == nil && !homeRes.Success {
			err = errors.Errorf("home service failed with error message: %v", homeRes.Message)
		}

		if err == nil {
			r.l.Logf(msgs.DEBUG, "setting home position")
			res := &boxbot_autonomous.ControlCmdRes{}
			err = errors.Wrap(r.setHomeSrv.Call(&boxbot_autonomous.ControlCmdReq{}, res), "failed to call set home service")
			if err == nil && res.ErrorCode != 0 {
				err = errors.Errorf("set home service return error code: %v", res.ErrorCode)
			}
		}

		if r.curHomeId.Load() == id {
			r.l.Logf(msgs.DEBUG, "home position set (err: %v)", err)

			//TODO(timanema): Move this elsewhere
			time.Sleep(time.Second * 5)
			cb(err)
			r.homeCb = nil
		}
	}()

	return nil
}

func (r *robotMove) Close() error {
	if err := r.moveSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", boxbot_autonomous.RcsControlMoveServiceName)
	}
	if err := r.stopSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", boxbot_autonomous.RcsControlStopServiceName)
	}
	if err := r.locationSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", boxbot_autonomous.RcsControlLocationServiceName)
	}
	if err := r.setHomeSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", boxbot_autonomous.RcsControlHomeServiceName)
	}
	if err := r.rotateSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", boxbot_autonomous.RcsControlRotateServiceName)
	}
	if err := r.homeSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", srv_rcs.HomeServiceName)
	}
	if err := r.cancelHomeSrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", srv_rcs.CancelHomeServiceName)
	}
	if err := r.readySrv.Close(); err != nil {
		return errors.Wrapf(err, "unable to close %s service client", srv_rcs.ReadyServiceName)
	}

	close(r.done)

	return nil
}
