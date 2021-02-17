package gripper_serial

import (
	"fmt"
	"github.com/jacobsa/go-serial/serial"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"io/ioutil"
	"sync"
)

var GripperNotReadyErr = errors.New("gripper not yet ready")

type DetectHitCallback func(sensor Sensor)

/*
Flow:

Reset ->
StartSearch ->
*move gripper lower indefinitely*
-> START_HIGHER_1
*move gripper higher for fixed amount*
CommandCompleted ->
*move right and record IR locations*
-> DETECT_HIT x2
-> RIGHT_EDGE_FOUND
*move back to starting position*
CommandCompleted ->
*move up/down depending on carton*
-> DETECT_HIT
-> HL_EDGE_FOUND
*move to calculated position*
*move lower for fixed amount*
IF -> GRIP_FAILED: *call operator*
CommandCompleted ->
IF -> GRIP_FAILED: *call operator*
IF -> GRIP_DONE: *move higher for fixed amount* *DONE*

*/
type SerialGripper struct {
	opts     serial.OpenOptions
	Port     string
	rwCloser io.ReadWriteCloser
	sync.Mutex

	connReady *atomic.Bool

	statusCh chan bool

	// callbacks
	gripCompleted    func()
	gripFailed       func()
	detectHit        DetectHitCallback
	startHigher      func()
	rightEdgeFound   func()
	highLowEdgeFound func()
}

func New(p string, gripCompleted func(), gripFailed func(), detectHit DetectHitCallback, startHigher func(),
	rightEdgeFound func(), highLowEdgeFound func()) *SerialGripper {
	options := serial.OpenOptions{
		PortName:              p,
		BaudRate:              153600,
		DataBits:              8,
		StopBits:              1,
		InterCharacterTimeout: 100,
	}

	return &SerialGripper{
		opts:      options,
		Port:      p,
		connReady: atomic.NewBool(false),

		statusCh: make(chan bool),

		// Callbacks
		gripCompleted:    gripCompleted,
		gripFailed:       gripFailed,
		detectHit:        detectHit,
		startHigher:      startHigher,
		rightEdgeFound:   rightEdgeFound,
		highLowEdgeFound: highLowEdgeFound,
	}
}

// Start sends a ping to the controller, and starts the connection checks. This needs to be called before
// the stepper controller can be used.
func (s *SerialGripper) Start() error {
	s.Lock()
	defer s.Unlock()

	// Open serial connection
	var err error
	s.rwCloser, err = serial.Open(s.opts)
	if err != nil {
		return errors.Wrap(err, "unable to open serial connection")
	}

	// Clear old data from buffer
	_, _ = ioutil.ReadAll(s.rwCloser)

	s.connReady.Store(true)

	// Send reset command
	if err := s.genericCommandRaw([]byte{Reset}); err != nil {
		return errors.Wrap(err, "failed to start connection")
	}

	go func() {
		if err := s.run(); err != nil {
			fmt.Println("ERROR: unable to run gripper serial loop")
		}
	}()

	return nil
}

func (s *SerialGripper) genericCommandRaw(cmd []byte) error {
	if !s.connReady.Load() {
		return GripperNotReadyErr
	}

	// Write to serial conn
	_, err := s.rwCloser.Write(cmd)
	return errors.Wrap(err, "unable to write request to controller")
}

func (s *SerialGripper) genericCommand(cmd []byte) error {
	s.Lock()
	defer s.Unlock()

	return s.genericCommandRaw(cmd)
}

func (s *SerialGripper) genericRead(n int) ([]byte, error) {
	buf := make([]byte, n)
	fmt.Printf("reading %d\n", n)
	r, err := s.rwCloser.Read(buf)
	if err != nil {
		return nil, errors.Wrap(err, "failed to read from serial connection")
	}

	if r != n {
		return nil, errors.Errorf("failed to read %d bytes of data, got %d", n, r)
	}

	return buf, nil
}

func (s *SerialGripper) CommandCompleted() error {
	return s.genericCommand([]byte{CommandCompleted})
}

func (s *SerialGripper) StartSearch() error {
	return s.genericCommand([]byte{StartSearch})
}

func (s *SerialGripper) Reset() error {
	return s.genericCommand([]byte{Reset})
}

func (s *SerialGripper) run() error {
	if !s.connReady.Load() {
		return GripperNotReadyErr
	}

	cmdBuf := make([]byte, 1)
	for {
		if !s.connReady.Load() {
			return nil
		}

		n, err := s.rwCloser.Read(cmdBuf)
		if err != nil && err != io.EOF {
			fmt.Printf("read err: %v\n", err)
			continue
		}

		if n == 0 {
			continue
		}

		fmt.Printf("handling cmd %d\n", cmdBuf[0])
		switch cmdBuf[0] {
		case GripCompleted:
			go s.gripCompleted()
		case GripFailed:
			go s.gripFailed()
		case DetectHit:
			buf, err := s.genericRead(1)
			if err != nil {
				fmt.Printf("invalid DetectHit command received: %v\n", err)
				continue
			}

			go s.detectHit(Sensor(buf[0]))
		case StartHigher:
			//TODO(timanema): Remove debug data
			buf, err := s.genericRead(1)
			if err != nil {
				fmt.Printf("invalid DetectHit command received: %v\n", err)
				continue
			}

			fmt.Printf("start higher DEBUG data: %v\n", buf)

			go s.startHigher()
		case RightEdgeFound:
			go s.rightEdgeFound()
		case HighLowEdgeFound:
			go s.highLowEdgeFound()
		//case GetGripStatusResponse:
		//	go func() {
		//		s.statusCh <-
		//	}()
		default:
			fmt.Printf("unknown command received: %v\n", cmdBuf[0])
		}
	}
}

func (s *SerialGripper) Ready() bool {
	return s.connReady.Load()
}

func (s *SerialGripper) SetOpen(open bool) error {
	if open {
		if err := s.genericCommand([]byte{OpenGripper}); err != nil {
			return errors.Wrap(err, "unable to send gripper open command")
		}
	} else {
		if err := s.genericCommand([]byte{CloseGripper}); err != nil {
			return errors.Wrap(err, "unable to send gripper close command")
		}
	}
	return nil
}

func (s *SerialGripper) IsOpen() (bool, error) {
	if err := s.genericCommand([]byte{GetGripStatus}); err != nil {
		return false, errors.Wrap(err, "unable to send gripper status command")
	}

	// Read response
	buf := make([]byte, 1)
	if n, err := s.rwCloser.Read(buf); err != nil || n != 1 {
		return false, errors.Wrapf(err, "unable to read response from gripper controller (n = %v)", n)
	}

	if buf[0] == 0x00 {
		return false, nil
	}

	return true, nil
}

func (s *SerialGripper) Close() error {
	// If the connection isn't even ready, there is nothing to clean up
	if !s.connReady.Load() {
		return nil
	}

	s.connReady.Store(false)
	return errors.Wrap(s.rwCloser.Close(), "unable to close serial connection")
}
