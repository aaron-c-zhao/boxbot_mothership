package gripper_serial

import (
	"encoding/binary"
	"fmt"
	"github.com/jacobsa/go-serial/serial"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"io/ioutil"
	"math"
	"sync"
)

var GripperNotReadyErr = errors.New("gripper not yet ready")

type MoveDirCallback func(direction Direction, indefinite bool, amount float32)
type DetectHitCallback func(sensor Sensor)

/*
Quick overview of protocol used to talk to the gripper controller

- Protocol is bidirectional request

- Commands (this to gripper):
  - CommandCompleted: 0x00
    cmd ->
    0x00 ->
  - StartSearch: 0x01
    cmd ->
    0x01 ->
  - StartGrip: 0x02
    cmd ->
    0x02 ->
  - Reset: 0x03
    cmd ->
    0x03 ->

- Commands (gripper to this):
  - SearchCompleted: 0x04
    -> cmd
    -> 0x04
  - SearchFailed: 0x05
    -> cmd
    -> 0x05
  - GripCompleted: 0x06
    -> cmd
    -> 0x06
  - GripFailed: 0x07
    -> cmd
    -> 0x07
  - MoveDirCommand: 0x08
    -> cmd + direction (byte) + indefinite (bool) + amount (float32)
    -> 0x08 + 0x00 + 0x00 + 0x14 0xae 0x29 0x42
  - DetectHit: 0x09
    -> cmd + sensor (byte)
    -> 0x09 + 0x01
  - WaitAnalyse: 0x0A
    -> cmd
    -> 0x0A

- Directions (top-down, from base):
  Left: 0x00 (x)
  Right: 0x01 (x)
  Up: 0x02 (y)
  Down: 0x03 (y)
  Higher: 0x04 (z)
  Lower: 0x05 (z)

- Sensors (sensors on right side, seen from below):
  Upper IR: 0x00
  Lower IR: 0x01
*/
type SerialGripper struct {
	opts     serial.OpenOptions
	Port     string
	rwCloser io.ReadWriteCloser
	sync.Mutex

	connReady *atomic.Bool

	// callbacks
	searchCompleted func()
	searchFailed    func()
	gripCompleted   func()
	gripFailed      func()
	moveDir         MoveDirCallback
	detectHit       DetectHitCallback
	waitAnalyse     func()
}

func New(p string, searchCompleted func(), searchFailed func(), gripCompleted func(), gripFailed func(),
	moveDir MoveDirCallback, detectHit DetectHitCallback, waitAnalyse func()) *SerialGripper {
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

		// callbacks
		searchCompleted: searchCompleted,
		searchFailed:    searchFailed,
		gripCompleted:   gripCompleted,
		gripFailed:      gripFailed,
		moveDir:         moveDir,
		detectHit:       detectHit,
		waitAnalyse:     waitAnalyse,
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

	go s.run()

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

func (s *SerialGripper) StartGrip() error {
	return s.genericCommand([]byte{StartGrip})
}

func (s *SerialGripper) Reset() error {
	return s.genericCommand([]byte{Reset})
}

//func (s *SerialGripper) AnalyseCompleted(val float32) error {
//	buf := []byte{AnalyseCompleted, 0x00, 0x00, 0x00, 0x00}
//	binary.LittleEndian.PutUint32(buf[1:], math.Float32bits(val))
//
//	return s.genericCommand(buf)
//}

//TODO(timanema): Implement
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
		case SearchCompleted:
			go s.searchCompleted()
		case SearchFailed:
			go s.searchFailed()
		case GripCompleted:
			go s.gripCompleted()
		case GripFailed:
			go s.gripFailed()
		case MoveDirCommand:
			buf, _ := s.genericRead(6)
			indef := false
			if buf[1] == 0x01 {
				indef = true
			}

			go s.moveDir(Direction(buf[0]), indef, math.Float32frombits(binary.LittleEndian.Uint32(buf[2:6])))
		case DetectHit:
			_, _ = s.genericRead(1)
			go s.detectHit(UpperIR)
		case WaitAnalyse:
			go s.waitAnalyse()
		default:
			fmt.Printf("unknown command received: %v\n", cmdBuf[0])
		}
	}
}

func (s *SerialGripper) Ready() bool {
	return s.connReady.Load()
}

func (s *SerialGripper) Close() error {
	// If the connection isn't even ready, there is nothing to clean up
	if !s.connReady.Load() {
		return nil
	}

	s.connReady.Store(false)
	return errors.Wrap(s.rwCloser.Close(), "unable to close serial connection")
}
