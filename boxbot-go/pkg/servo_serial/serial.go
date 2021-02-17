package servo_serial

import (
	"encoding/binary"
	"fmt"
	"github.com/jacobsa/go-serial/serial"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"io/ioutil"
	"sync"
)

var ServoNotYetErr = errors.New("servo connection not yet ready")
var InvalidAngleErr = errors.Errorf("invalid angle, outside of range %d - %d", 0, 180)

const mappingSlope float64 = 2000.0 / 180.0

type ServoCommand = byte

const (
	SetPosition ServoCommand = 0x00
	Ready       ServoCommand = 0x01
)

/*
Quick overview of protocol used to talk to servo controllers

- Protocol is request based

- Commands:
  - SetPosition: 0x00
    cmd + position (uint16) + speed (uint16)
    0x00 -> 0x00 0x42
*/
type SerialServo struct {
	opts     serial.OpenOptions
	Port     string
	rwCloser io.ReadWriteCloser
	sync.Mutex

	connReady *atomic.Bool
}

func New(p string) *SerialServo {
	options := serial.OpenOptions{
		PortName:              p,
		BaudRate:              153600,
		DataBits:              8,
		StopBits:              1,
		InterCharacterTimeout: 100,
	}

	return &SerialServo{
		opts:      options,
		Port:      p,
		connReady: atomic.NewBool(false),
	}
}

func mapAngleToMicroseconds(angle float64) uint16 {
	return uint16(500 + mappingSlope*angle)
}

func (s *SerialServo) SetPosition(angle float64, speed float64) error {
	if !s.connReady.Load() {
		return ServoNotYetErr
	}

	buf := make([]byte, 5)

	if angle < -10 || angle > 190 {
		return InvalidAngleErr
	}

	// Set command
	buf[0] = SetPosition

	fmt.Printf("writing %d\n", mapAngleToMicroseconds(angle))

	// Map angle, and set it
	binary.LittleEndian.PutUint16(buf[1:3], mapAngleToMicroseconds(angle))

	// Set speed
	binary.LittleEndian.PutUint16(buf[3:5], uint16(speed/50.0*mappingSlope))

	_, err := s.rwCloser.Write(buf)

	return errors.Wrap(err, "unable to write request to controller")
}

func (s *SerialServo) AtTarget() (bool, error) {
	if !s.connReady.Load() {
		return false, ServoNotYetErr
	}

	// Write command
	if _, err := s.rwCloser.Write([]byte{Ready}); err != nil {
		return false, errors.Wrap(err, "unable to write request to servo controller")
	}

	// Read response
	buf := make([]byte, 1)
	if n, err := s.rwCloser.Read(buf); err != nil || n != 1 {
		return false, errors.Wrapf(err, "unable to read response from servo controller (n = %v)", n)
	}

	if buf[0] == 0x00 {
		return false, nil
	}

	return true, nil
}

// Start sends a ping to the controller, and starts the connection checks. This needs to be called before
// the stepper controller can be used.
func (s *SerialServo) Start() error {
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

	//// Send ping
	//if err := s.SetPosition(90, 500); err != nil {
	//	return errors.Wrap(err, "failed to start connection")
	//}

	s.connReady.Store(true)

	return nil
}

func (s *SerialServo) Ready() bool {
	return s.connReady.Load()
}

func (s *SerialServo) Close() error {
	// If the connection isn't even ready, there is nothing to clean up
	if !s.connReady.Load() {
		return nil
	}

	s.connReady.Store(false)
	return errors.Wrap(s.rwCloser.Close(), "unable to close serial connection")
}
