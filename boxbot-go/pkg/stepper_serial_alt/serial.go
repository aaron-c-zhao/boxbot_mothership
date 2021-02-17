package stepper_serial_alt

import (
	"boxbot-go/pkg/msgs/rcs"
	"boxbot-go/pkg/srvs/srv_rcs"
	"encoding/binary"
	"fmt"
	"github.com/jacobsa/go-serial/serial"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"io/ioutil"
	"math"
	"sync"
	"time"
)

const watchdogTimeout = 1700 * time.Millisecond

var InvalidResponseErr = errors.New("stepper controller responded with incorrect data")

type StepperCommand = byte

const (
	Ping           StepperCommand = 0x00
	SetMotor       StepperCommand = 0x01
	GetMotorStatus StepperCommand = 0x02
)

/*
Quick overview of protocol used to talk to stepper controllers

- Protocol is request+response based

- If no ping is received for more than 2 seconds, the controller will set the speed to zero and go into fault state, until a new ping is received

- Commands:
  - Ping: 0x00
    cmd -> cmd + pong
    0x00 -> 0x00 0x42
  - SetMotor: 0x01
    cmd + speed (float32) + disabled (bool) -> cmd + err_code (byte)
    0x01 + 0x14 0xae 0x29 0x42 + 0x00 -> 0x01 + 0x00
  - GetMotorStatus: 0x02
    cmd -> cmd + state (byte) + speed (float32)
    0x02 -> 0x02 + 0x00 + 0x14 0xae 0x29 0x42

- States:
  Calibrating =   0b000000
  Blocked left =  0b000001
  Blocked right = 0b000010
  Idle =          0b000100
  Moving =        0b001000
  Disabled =      0b010000
  Fault =         0b100000

- Error codes:
  0x00: No error
  0x01: Blocked by limit switches
  0x02: Motor disabled
  0x03: Watchdog fault
*/
type SerialStepper struct {
	opts     serial.OpenOptions
	Port     string
	rwCloser io.ReadWriteCloser
	sync.Mutex

	connReady *atomic.Bool
}

func New(p string) *SerialStepper {
	options := serial.OpenOptions{
		PortName:              p,
		BaudRate:              153600,
		DataBits:              8,
		StopBits:              1,
		InterCharacterTimeout: 100,
	}

	return &SerialStepper{
		opts:      options,
		Port:      p,
		connReady: atomic.NewBool(false),
	}
}

func (s *SerialStepper) ping() error {
	if _, err := s.rwCloser.Write([]byte{Ping}); err != nil {
		return errors.Wrap(err, "unable to write request to controller")
	}

	res := make([]byte, 2)
	n, err := s.rwCloser.Read(res)
	if err != nil {
		return errors.Wrap(err, "unable to read response from controller")
	}

	if n != 2 || res[0] != Ping || res[1] != 0x42 {
		return InvalidResponseErr
	}

	return nil
}

func (s *SerialStepper) run() {
	ticker := time.NewTicker(watchdogTimeout)

	for range ticker.C {
		s.Lock()
		//TODO(timanema): Log errors
		if err := s.ping(); err != nil {
			fmt.Printf("err %v\n", err)
		}

		s.Unlock()
	}
}

// Start sends a ping to the controller, and starts the connection checks. This needs to be called before
// the stepper controller can be used.
func (s *SerialStepper) Start() error {
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

	// Send ping
	if err := s.ping(); err != nil {
		return errors.Wrap(err, "failed to start connection")
	}

	s.connReady.Store(true)

	// Start watchdog loop
	go s.run()

	return nil
}

func (s *SerialStepper) SetMotor(ctr *rcs.Joint, speed float32) (srv_rcs.ErrorCode, error) {
	s.Lock()
	defer s.Unlock()

	if !s.connReady.Load() {
		return srv_rcs.MotorCalibrating, nil
	}

	moveBuf := make([]byte, 8)
	binary.LittleEndian.PutUint32(moveBuf[:4], math.Float32bits(speed))
	binary.LittleEndian.PutUint32(moveBuf[4:8], uint32(ctr.Position))

	// Create request for controller
	buf := []byte{SetMotor}
	buf = append(buf, moveBuf...)
	if ctr.Disable {
		buf = append(buf, 0x1)
	} else {
		buf = append(buf, 0x0)
	}

	if _, err := s.rwCloser.Write(buf); err != nil {
		return 0, errors.Wrap(err, "unable to write request to controller")
	}

	buf = make([]byte, 2)
	n, err := s.rwCloser.Read(buf)
	if err != nil {
		return 0, errors.Wrapf(err, "unable to read response from controller (%v bytes read)", n)
	}

	if n != 2 || buf[0] != SetMotor {
		return 0, InvalidResponseErr
	}

	return srv_rcs.ErrorCode(buf[1]), nil
}

func (s *SerialStepper) GetStatus() (srv_rcs.Status, error) {
	s.Lock()
	defer s.Unlock()

	if !s.connReady.Load() {
		return srv_rcs.Status{
			Calibrating: true,
			Idle:        true,
		}, nil
	}

	if _, err := s.rwCloser.Write([]byte{GetMotorStatus}); err != nil {
		return srv_rcs.Status{}, errors.Wrap(err, "unable to write request to controller")
	}

	buf := make([]byte, 6)
	n, err := s.rwCloser.Read(buf)
	if err != nil {
		return srv_rcs.Status{}, errors.Wrap(err, "unable to read response from controller")
	}

	if n != 6 || buf[0] != GetMotorStatus {
		return srv_rcs.Status{}, InvalidResponseErr
	}

	return srv_rcs.Status{
		Calibrating:  buf[1] == 0x00,
		BlockedLeft:  (buf[1] & 0x01) != 0x00,
		BlockedRight: (buf[1] & 0x02) != 0x00,
		Idle:         (buf[1] & 0x04) != 0x00,
		Moving:       (buf[1] & 0x08) != 0x00,
		Disabled:     (buf[1] & 0x10) != 0x00,
		Fault:        (buf[1] & 0x20) != 0x00,
		Speed:        math.Float32frombits(binary.LittleEndian.Uint32(buf[2:6])),
	}, nil
}

func (s *SerialStepper) Ready() bool {
	return s.connReady.Load()
}

func (s *SerialStepper) Close() error {
	// If the connection isn't even ready, there is nothing to clean up
	if !s.connReady.Load() {
		return nil
	}

	s.connReady.Store(false)
	return errors.Wrap(s.rwCloser.Close(), "unable to close serial connection")
}
