package stepper_serial

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
var HomingFailedErr = errors.New("homing failed")
var HomingOffsetBlockedErr = errors.New("unable to move to home offset, since the limit switch in that " +
	"direction is blocked")
var HomingOffsetSpeedErr = errors.New("unable to move to home offset, since the speed is 0, " +
	"did a limit switch trigger?")

type StepperCommand = byte

const (
	Ping           StepperCommand = 0x00
	SetMotor       StepperCommand = 0x01
	GetMotorStatus StepperCommand = 0x02
	StartHoming    StepperCommand = 0x03
	StopHoming     StepperCommand = 0x04
	Zero           StepperCommand = 0x05
	Stop           StepperCommand = 0x06
	Reset          StepperCommand = 0x07
)

const (
	homePollDelay = time.Second
	posPollDelay  = time.Millisecond * 500
)

/*
Quick overview of protocol used to talk to stepper controllers

- Protocol is request(+response) based

- If no ping is received for more than 2 seconds, the controller will set the speed to zero and go into fault state, until a new ping is received

- Commands:
  - Ping: 0x00
    cmd -> cmd + pong
    0x00 -> 0x00 0x42
  - SetMotor: 0x01
    cmd + speed (float32) + disabled (bool)
    0x01 + 0x14 0xae 0x29 0x42 + 0x00
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
	startErr  error
	done      chan struct{}

	blocked *atomic.Bool
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
		blocked:   atomic.NewBool(false),
		done:      make(chan struct{}),
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
		select {
		case <-s.Done():
			return
		default:
		}

		s.Lock()
		//TODO(timanema): Log errors
		if err := s.ping(); err != nil {
			fmt.Printf("ping error %v\n", err)
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
		s.startErr = err
		return errors.Wrap(err, "unable to open serial connection")
	}

	// Clear old data from buffer
	_, _ = ioutil.ReadAll(s.rwCloser)

	// Send ping
	if err := s.ping(); err != nil {
		s.startErr = err
		return errors.Wrap(err, "failed to start connection")
	}

	s.connReady.Store(true)

	// Reset motor
	if err := s.genericWriteRaw([]byte{Reset}); err != nil {
		s.startErr = err
		return errors.Wrap(err, "failed to start connection")
	}

	// Start watchdog loop
	go s.run()

	//TODO(timanema): Remove later
	go func() {
		c := time.NewTicker(time.Second)

		for {
			select {
			case <-s.done:
				return
			case <-c.C:
				fmt.Println(s.GetStatus())
			}
		}
	}()

	return nil
}

func (s *SerialStepper) Blocked() bool {
	return s.blocked.Load()
}

func (s *SerialStepper) Block(blocked bool) {
	s.blocked.Store(blocked)
}

func (s *SerialStepper) SetMotor(ctr *rcs.Joint, speed float32, reverse bool) (error, int32) {
	return s.BlockSetMotor(ctr, speed, reverse, false)
}

func (s *SerialStepper) BlockSetMotor(ctr *rcs.Joint, speed float32, reverse, block bool) (error, int32) {
	return s.IgnoreBlockSetMotor(ctr, speed, reverse, block, false)
}

func (s *SerialStepper) IgnoreBlockSetMotor(ctr *rcs.Joint, speed float32, reverse, block, ignore bool) (error, int32) {
	s.Lock()
	defer s.Unlock()

	if !s.connReady.Load() {
		// Avoid returning nil when there is no error yet, but actually not ready
		if s.startErr != nil {
			return errors.Wrap(s.startErr, "connection not yet ready"), 0
		} else {
			return errors.New("connection not yet ready"), 0
		}
	}

	// If the motor is blocked, ignore the set request
	if s.blocked.Load() && !ignore {
		return nil, 0
	}

	// Block motor if required
	if block {
		s.blocked.Store(block)
	}

	// Reverse
	if reverse {
		ctr.Position *= -1
	}

	speedBuf := make([]byte, 8)
	binary.LittleEndian.PutUint32(speedBuf[0:4], math.Float32bits(speed))
	binary.LittleEndian.PutUint32(speedBuf[4:8], uint32(ctr.Position))

	// Create request for controller
	buf := []byte{SetMotor}
	buf = append(buf, speedBuf...)
	if ctr.Disable {
		buf = append(buf, 0x1)
	} else {
		buf = append(buf, 0x0)
	}

	if _, err := s.rwCloser.Write(buf); err != nil {
		return errors.Wrap(err, "unable to write request to controller"), 0
	}

	return nil, int32(ctr.Position)
}

func (s *SerialStepper) WaitForPosition(pos int32, left bool) error {
	// Block until position is reached
	t := time.NewTicker(posPollDelay)
	for {
		select {
		case <-s.done:
			return HomingFailedErr
		case <-t.C:
		}

		status, err := s.GetStatus()
		if err != nil {
			return errors.Wrap(err, "failed to get status from controller for waiting")
		}

		// Return an error if the limit switch in that direction is active
		if (left && status.BlockedLeft) || (!left && status.BlockedRight) {
			return HomingOffsetBlockedErr
		}

		//TODO(timanema): check if 0 is safe, fp is generally not, but zero is likely special

		// Return an error if the speed has been set to zero (possible limit switch hit
		if status.Speed == 0 {
			return HomingOffsetSpeedErr
		}

		// Return if the position has been reached
		if status.Location == pos {
			return nil
		}
	}
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
		return srv_rcs.Status{}, errors.Wrap(err, "unable to write get motor status request to controller")
	}

	buf := make([]byte, 10)
	n, err := s.rwCloser.Read(buf)
	if err != nil {
		return srv_rcs.Status{}, errors.Wrap(err, "unable to read response from controller")
	}

	if n != 10 || buf[0] != GetMotorStatus {
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
		Homing:       (buf[1] & 0x40) != 0x00,
		Homed:        (buf[1] & 0x80) != 0x00,
		Speed:        math.Float32frombits(binary.LittleEndian.Uint32(buf[2:6])),
		Location:     int32(binary.LittleEndian.Uint32(buf[6:10])),
	}, nil
}

func (s *SerialStepper) StartHoming(speed float32) error {
	if !s.connReady.Load() {
		// Avoid returning nil when there is no error yet, but actually not ready
		if s.startErr != nil {
			return errors.Wrap(s.startErr, "connection not yet ready")
		} else {
			return errors.New("connection not yet ready")
		}
	}

	// Block the connection until homing is done, or cancelled
	s.blocked.Store(true)
	defer s.blocked.Store(false)

	// Reset motor
	if err := s.genericWriteRaw([]byte{Reset}); err != nil {
		s.startErr = err
		return errors.Wrap(err, "failed to reset motor before homing")
	}

	buf := make([]byte, 5)
	buf[0] = StartHoming

	binary.LittleEndian.PutUint32(buf[1:5], math.Float32bits(speed))

	s.Lock()
	_, err := s.rwCloser.Write(buf)
	s.Unlock()

	if err != nil {
		return errors.Wrap(err, "unable to write start homing request to controller")
	}

	// Block until homing is done or failed
	t := time.NewTicker(homePollDelay)
	for {
		select {
		case <-s.done:
			return HomingFailedErr
		case <-t.C:
		}

		status, err := s.GetStatus()
		if err != nil {
			return errors.Wrap(err, "failed to get status from controller for homing")
		}

		// Continue if still homing
		if status.Homing {
			continue
		}

		// If done homing, return error if not homed, otherwise done
		if !status.Homed {
			return HomingFailedErr
		}

		return nil
	}
}

func (s *SerialStepper) StopHoming() error {
	s.Lock()
	defer s.Unlock()
	defer s.blocked.Store(false)

	if !s.connReady.Load() {
		// Avoid returning nil when there is no error yet, but actually not ready
		if s.startErr != nil {
			return errors.Wrap(s.startErr, "connection not yet ready")
		} else {
			return errors.New("connection not yet ready")
		}
	}

	if _, err := s.rwCloser.Write([]byte{StopHoming}); err != nil {
		return errors.Wrap(err, "unable to write stop homing request to controller")
	}

	if _, err := s.rwCloser.Write([]byte{Stop}); err != nil {
		return errors.Wrap(err, "unable to write stop (homing) request to controller")
	}

	return nil
}

func (s *SerialStepper) Zero() error {
	s.Lock()
	defer s.Unlock()

	return errors.Wrap(s.genericWriteRaw([]byte{Zero}), "unable to zero motor")
}

func (s *SerialStepper) Stop() error {
	s.Lock()
	defer s.Unlock()

	return errors.Wrap(s.genericWriteRaw([]byte{Stop}), "unable to stop motor")
}

func (s *SerialStepper) Reset() error {
	s.Lock()
	defer s.Unlock()

	return errors.Wrap(s.genericWriteRaw([]byte{Reset}), "unable to reset motor")
}

func (s *SerialStepper) genericWriteRaw(buf []byte) error {
	if !s.connReady.Load() {
		// Avoid returning nil when there is no error yet, but actually not ready
		if s.startErr != nil {
			return errors.Wrap(s.startErr, "connection not yet ready")
		} else {
			return errors.New("connection not yet ready")
		}
	}

	if _, err := s.rwCloser.Write(buf); err != nil {
		return errors.Wrap(err, "unable to write request to controller")
	}

	return nil
}

func (s *SerialStepper) Ready() bool {
	return s.connReady.Load()
}

func (s *SerialStepper) Done() <-chan struct{} {
	return s.done
}

func (s *SerialStepper) Close() error {
	// If the connection isn't even ready, there is nothing to clean up
	if !s.connReady.Load() {
		return nil
	}

	s.connReady.Store(false)
	close(s.done)
	return errors.Wrap(s.rwCloser.Close(), "unable to close serial connection")
}
