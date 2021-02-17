package pins

import (
	"github.com/pkg/errors"
	"github.com/stianeikeland/go-rpio/v4"
	"go.uber.org/atomic"
	"sync"
	"time"
)

const (
	PowerButtonLed   = rpio.Pin(24)
	StartButtonLed   = rpio.Pin(25)
	PauseButtonLed   = rpio.Pin(1)
	PowerButton      = rpio.Pin(21)
	StartButton      = rpio.Pin(16)
	PauseButton      = rpio.Pin(20)
	EmergencyFlag    = rpio.Pin(18)
	MotorRelay       = rpio.Pin(22)
	ServoRelay       = rpio.Pin(0)
	InputVerifyIn    = rpio.Pin(26)
	Motor1VerifyIn   = rpio.Pin(19)
	Motor2VerifyIn   = rpio.Pin(13)
	Motor3VerifyIn   = rpio.Pin(6)
	ServoVerifyIn    = rpio.Pin(5)
	WatchdogResetPin = rpio.Pin(23)
)

const (
	HintInterval       = time.Second
	ErrInterval        = time.Millisecond * 80
	ErrDuration        = time.Millisecond * 800
	PollInterval       = time.Millisecond * 100
	DebounceDelay      = time.Millisecond * 200
	WatchdogPulseDelay = time.Millisecond * 10
)

// WARNING: this might make the Pi unresponsive, add `dtoverlay=gpio-no-irq`
// to `/boot/config.txt` and restart the PI.
type PinHandler struct {
	blinking  map[rpio.Pin]chan struct{}
	blinkLock sync.RWMutex

	detecting  map[rpio.Pin]chan struct{}
	detectLock sync.RWMutex

	open  *atomic.Bool
	close chan struct{}
}

func NewPinHandler() *PinHandler {
	return &PinHandler{
		blinking:  make(map[rpio.Pin]chan struct{}),
		detecting: make(map[rpio.Pin]chan struct{}),
		close:     make(chan struct{}, 1),
		open:      atomic.NewBool(false),
	}
}

func (b *PinHandler) PrepareInterface() error {
	if err := rpio.Open(); err != nil {
		return errors.Wrap(err, "failed to access gpio memory")
	}
	b.open.Store(true)

	// Prepare buttons
	b.prepareInputPullUp(PowerButton)
	b.prepareInputPullUp(StartButton)
	b.prepareInputPullUp(PauseButton)

	// Prepare LEDs
	b.prepareOutput(PowerButtonLed)
	b.prepareOutput(StartButtonLed)
	b.prepareOutput(PauseButtonLed)

	// Prepare emergency input
	b.prepareInputPullDown(EmergencyFlag)

	// Prepare monitoring inputs
	b.prepareInputPullDown(InputVerifyIn)
	b.prepareInputPullDown(Motor1VerifyIn)
	b.prepareInputPullDown(Motor2VerifyIn)
	b.prepareInputPullDown(Motor3VerifyIn)
	b.prepareInputPullDown(ServoVerifyIn)

	// Prepare relay output
	b.prepareOutput(MotorRelay)
	b.prepareOutput(ServoRelay)

	// Disable power to motors at init
	b.SetPin(MotorRelay, rpio.Low)
	b.SetPin(ServoRelay, rpio.Low)

	// Prepare watchdog reset output
	WatchdogResetPin.Output()
	WatchdogResetPin.Write(rpio.Low)

	return nil
}

func (b *PinHandler) prepareInputPullUp(pin rpio.Pin) {
	pin.Input()
	pin.PullUp()
}

func (b *PinHandler) prepareInputPullDown(pin rpio.Pin) {
	pin.Input()
	pin.PullDown()
}

func (b *PinHandler) prepareOutput(pin rpio.Pin) {
	pin.Output()
	pin.Write(rpio.High)
}

func (b *PinHandler) CancelDetect(pin rpio.Pin) {
	b.detectLock.Lock()
	defer b.detectLock.Unlock()

	// Check if pins are properly loaded
	if !b.open.Load() {
		return
	}

	pin.Detect(rpio.NoEdge)

	if _, ok := b.detecting[pin]; ok {
		close(b.detecting[pin])
		delete(b.detecting, pin)
	}
}

func (b *PinHandler) DetectEdge(pin rpio.Pin, edge rpio.Edge, cb func(), ledPins ...rpio.Pin) {
	b.CancelDetect(pin)

	// Check if pins are properly loaded
	if !b.open.Load() {
		return
	}

	b.detectLock.Lock()
	c := make(chan struct{})
	b.detecting[pin] = c
	b.detectLock.Unlock()

	pin.Detect(edge)

	ticker := time.NewTicker(PollInterval)

	for range ticker.C {
		select {
		case <-b.close:
			return
		case <-c:
			return
		default:
		}

		if pin.EdgeDetected() {
			// Debounce
			current := pin.Read()
			time.After(DebounceDelay)
			if current != pin.Read() {
				continue
			}

			// Prevent readings due to bouncing
			if current == rpio.High && edge == rpio.FallEdge {
				continue
			}
			if current == rpio.Low && edge == rpio.RiseEdge {
				continue
			}

			cb()

			for _, led := range ledPins {
				b.CancelBlink(led)
				b.SetPin(led, rpio.High)
			}
		}
	}
}

func (b *PinHandler) CancelBlink(pin rpio.Pin) {
	b.blinkLock.Lock()
	defer b.blinkLock.Unlock()

	// Check if pins are properly loaded
	if !b.open.Load() {
		return
	}

	if _, ok := b.blinking[pin]; ok {
		close(b.blinking[pin])
		delete(b.blinking, pin)

		pin.Write(rpio.High)
	}
}

func (b *PinHandler) BlinkPinIndeterminate(pin rpio.Pin, interval time.Duration) {
	b.BlinkPin(pin, interval, time.Nanosecond)
}

//TODO(timanema): Find reason freeze when interrupting a blink event
//TODO(timanema): Find out why this also freezes base node
func (b *PinHandler) BlinkPin(pin rpio.Pin, interval, duration time.Duration) {
	b.CancelBlink(pin)

	// Check if pins are properly loaded
	if !b.open.Load() {
		return
	}

	b.blinkLock.Lock()
	c := make(chan struct{})
	b.blinking[pin] = c
	b.blinkLock.Unlock()

	ticker := time.NewTicker(interval)
	timer := time.NewTimer(duration)

	// Stop timer if lower than 1ms, indeterminate blinking
	if duration < time.Millisecond && !timer.Stop() {
		<-timer.C
	}

	for range ticker.C {
		select {
		case <-b.close:
			return
		case <-c:
			return
		case <-timer.C:
			b.SetPin(pin, rpio.High)
			return
		default:
		}

		pin.Toggle()
	}
}

func (b *PinHandler) SetPin(pin rpio.Pin, state rpio.State) {
	b.CancelBlink(pin)

	// Check if pins are properly loaded
	if !b.open.Load() {
		return
	}

	pin.Write(state)
}

func (b *PinHandler) GetPin(pin rpio.Pin) rpio.State {
	// Check if pins are properly loaded
	if !b.open.Load() {
		//TODO(timanema): Returns low for now (a la C), but error is better. Do later
		return rpio.Low
	}

	return pin.Read()
}

func (b *PinHandler) PulsePin(pin rpio.Pin, highToLow bool, delay time.Duration) {
	if highToLow {
		b.SetPin(pin, rpio.High)
		time.Sleep(delay)
		b.SetPin(pin, rpio.Low)
	} else {
		b.SetPin(pin, rpio.Low)
		time.Sleep(delay)
		b.SetPin(pin, rpio.High)
	}
}

func (b *PinHandler) Close() error {
	close(b.close)

	// Disable future pin operations
	b.open.Store(false)

	// Disable all button lights
	PowerButtonLed.Write(rpio.Low)
	StartButtonLed.Write(rpio.Low)
	PauseButtonLed.Write(rpio.Low)

	// Disable all relays
	MotorRelay.Write(rpio.Low)
	ServoRelay.Write(rpio.Low)

	// Close rpio memory
	return errors.Wrap(rpio.Close(), "failed to close gpio memory")
}
