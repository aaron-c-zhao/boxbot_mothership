package gripper_serial

type Direction byte

const (
	Left                     Direction = 0x00
	Right                    Direction = 0x01
	Up                       Direction = 0x02
	Down                     Direction = 0x03
	Higher                   Direction = 0x04
	Lower                    Direction = 0x05
	None                     Direction = 0x06
	ClockwiseRotation        Direction = 0x07
	CounterClockwiseRotation Direction = 0x08
)

type Sensor byte

const (
	UpperIR Sensor = 0x00
	LowerIR Sensor = 0x01
)
