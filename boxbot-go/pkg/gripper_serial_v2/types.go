package gripper_serial

type Direction byte

type Sensor byte

const (
	UpperIR Sensor = 0x01
	LowerIR Sensor = 0x00
)
