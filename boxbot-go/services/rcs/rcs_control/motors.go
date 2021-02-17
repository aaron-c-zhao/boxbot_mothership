package main

import "boxbot-go/internal/boxbot"

type MotorType int

const (
	ArmMotor MotorType = iota
	BaseMotor
	GripperServo
	TowerMotor
)

var RobotTransmissions = TransmissionMap{
	ArmMotor:     boxbot.ArmTransmission,
	BaseMotor:    boxbot.BaseTransmission,
	GripperServo: boxbot.ServoTransmission,
	TowerMotor:   boxbot.TowerTransmission,
}

type TransmissionMap map[MotorType]float64

func (t *TransmissionMap) Verify() {
	t.verify(ArmMotor)
	t.verify(BaseMotor)
	t.verify(GripperServo)
	t.verify(TowerMotor)
}

func (t *TransmissionMap) verify(m MotorType) {
	if _, ok := (*t)[m]; !ok {
		(*t)[m] = 1.0
	}
}
