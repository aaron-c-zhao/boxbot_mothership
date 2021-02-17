package ik_solver

import (
	"github.com/stretchr/testify/assert"
	"math"
	"testing"
)

func TestToShortArm(t *testing.T) {
	_, err := FindSolution(Coordinate{
		X:     100,
		Y:     100,
		Z:     0,
		Theta: 0,
	})

	assert.Error(t, err)
}

func TestGripperInArm(t *testing.T) {
	_, err := FindSolution(Coordinate{
		X:     0,
		Y:     1,
		Z:     42,
		Theta: 50,
	})

	assert.Error(t, err)
}

func TestServoOverrotationLeft(t *testing.T) {
	_, err := FindSolution(Coordinate{
		X:     -50,
		Y:     -50,
		Z:     42,
		Theta: 0,
	})

	assert.Error(t, err)
}

func TestMaxRightWithServoNormal(t *testing.T) {
	s, err := FindSolution(Coordinate{
		X:     50,
		Y:     0,
		Z:     0,
		Theta: 0,
	})

	assert.NoError(t, err)
	assert.InDelta(t, 180, s.ServoAngleDegrees, 0.001)
	assert.InDelta(t, 90, radToDeg(s.BaseAngleRad), 0.001)
	assert.InDelta(t, 50, s.ArmLength, 0.001)
	assert.InDelta(t, 0, s.TowerHeight, 0.001)
}

func TestMaxRightWithServoRight(t *testing.T) {
	s, err := FindSolution(Coordinate{
		X:     0,
		Y:     -50,
		Z:     0,
		Theta: 90,
	})

	assert.NoError(t, err)
	assert.InDelta(t, 180, s.ServoAngleDegrees, 0.001)
	assert.InDelta(t, 180, radToDeg(s.BaseAngleRad), 0.001)
	assert.InDelta(t, 50, s.ArmLength, 0.001)
	assert.InDelta(t, 0, s.TowerHeight, 0.001)
}

func TestMaxLeftWithServoNormal(t *testing.T) {
	s, err := FindSolution(Coordinate{
		X:     -50,
		Y:     0,
		Z:     0,
		Theta: 0,
	})

	assert.NoError(t, err)
	assert.InDelta(t, 0, s.ServoAngleDegrees, 0.001)
	assert.InDelta(t, -90, radToDeg(s.BaseAngleRad), 0.001)
	assert.InDelta(t, 50, s.ArmLength, 0.001)
	assert.InDelta(t, 0, s.TowerHeight, 0.001)
}

func TestMaxLeftWithServoLeft(t *testing.T) {
	s, err := FindSolution(Coordinate{
		X:     -.0000001,
		Y:     -50,
		Z:     0,
		Theta: -90,
	})

	assert.NoError(t, err)
	assert.InDelta(t, 0, s.ServoAngleDegrees, 0.001)
	assert.InDelta(t, -180, radToDeg(s.BaseAngleRad), 0.001)
	assert.InDelta(t, 50, s.ArmLength, 0.001)
	assert.InDelta(t, 0, s.TowerHeight, 0.001)
}

func TestLeftUpperCornerServoNormal(t *testing.T) {
	s, err := FindSolution(Coordinate{
		X:     -50,
		Y:     50,
		Z:     50,
		Theta: 0,
	})

	assert.NoError(t, err)
	assert.InDelta(t, 45, s.ServoAngleDegrees, 0.001)
	assert.InDelta(t, -45, radToDeg(s.BaseAngleRad), 0.001)
	assert.InDelta(t, math.Sqrt(50*50+50*50), s.ArmLength, 0.001)
	assert.InDelta(t, 50, s.TowerHeight, 0.001)
}
