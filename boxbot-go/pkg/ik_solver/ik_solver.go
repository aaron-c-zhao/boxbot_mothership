package ik_solver

import (
	"boxbot-go/internal/boxbot"
	"github.com/pkg/errors"
	"math"
)

var NoSolutionErr = errors.New("no solution for given coordinate exists")

type Coordinate struct {
	X, Y, Z, Theta float64
}

type JointState struct {
	BaseAngleRad, ArmLength, TowerHeight, ServoAngleDegrees float64
}

// TODO(timanema): Maybe allow for some small margins, due to FP-accuracy

// Valid returns true if the current JointState is viable
func (s JointState) Valid() bool {
	if s.ArmLength < boxbot.ArmLengthOffset || s.ArmLength > boxbot.ArmLength {
		return false
	}

	if s.TowerHeight < boxbot.TowerMin || s.TowerHeight > boxbot.TowerMax {
		return false
	}

	if s.BaseAngleRad < boxbot.BaseMinAngle || s.BaseAngleRad > boxbot.BaseMaxAngle {
		return false
	}

	return s.ServoAngleDegrees >= boxbot.ServoMinAngle && s.ServoAngleDegrees <= boxbot.ServoMaxAngle
}

func radToDeg(r float64) float64 {
	return r * (180.0 / math.Pi)
}

// Returns the normalized coordinate, which means it's rotated 90 degrees clockwise
func normalizeCoordinate(x, y float64) (float64, float64) {
	// x' = x * cos(90) + y * sin(90) = y
	// y' = -x * sin(90) + y * cos(90) = -x
	return y, -x
}

func findPhiOffset(c Coordinate) float64 {
	a := math.Sqrt(c.X*c.X + c.Y*c.Y)
	beta := math.Acos(boxbot.ArmBaseSideOffset / a)

	return math.Pi/2 - beta
}

func FindSolution(c Coordinate) (JointState, error) {
	// Find base rotation without offset
	nX, nY := normalizeCoordinate(c.X, c.Y)
	phi := -math.Atan2(nY, nX) // -1* since our defined rotation is the the inverse of normal

	// Adjust for offset
	phi -= findPhiOffset(c)

	// Calculate arm length
	a := c.X*c.X + c.Y*c.Y
	l := math.Sqrt(a - boxbot.ArmBaseSideOffset*boxbot.ArmBaseSideOffset)

	// Calculate servo rotation
	s := boxbot.ServoHoldAngle + radToDeg(phi) - c.Theta

	// Return solution, after checking its validity
	sol := JointState{
		BaseAngleRad:      phi,
		ArmLength:         l,
		TowerHeight:       c.Z,
		ServoAngleDegrees: s,
	}

	if !sol.Valid() {
		return JointState{}, NoSolutionErr
	}

	return sol, nil
}
