package path_solver

import (
	"boxbot-go/internal/boxbot"
	"boxbot-go/pkg/ik_solver"
	"github.com/pkg/errors"
	"math"
)

var NoPathErr = errors.New("no path for given solution exists")
var NoAccelPathErr = errors.New("no accel path within limits exists")

func requiredTimeAccelToMax(dist, vel, accelDist, accelTime float64) float64 {
	// 0 -> accelDist: accel
	// accelDist -> dist - accelDist: vel
	// dist - accelDist -> dist: accel

	// Calculate time to travel at max velocity
	t := (dist - 2*accelDist) / vel

	return accelTime*2 + t
}

func requiredTimeAccel(dist, accel float64) float64 {
	// t = sqrt((2s)/a)
	return 2 * math.Sqrt(dist/accel)
}

// Returns the required time in seconds for the operation
func requiredTime(dist, vel, accel, accelDist, accelTime float64) float64 {
	dist = math.Abs(dist)

	// Check if the maximum velocity can be reached
	if accelDist*2 < dist+0.00001 {
		return requiredTimeAccelToMax(dist, vel, accelDist, accelTime)
	}

	// The max velocity cannot be reached, so return the amount of time it takes to accel and decel
	return requiredTimeAccel(dist, accel)
}

// estimatedTime returns the estimated time of the action, based on the maximum velocities and accelerations
func estimatedTime(arm, base, servo, tower float64) float64 {
	armT := requiredTime(arm, boxbot.MaxVelocityArm, boxbot.MaxAccelArm, boxbot.AccelDistArm, boxbot.AccelTimeArm)
	baseT := requiredTime(base, boxbot.MaxVelocityBase, boxbot.MaxAccelBase, boxbot.AccelDistBase, boxbot.AccelTimeBase)
	servoT := requiredTime(servo, boxbot.MaxVelocityServo, boxbot.MaxAccelServo, boxbot.AccelDistServo, boxbot.AccelTimeServo)
	towerT := requiredTime(tower, boxbot.MaxVelocityTower, boxbot.MaxAccelTower, boxbot.AccelDistTower, boxbot.AccelTimeTower)

	max := math.Max(armT, baseT)
	max = math.Max(max, servoT)
	return math.Max(max, towerT)
}

// FindSimplePath returns a SimplePath between the start and target Solutions, if it exists.
// This is a simple path, that will not necessarily be smooth, but it will just ensure the
// arm, base, servo, and tower, arrive at their destination around the same time.
func FindSimplePath(start, target ik_solver.JointState) (SimplePath, error) {
	// Fail if one of the solution is not valid
	if !start.Valid() || !target.Valid() {
		return SimplePath{}, NoPathErr
	}

	// Find deltas
	dArm := target.ArmLength - start.ArmLength
	dBase := target.BaseAngleRad - start.BaseAngleRad
	dServo := target.ServoAngleDegrees - start.ServoAngleDegrees
	dTower := target.TowerHeight - start.TowerHeight

	// Get estimated time
	t := estimatedTime(dArm, dBase, dServo, dTower)

	// Find accel maps
	armMap, err := findSimpleAccelMap(dArm, boxbot.MaxVelocityArm, boxbot.MaxAccelArm, boxbot.AccelDistArm, boxbot.AccelTimeArm, t)
	if err != nil {
		return SimplePath{}, errors.Wrap(err, "unable to find accel map for arm")
	}

	baseMap, err := findSimpleAccelMap(dBase, boxbot.MaxVelocityBase, boxbot.MaxAccelBase, boxbot.AccelDistBase, boxbot.AccelTimeBase, t)
	if err != nil {
		return SimplePath{}, errors.Wrap(err, "unable to find accel map for base")
	}

	servoMap, err := findSimpleAccelMap(dServo, boxbot.MaxVelocityServo, boxbot.MaxAccelServo, boxbot.AccelDistServo, boxbot.AccelTimeServo, t)
	if err != nil {
		return SimplePath{}, errors.Wrap(err, "unable to find accel map for servo")
	}

	towerMap, err := findSimpleAccelMap(dTower, boxbot.MaxVelocityTower, boxbot.MaxAccelTower, boxbot.AccelDistTower, boxbot.AccelTimeTower, t)
	if err != nil {
		return SimplePath{}, errors.Wrap(err, "unable to find accel map for tower")
	}

	// Return result
	return SimplePath{
		Duration: t,
		Target:   target,
		ArmMap:   newVelMap(armMap, boxbot.CreepVelocityArm),
		BaseMap:  newVelMap(baseMap, boxbot.CreepVelocityBase),
		ServoMap: newVelMap(servoMap, boxbot.CreepVelocityServo),
		TowerMap: newVelMap(towerMap, boxbot.CreepVelocityTower),
	}, nil
}
