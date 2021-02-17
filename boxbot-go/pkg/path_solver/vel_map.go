package path_solver

// Motors have 1.5 seconds to slowly move to the desired location, in case of speed undershoot
const CreepDuration = 1.5

type VelMap struct {
	accelMap            AccelMap
	lastTime, lastAccel float64
	curVel, creepVel    float64
}

func newVelMap(accelMap AccelMap, creepVel float64) *VelMap {
	return &VelMap{
		accelMap: accelMap,
		creepVel: creepVel,
	}
}

func (v *VelMap) GetVel(t float64) float64 {
	// If the current time is totally out of scope for this map, just return zero
	if t > v.accelMap.duration+CreepDuration {
		return 0
	}

	// If the time is higher than the duration of the movement, return the creep velocity
	// so the position based controllers have a chance to reach the final destination (in case of undershoot)
	if t > v.accelMap.duration {
		return v.creepVel
	}

	// Else calculate the delta time, and update some internal state
	dT := t - v.lastTime
	v.curVel += dT * v.lastAccel
	v.lastAccel = v.accelMap.GetAccel(t)
	v.lastTime = t

	return v.curVel
}

func (v *VelMap) findDistance(rate int) float64 {
	r := 1.0 / float64(rate)
	pos := 0.0

	for t := 0.0; t < v.accelMap.duration; t += r {
		vel := v.GetVel(t)
		pos += vel * r
	}

	return pos
}
