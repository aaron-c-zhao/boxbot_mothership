package path_solver

import "math"

type accelPoint struct {
	time  float64
	accel float64
}

type AccelMap struct {
	duration float64
	points   []accelPoint
}

func (a AccelMap) GetAccel(time float64) float64 {
	for i, p := range a.points {
		// Check if this is the last entry
		if len(a.points) == i+1 {
			return p.accel
		}

		// If this belongs in the next range (or further), continue
		if time > a.points[i+1].time {
			continue
		}

		return p.accel
	}

	return 0
}

func simpleAccelMap(vel, accel, t float64) AccelMap {
	return AccelMap{
		duration: t,
		points: []accelPoint{
			{
				time:  0,
				accel: accel,
			},
			{
				time:  math.Abs(vel / accel),
				accel: 0,
			},
			{
				time:  t - math.Abs(vel/accel),
				accel: -accel,
			},
			{
				time:  t,
				accel: 0,
			},
		},
	}
}

/*
Based on: https://math.stackexchange.com/questions/3304696/calculate-max-velocity-given-time-distance-and-acceleration
*/
func findSimpleAccelMap(dist, maxVel, maxAccel, accelDist, accelTime, time float64) (AccelMap, error) {
	// Check if the path exceeds limits
	if requiredTime(dist, maxVel, maxAccel, accelDist, accelTime) > time+0.000001 {
		return AccelMap{}, NoAccelPathErr
	}

	// Account for floating point accuracy and negatives
	maxAccel += 0.00001
	if dist < 0 {
		maxAccel *= -1
	}

	// Check if distance is negligible, and return no movement if so
	if dist < 0.00001 && dist > -0.00001 {
		return AccelMap{
			duration: time,
			points: []accelPoint{
				{
					time:  0,
					accel: 0,
				},
			},
		}, nil
	}

	a := 2.0 / maxAccel
	b := -2.0 * time
	c := 2.0 * dist

	vel1, vel2 := quadratic(a, b, c)
	switch {
	case math.Abs(vel1) > maxVel && math.Abs(vel2) > maxVel:
		return AccelMap{}, NoAccelPathErr
	case math.Abs(vel2) > maxVel:
		return simpleAccelMap(vel1, maxAccel, time), nil
	default:
		return simpleAccelMap(vel2, maxAccel, time), nil
	}
}

func quadratic(a, b, c float64) (float64, float64) {
	negB := -b
	twoA := 2 * a
	bSquared := b * b
	fourAC := 4 * a * c
	discrim := bSquared - fourAC
	sq := math.Sqrt(discrim)
	return (negB + sq) / twoA, (negB - sq) / twoA
}

func findDistance(m AccelMap) float64 {
	pos := 0.0
	vel := 0.0
	time := 0.0
	prevAccel := 0.0
	for _, p := range m.points {
		dT := p.time - time
		pos += vel*dT + prevAccel*dT*dT*0.5
		vel += dT * prevAccel
		prevAccel = p.accel
		time = p.time
	}

	return pos
}

func findDistanceSampling(m AccelMap, rate int) float64 {
	r := 1.0 / float64(rate)
	pos := 0.0
	vel := 0.0
	prevAccel := 0.0

	for t := 0.0; t < m.duration; t += r {
		pos += vel*r + prevAccel*r*r*0.5
		vel += r * prevAccel
		prevAccel = m.GetAccel(t)
	}

	return pos
}
