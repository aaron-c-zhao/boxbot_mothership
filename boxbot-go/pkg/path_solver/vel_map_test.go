package path_solver

import (
	"fmt"
	"testing"
	"time"
)

func TestPlotGraphVelMap(t *testing.T) {
	// Only manual testing allowed
	t.Skip("Plotting is strictly manual, not automated")

	dur := 4.0
	rate := 50
	dis, vel, accel := 10.0, 5.0, 4.5
	accelTime := vel / accel
	accelDist := 0.5 * accel * accelTime * accelTime

	m, err := findSimpleAccelMap(dis, vel, accel, accelDist, accelTime, dur)
	v := newVelMap(m, 0.1)

	fmt.Printf("Err from find accel path: %v\n", err)
	fmt.Printf("Sampled distance based on path (%d Hz): %v\n", rate, v.findDistance(rate))

	v = newVelMap(m, 0.1)
	plotSimulatedVel(v, rate, 0, dis, dur)

	v = newVelMap(m, 0.1)
	plotRealtimeVel(v, rate, 0, dis, dur)
}

func plotSimulatedVel(vm *VelMap, rate int, minY, maxY, dur float64) {
	r := 1.0 / float64(rate)
	p := 0.0
	rs := make([]float64, 0, int(vm.accelMap.duration/r))
	pos := make([]float64, 0, int(vm.accelMap.duration/r))
	v := make([]float64, 0, int(vm.accelMap.duration/r))

	for t := 0.0; t < vm.accelMap.duration; t += r {
		vel := vm.GetVel(t)
		p += vel * r

		pos = append(pos, p)
		v = append(v, vel)
		rs = append(rs, t)
	}

	plot(rs, pos, make([]float64, int(vm.accelMap.duration/r)), v, "vel_map_simulated.png", minY, maxY, dur)
}

func plotRealtimeVel(vm *VelMap, rate int, minY, maxY, dur float64) {
	delay := time.Second / time.Duration(rate)

	tm := time.NewTicker(delay)
	t := time.Now()
	prev := t

	r := 1.0 / float64(rate)
	p := 0.0
	rs := make([]float64, 0, int(vm.accelMap.duration/r))
	pos := make([]float64, 0, int(vm.accelMap.duration/r))
	v := make([]float64, 0, int(vm.accelMap.duration/r))

	for t := 0.0; t < vm.accelMap.duration; t += r {
		now := <-tm.C
		diff := float64(now.Sub(prev)-delay) / float64(time.Second)
		prev = now

		vel := vm.GetVel(t + diff)
		p += vel * r

		pos = append(pos, p)
		v = append(v, vel)
		rs = append(rs, t+diff)
	}

	plot(rs, pos, make([]float64, int(vm.accelMap.duration/r)), v, "vel_map_realtime.png", minY, maxY, dur)
}
