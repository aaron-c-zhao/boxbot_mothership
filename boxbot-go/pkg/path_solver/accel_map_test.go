package path_solver

import (
	"fmt"
	"github.com/stretchr/testify/assert"
	"github.com/wcharczuk/go-chart/v2"
	"os"
	"testing"
	"time"
)

func TestRequiredTimeMaxVelocity(t *testing.T) {
	dis, vel, accel := 10.0, 3.0, 1.0
	accelTime := vel / accel
	accelDist := 0.5 * accel * accelTime * accelTime

	assert.InDelta(t, 6+1.0/3.0, requiredTime(dis, vel, accel, accelDist, accelTime), 0.001)
}

func TestRequiredTimeAccel(t *testing.T) {
	dis, vel, accel := 5.0, 3.0, 1.0
	accelTime := vel / accel
	accelDist := 0.5 * accel * accelTime * accelTime

	assert.InDelta(t, 4.472135, requiredTime(dis, vel, accel, accelDist, accelTime), 0.001)
}

func TestPlotGraphAccelMap(t *testing.T) {
	// Only manual testing allowed
	t.Skip("Plotting is strictly manual, not automated")

	dur := 4.0
	rate := 50
	dis, vel, accel := 10.0, 5.0, 4.5
	accelTime := vel / accel
	accelDist := 0.5 * accel * accelTime * accelTime

	m, err := findSimpleAccelMap(dis, vel, accel, accelDist, accelTime, dur)
	fmt.Printf("Err from find accel path: %v\n", err)
	fmt.Printf("Perfect distance based on path: %v\n", findDistance(m))
	fmt.Printf("Sampled distance based on path (%d Hz): %v\n", rate, findDistanceSampling(m, rate))

	plotSimulatedAccel(m, rate, -accel, dis, dur)
	plotRealtimeAccel(m, rate, -accel, dis, dur)
}

func plot(rs, pos, acc, v []float64, name string, minY, maxY, time float64) {
	graph := chart.Chart{
		XAxis: chart.XAxis{
			Name: "Time (s)",
			Range: &chart.ContinuousRange{
				Min: 0.0,
				Max: time,
			},
		},
		YAxis: chart.YAxis{
			Range: &chart.ContinuousRange{
				Min: minY,
				Max: maxY,
			},
		},
		Series: []chart.Series{
			chart.ContinuousSeries{
				Name:    "Position (m)",
				XValues: rs,
				YValues: pos,
			},
			chart.ContinuousSeries{
				Name:    "Acceleration (m/s^2)",
				XValues: rs,
				YValues: acc,
			},
			chart.ContinuousSeries{
				Name:    "Velocity (m/s)",
				XValues: rs,
				YValues: v,
			},
		},
	}

	graph.Elements = []chart.Renderable{
		chart.Legend(&graph),
	}

	f, _ := os.Create(name)
	defer f.Close()
	graph.Render(chart.PNG, f)
}

func plotRealtimeAccel(m AccelMap, rate int, minY, maxY, dur float64) {
	delay := time.Second / time.Duration(rate)

	tm := time.NewTicker(delay)
	t := time.Now()
	prev := t

	r := 1.0 / float64(rate)
	p := 0.0
	rs := make([]float64, 0, int(m.duration/r))
	pos := make([]float64, 0, int(m.duration/r))
	acc := make([]float64, 0, int(m.duration/r))
	v := make([]float64, 0, int(m.duration/r))
	vel := 0.0
	prevAccel := 0.0

	for t := 0.0; t < m.duration; t += r {
		now := <-tm.C
		diff := float64(now.Sub(prev)-delay) / float64(time.Second)
		prev = now

		p += vel*r + prevAccel*r*r*0.5
		vel += r * prevAccel
		prevAccel = m.GetAccel(t + diff)
		pos = append(pos, p)
		acc = append(acc, prevAccel)
		v = append(v, vel)
		rs = append(rs, t+diff)
	}

	plot(rs, pos, acc, v, "accel_map_realtime.png", minY, maxY, dur)
}

func plotSimulatedAccel(m AccelMap, rate int, minY, maxY, dur float64) {
	r := 1.0 / float64(rate)
	p := 0.0
	rs := make([]float64, 0, int(m.duration/r))
	pos := make([]float64, 0, int(m.duration/r))
	acc := make([]float64, 0, int(m.duration/r))
	v := make([]float64, 0, int(m.duration/r))
	vel := 0.0
	prevAccel := 0.0

	for t := 0.0; t < m.duration; t += r {
		p += vel*r + prevAccel*r*r*0.5
		vel += r * prevAccel
		prevAccel = m.GetAccel(t)
		pos = append(pos, p)
		acc = append(acc, prevAccel)
		v = append(v, vel)
		rs = append(rs, t)
	}

	plot(rs, pos, acc, v, "accel_map_simulated.png", minY, maxY, dur)
}
