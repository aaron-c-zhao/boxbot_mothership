package path_solver

import (
	"boxbot-go/pkg/ik_solver"
	"github.com/stretchr/testify/assert"
	"github.com/wcharczuk/go-chart/v2"
	"os"
	"testing"
	"time"
)

func TestPlotSimplePath(t *testing.T) {
	// Only manual testing allowed
	//t.Skip("Plotting is strictly manual, not automated")

	start, err := ik_solver.FindSolution(ik_solver.Coordinate{
		X:     50,
		Y:     50,
		Z:     80,
		Theta: 0,
	})
	assert.NoError(t, err)

	end, err := ik_solver.FindSolution(ik_solver.Coordinate{
		X:     -50,
		Y:     50,
		Z:     30,
		Theta: 0,
	})
	assert.NoError(t, err)

	p, err := FindSimplePath(start, end)
	assert.NoError(t, err)

	plotSimulatedJointVelocities(p, 50)
}

func plotJointVelocities(t []float64, vels [][]float64, joints []string, time float64) {
	series := make([]chart.Series, 0, len(vels))
	for i, vel := range vels {
		series = append(series, chart.ContinuousSeries{
			Name:    joints[i],
			XValues: t,
			YValues: vel,
		})
	}

	graph := chart.Chart{
		XAxis: chart.XAxis{
			Name: "Time (s)",
			Range: &chart.ContinuousRange{
				Min: 0.0,
				Max: time,
			},
		},
		Series: series,
	}

	graph.Elements = []chart.Renderable{
		chart.Legend(&graph),
	}

	f, _ := os.Create("joint_velocities.png")
	defer f.Close()
	graph.Render(chart.PNG, f)
}

func plotSimulatedJointVelocities(path SimplePath, rate int) {
	delay := time.Second / time.Duration(rate)

	tm := time.NewTicker(delay)
	t := time.Now()
	prev := t

	r := 1.0 / float64(rate)
	rs := make([]float64, 0, int(path.Duration/r))
	vels := make([][]float64, 4)
	for i := 0; i < 4; i++ {
		v := make([]float64, 0, int(path.Duration/r))
		vels[i] = v
	}

	for t := 0.0; t < path.Duration; t += r {
		now := <-tm.C
		diff := float64(now.Sub(prev)-delay) / float64(time.Second)
		prev = now

		vels[0] = append(vels[0], path.ArmMap.GetVel(t+diff))
		vels[1] = append(vels[1], path.BaseMap.GetVel(t+diff))
		vels[2] = append(vels[2], path.ServoMap.GetVel(t+diff))
		vels[3] = append(vels[3], path.TowerMap.GetVel(t+diff))
		rs = append(rs, t+diff)
	}

	plotJointVelocities(rs, vels, []string{"Arm (m/s)", "Base (rad/s)", "Servo (degree/s)", "Tower (m/s)"}, path.Duration)
}
