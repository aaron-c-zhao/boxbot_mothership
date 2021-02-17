package main

import (
	"boxbot-go/pkg/ik_solver"
	"fmt"
	"os"
	"os/signal"
	"syscall"
)

func main() {
	s, err := ik_solver.FindSolution(ik_solver.Coordinate{
		X: 20,
		Y: 20,
		Z: 50,
	})
	if err != nil {
		fmt.Printf("ik solver err: %v\n", err)
		os.Exit(1)
	}

	c := make(chan os.Signal, 1)
	stopCh := make(chan struct{})
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)

	control := NewSimpleControl(s, stopCh, &RobotTransmissions)

	go func() {
		control.Start()
	}()

	go func() {
		err := control.SetTarget(ik_solver.Coordinate{
			X:     -25,
			Y:     30,
			Z:     70,
			Theta: 5,
		})
		if err != nil {
			fmt.Printf("target error: %v\n", err)
			os.Exit(1)
		}
	}()

	select {
	case <-c:
	}

	close(stopCh)

	<-control.Done()
}
