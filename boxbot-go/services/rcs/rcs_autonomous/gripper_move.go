package main

import (
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/pkg/errors"
)

type Direction int

const (
	LEFT Direction = iota
	RIGHT
	UP
	DOWN
)

func (d Direction) offsetX(dX float64) float64 {
	switch d {
	case LEFT:
		return -dX
	case RIGHT:
		return dX
	default:
		return 0
	}
}

func (d Direction) offsetY(dY float64) float64 {
	switch d {
	case UP:
		return dY
	case DOWN:
		return -dY
	default:
		return 0
	}
}

func (g *gripper) moveDirection(dir Direction) error {
	return g.moveRelative(dir.offsetX(g.c.DeltaXSearch), dir.offsetY(g.c.DeltaYSearch), 0, 0)
}

func (g *gripper) moveRelative(x, y, z, w float64) error {
	q, err := g.move.GetLocation()
	if err != nil {
		return errors.Wrap(err, "unable to retrieve location")
	}

	//TODO(Timanema): remove debug statement
	//fmt.Printf("location get: %v\n", q)

	return g.moveTo(geometry_msgs.Quaternion{
		X: q.X + x,
		Y: q.Y + y,
		Z: q.Z + z,
		W: q.W + w,
	})
}

func (g *gripper) moveTo(q geometry_msgs.Quaternion) error {
	errCh := make(chan error)
	err := g.move.Move(q, func(err error) {
		errCh <- err
	})
	if err != nil {
		return errors.Wrap(err, "unable to move")
	}

	err = <-errCh
	return errors.Wrap(err, "error while moving")
}
