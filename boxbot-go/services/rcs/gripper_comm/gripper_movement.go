package main

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
	x, y, z := 0.0, 0.1, 0.2

	return g.moveTo(x+dir.offsetX(g.c.DeltaXSearch),
		y+dir.offsetY(g.c.DeltaYSearch), z)
}

func (g *gripper) moveTo(x, y, z float64) error {
	return nil
}
