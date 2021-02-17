package gripper_solver

import (
	"fmt"
	"testing"
)

func TestStraightCarton(t *testing.T) {
	centerLocation := Point{
		X: 0,
		Y: 1.2,
	}

	highIR := FindCoordinateIrSensor(centerLocation, 0, false)
	lowIR := FindCoordinateIrSensor(centerLocation, 0, true)

	fmt.Println(highIR)
	fmt.Println(lowIR)
}

func TestStraightCartonCorner(t *testing.T) {
	A := Point{ // lower
		X: 0.3,
		Y: 0.2,
	}
	B := Point{
		X: 0.5,
		Y: 0.5,
	}
	C := Point{
		X: 0.1,
		Y: 0.8,
	}

	corner, _ := FindCorner(A, B, C)

	fmt.Println(corner)
}

func TestStraightCartonMiddle(t *testing.T) {
	A := Point{ // lower
		X: 0.3 - 1,
		Y: 0.2,
	}
	B := Point{
		X: 0.5 - 1,
		Y: 0.5,
	}
	C := Point{
		X: 0.2 - 1,
		Y: 0.8,
	}

	corner, R := FindCorner(A, B, C)

	fmt.Println(corner)

	middle, angle := FindMiddle(R, corner)

	fmt.Println(middle)
	fmt.Println(angle)
}

func TestStraightCartonMiddleWithOffset(t *testing.T) {
	A := Point{ // lower
		X: 0.3 - 1,
		Y: 0.2,
	}
	B := Point{
		X: 0.5 - 1,
		Y: 0.5,
	}
	C := Point{
		X: 0.2 - 1,
		Y: 0.8,
	}

	corner, R := FindCorner(A, B, C)

	fmt.Println(corner)

	middle, angle := FindMiddle(R, corner)

	fmt.Println(middle)
	fmt.Println(angle)

	offset := FindOffsetMiddle(R, middle, B, C)

	fmt.Println(offset)
}

func TestStraightCartonMiddleWithOffset2(t *testing.T) {
	A := Point{ // lower
		X: 0,
		Y: 0.2,
	}
	B := Point{
		X: cartonWidth/2 - 0.01,
		Y: 0.25,
	}
	C := Point{
		X: cartonWidth / 2,
		Y: 0.30,
	}

	corner, R := FindCorner(A, B, C)

	fmt.Println(corner)

	middle, angle := FindMiddle(R, corner)

	fmt.Println(middle)
	fmt.Println(angle)

	offset := FindOffsetMiddle(R, middle, B, C)

	fmt.Println(offset)
}
