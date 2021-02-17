package gripper_solver

import (
	"math"
)

// p = breed cart
// q = hoog cart

// Constants:
/*
X1, Y1 = end effector
phi =
psi =
theta = phi - psi


IR sens: a, b
r: ... (ir to mid)
d: ... (delta ir)

ax = X1 + cos(theta) * r + sin (theta) * .5d
....

*/

/*
sensor a (lower) -> B, A
sensor b (upper) -> C

A, B, C given (B + C on same line)
H hoek


R = (Bx-Cx)/(By-Cy)
*/

const (
	distanceIrLineToMidLine float64 = 0.23547 // 208.22 + 2.75 + (216 - 191.5) = 235.47mm
	distanceBetweenIr       float64 = 0.149   // 149mm
	cartonWidth             float64 = 0.355   // 355mm (p)
	cartonLength            float64 = 0.473   // 473mm (q)
	magicConstant           float64 = 0.02    // ~2cm estimate
)

func FindCoordinateIrSensor(center Point, servoAngle float64, lower bool) Point {
	mod := 1.0
	if !lower {
		mod *= -1.0
	}

	theta := servoAngle //phi - psi

	x := center.X + math.Cos(theta)*distanceIrLineToMidLine + math.Sin(theta)*0.5*distanceBetweenIr*mod
	y := center.Y + math.Sin(theta)*distanceIrLineToMidLine - math.Cos(theta)*0.5*distanceBetweenIr*mod
	return Point{
		X: x,
		Y: y,
	}
}

type Point struct {
	X, Y float64
}

// A = lower edge
// B & C = side edges
func FindCorner(A, B, C Point) (Point, float64) {
	R := (B.X - C.X) / (B.Y - C.Y)

	//TODO(timanema): floating point
	if B.X-C.X == 0 {
		return Point{
			X: B.X,
			Y: A.Y,
		}, math.MaxFloat64
	}

	x := (A.Y + R*A.X - B.Y + (B.X / R)) / ((1.0 / R) + R)

	y1 := -R*x + A.Y + R*A.X
	y2 := x/R + B.Y - B.X/R
	y3 := x/R + C.Y - C.X/R
	y := (y1 + y2 + y3) / 3.0

	return Point{
		X: x,
		Y: y,
	}, R
}

func FindMiddle(R float64, H Point) (Point, float64) {
	g := math.Atan(-R)

	x := H.X - math.Cos(g)*0.5*cartonWidth - math.Sin(g)*0.5*cartonLength
	y := H.Y - math.Sin(g)*0.5*cartonWidth + math.Cos(g)*0.5*cartonLength

	return Point{
		X: x,
		Y: y,
	}, g
}

func FindOffsetMiddle(R float64, m Point, B, C Point) Point {
	//TODO(timanema): floating point
	if B.X-C.X == 0 {
		return Point{
			X: m.X - magicConstant,
			Y: m.Y,
		}
	}

	x := (m.X*R*R + m.X - math.Sqrt((R*R+1)*magicConstant*magicConstant)) / (R*R + 1)
	y := -R*x + m.Y + R*m.X

	return Point{
		X: x,
		Y: y,
	}
}
