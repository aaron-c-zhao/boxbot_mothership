package carton_search

import (
	"errors"
	"fmt"
	"io"
	"time"
)

var NoCartonsFound = errors.New("not enough cartons could be found")

type Coordinate struct {
	X, Y, Z, Theta float64
}

type CartonSearcher interface {
	io.Closer
	Search(cb func(coordinates []Coordinate, err error)) error
}

//TODO(timanema): Implement
type opencv struct {
}

func New() CartonSearcher {
	return &opencv{}
}

func (o *opencv) Search(cb func(coordinates []Coordinate, err error)) error {
	go func() {
		time.Sleep(time.Second * 3)
		fmt.Println("cv searched")

		cb([]Coordinate{{
			X:     0.0,
			Y:     0.0,
			Z:     0.0,
			Theta: 0,
		}, {
			X:     0.1,
			Y:     0.1,
			Z:     0.1,
			Theta: 1,
		}, {
			X:     0.2,
			Y:     0.2,
			Z:     0.2,
			Theta: 2,
		}, {
			X:     0.3,
			Y:     0.3,
			Z:     0.3,
			Theta: 3,
		}, {
			X:     0.4,
			Y:     0.4,
			Z:     0.4,
			Theta: 4,
		}, {
			X:     0.5,
			Y:     0.5,
			Z:     0.5,
			Theta: 5,
		}}, nil)
	}()
	return nil
}

func (o *opencv) Close() error {
	return nil
}
