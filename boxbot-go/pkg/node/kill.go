package node

import (
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"log"
)

func KillNodes(n *goroslib.Node, names []string) error {
	var err error
	for _, name := range names {
		if err2 := n.KillNode(name); err2 != nil {
			log.Printf("failed to kill %s: %v", name, err2)
			err = err2
		}
	}

	return errors.Wrap(err, "failed to kill all nodes")
}
