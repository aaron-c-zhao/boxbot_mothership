package node

import (
	"fmt"
	"os"
	"os/signal"
	"syscall"
)

func SignalIntercept(n Node) {
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)

	select {
	case <-c:
	case <-n.Done():
	}

	if err := n.Close(); err != nil {
		fmt.Printf("failed to exit cleanly, got err: %v\n", err)
		os.Exit(1)
	}
}
