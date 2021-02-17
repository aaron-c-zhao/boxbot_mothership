package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("watchdog", NewNode); err != nil {
		log.Fatalf("failed to run watchdog node: %v\n", err)
	}
}
