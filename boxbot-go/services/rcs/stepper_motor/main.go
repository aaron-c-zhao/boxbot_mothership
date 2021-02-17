package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("stepper", NewNode); err != nil {
		log.Fatalf("failed to run stepper node: %v\n", err)
	}
}
