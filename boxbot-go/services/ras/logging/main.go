package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("logging", NewNode); err != nil {
		log.Fatalf("failed to run logging node: %v\n", err)
	}
}
