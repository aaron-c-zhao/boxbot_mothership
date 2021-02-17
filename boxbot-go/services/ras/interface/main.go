package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("interface", NewNode); err != nil {
		log.Fatalf("failed to run interface node: %v\n", err)
	}
}
