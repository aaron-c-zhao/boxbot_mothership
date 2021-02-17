package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("autonomous", NewNode); err != nil {
		log.Fatalf("failed to run autonomous node: %v\n", err)
	}
}
