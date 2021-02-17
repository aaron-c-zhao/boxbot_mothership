package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("base", NewNode); err != nil {
		log.Fatalf("failed to run base node: %v\n", err)
	}
}
