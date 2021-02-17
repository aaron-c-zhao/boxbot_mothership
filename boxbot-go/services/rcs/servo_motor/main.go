package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("servo", NewNode); err != nil {
		log.Fatalf("failed to run servo node: %v\n", err)
	}
}
