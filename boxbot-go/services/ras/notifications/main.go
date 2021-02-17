package main

import (
	"boxbot-go/pkg/node"
	"log"
)

func main() {
	if err := node.RunNode("notification", NewNode); err != nil {
		log.Fatalf("failed to run notification node: %v\n", err)
	}
}
