package node

import (
	"fmt"
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
	"io"
	"log"
	"os"
	"time"
)

const (
	GlobalNamespace = "/boxbot"
	RasNamespace    = GlobalNamespace + "/ras"
	RcsNamespace    = GlobalNamespace + "/rcs"
)

const (
	NotificationServiceName = "notification_service"
	LoggingServiceName      = "logging_service"
	WatchdogName            = "watchdog"
	InterfaceName           = "interface"
	RasBaseName             = "ras_base"
)

const (
	StepperDriverName = "steppers_driver"
	GripperDriverName = "gripper_driver"
	ServoDriverName   = "servo_driver"
	RcsAutonomousName = "rcs_autonomous"
)

const (
	RcsServiceName = "rcs_service"
)

var (
	RasNodes = []string{WatchdogName, NotificationServiceName, LoggingServiceName, InterfaceName, RasBaseName}
	RcsNodes = []string{RcsServiceName}

	// List of nodes that will be killed by base on shutdown. Note that both the base and watchdog are not in this list
	// as killing those would have consequences (incomplete stop and hard reset, resp)
	RasKillNodes = []string{NotificationServiceName, LoggingServiceName, InterfaceName}
)

type Config struct {
	RosPort                int    `default:"11311"`
	RosHost                string `default:"localhost"`
	RosRetries             int    `default:"4"`
	RosRetryBackoffSeconds int    `default:"4"`
}

type Node interface {
	io.Closer
	Done() <-chan struct{}
}

// RunNode attempts to start the given node node with the config retrieved from the environment.
// This function will block until the node stops, or after the node has failed to start in which case it will
// return an error.
func RunNode(name string, init func(Config) (Node, error)) error {
	return RunNodeWithCallback(name, init, func(_ Node) {})
}

// RunNodeWithCallback has the same functionality as RunNode, but will also call the given callback in a new
// goroutine, after the node has started.
func RunNodeWithCallback(name string, init func(Config) (Node, error), cb func(Node)) error {
	log.Printf("starting %s node\n", name)
	var c Config
	err := envconfig.Process("RAS", &c)
	if err != nil {
		return errors.Errorf("unable to retrieve full configuration from environment: %v\n", err)
	}

	var n Node
	for i := 0; i < c.RosRetries; i++ {
		n, err = init(c)
		if err == nil {
			break
		}
		log.Printf("failed to start %s node: %v\n", name, err)

		if tries := c.RosRetries - i - 1; tries > 0 {
			log.Printf("Retrying %v more times in %v seconds\n", tries, c.RosRetryBackoffSeconds)
			time.Sleep(time.Second * time.Duration(c.RosRetryBackoffSeconds))
		}
	}

	if n == nil {
		return errors.Errorf("unable to start %s node, failed %v times.\n", name, c.RosRetries)
	}

	go SignalIntercept(n)
	go cb(n)

	<-n.Done()

	log.Printf("stopping %s node\n", name)
	if err := n.Close(); err != nil {
		fmt.Printf("failed to stop %s node properly: %v\n", name, err)
		os.Exit(1)
	}

	return nil
}
