package main

import (
	"boxbot-go/pkg/msgs/ras"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/srvs/srv_hardware"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/manifoldco/promptui"
	"log"
	"time"
)

func main() {
	err := node.RunNodeWithCallback("mock interface", NewNode, func(n node.Node) {
		inputHandler(n.(*interfaceNode), n.Done())
	})
	if err != nil {
		log.Fatalf("failed to run mock interface node: %v\n", err)
	}
}

type entry struct {
	Name        string
	Icon        string
	Current     interface{}
	Open, Close string
}

func nodeActive(n *goroslib.Node, name string) string {
	_, err := n.PingNode(name)
	return boolColor(err == nil)
}

func boolColor(b bool) string {
	if b {
		return fmt.Sprintf("{{ %v | green | bold }}", b)
	} else {
		return fmt.Sprintf("{{ %v | red | bold }}", b)
	}
}

func inputHandler(n *interfaceNode, stop <-chan struct{}) {
	fmt.Printf("Current state will refresh on every action or when 'Refresh' is selected\n")
	cnt := 1
	go func() {
		for {
			cnt += 1
			time.Sleep(time.Second)
		}
	}()

	for {
		select {
		case <-stop:
			break
		default:
		}

		entries := []entry{
			{Name: "Refresh", Icon: "\U0001F5D8", Current: ""},
			{Name: "Toggle input power check", Icon: "\U0001F5F2", Current: n.InputPower, Open: "(Failed: ", Close: ")"},
			{Name: "Toggle motor1 power check", Icon: "\U0001F5F2", Current: n.Motor1Power, Open: "(Failed: ", Close: ")"},
			{Name: "Toggle motor2 power check", Icon: "\U0001F5F2", Current: n.Motor2Power, Open: "(Failed: ", Close: ")"},
			{Name: "Toggle motor3 power check", Icon: "\U0001F5F2", Current: n.Motor2Power, Open: "(Failed: ", Close: ")"},
			{Name: "Toggle servo power check", Icon: "\U0001F5F2", Current: n.ServoPower, Open: "(Failed: ", Close: ")"},
			{Name: "Toggle emergency button", Icon: "\U0001F6C7", Current: n.Emergency, Open: "(Emergency: ", Close: ")"},
			{Name: "Press start button", Icon: "\U0001F446", Current: n.StartBtn, Open: "(Last received effect: ", Close: ")"},
			{Name: "Press pause button", Icon: "\U0001F446", Current: n.PauseBtn, Open: "(Last received effect: ", Close: ")"},
			{Name: "Press power button", Icon: "\U0001F446", Current: n.PowerBtn, Open: "(Last received effect: ", Close: ")"},
		}

		templates := &promptui.SelectTemplates{
			Label: "\033[34m?\033[0m  {{ . | red }}",
			Active: "\033[1m▸\033[0m {{ .Icon | red }} {{ .Name | cyan | bold }} {{ .Open | bold }}" +
				"{{ .Current | bold }}{{ .Close | bold }}",
			Inactive: " {{ .Icon }} {{ .Name | cyan }} {{ .Open }}{{ .Current }}{{ .Close }}",
			Selected: "\U0001F336 {{ .Name | red | cyan }}",
			Details: fmt.Sprintf(`
--- {{ "Interface" | underline }} ---
Interface text: '%s'
Bar color: %s
Bar effect: %s
Motor relay active: %s
Servo relay active: %s
--- {{ "Active RAS nodes" | underline }} ---
Mock interface: {{ true | green | bold }}
Base: %s
Notification: %s
Logging: %s
Watchdog: %s
`, n.InterfaceText, n.BarColor, n.BarEffect, boolColor(n.MotorRelayActive), boolColor(n.ServoRelayActive),
				nodeActive(n.n, node.RasBaseName), nodeActive(n.n, node.NotificationServiceName),
				nodeActive(n.n, node.LoggingServiceName), nodeActive(n.n, node.WatchdogName)),
		}

		prompt := promptui.Select{
			Label:        "Choose action to simulate",
			Items:        entries,
			Templates:    templates,
			Size:         10,
			HideHelp:     true,
			HideSelected: true,
		}

		i, _, err := prompt.Run()

		if err != nil {
			fmt.Printf("Prompt failed %v\n", err)
			return
		}

		switch i {
		case 1:
			n.InputPower = !n.InputPower
			if err := n.setCheck(srv_hardware.CheckInput, n.InputPower); err != nil {
				log.Fatalf("unable to interact with ROS: %v\n", err)
			}
		case 2:
			n.Motor1Power = !n.Motor1Power
			if err := n.setCheck(srv_hardware.CheckMotor1, n.Motor1Power); err != nil {
				log.Fatalf("unable to interact with ROS: %v\n", err)
			}
		case 3:
			n.Motor2Power = !n.Motor2Power
			if err := n.setCheck(srv_hardware.CheckMotor2, n.Motor2Power); err != nil {
				log.Fatalf("unable to interact with ROS: %v\n", err)
			}
		case 4:
			n.Motor3Power = !n.Motor3Power
			if err := n.setCheck(srv_hardware.CheckMotor3, n.Motor3Power); err != nil {
				log.Fatalf("unable to interact with ROS: %v\n", err)
			}
		case 5:
			n.ServoPower = !n.ServoPower
			if err := n.setCheck(srv_hardware.CheckServo, n.ServoPower); err != nil {
				log.Fatalf("unable to interact with ROS: %v\n", err)
			}
		case 6:
			if err := n.toggleEmergency(); err != nil {
				log.Fatalf("unable to interact with ROS: %v\n", err)
			}
		case 7:
			n.pressButton(ras.StartButton)
		case 8:
			n.pressButton(ras.PauseButton)
		case 9:
			n.pressButton(ras.PowerButton)
		}
	}
}
