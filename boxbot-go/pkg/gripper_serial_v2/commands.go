package gripper_serial

type GripperCommand = byte

// All gripper commands, these include all commands
const (
	// This -> gripper

	// CommandCompleted is issued when a move target has been achieved
	CommandCompleted GripperCommand = 0x00
	// StartSearch is issued when the gripper should start searching for the stack
	StartSearch GripperCommand = 0x01
	// Reset is issued when the gripper should enter idle mode
	Reset GripperCommand = 0x03

	// Gripper -> this

	// GripCompleted is issued when the gripper has gripped the stack
	GripCompleted GripperCommand = 0x06
	// GripFailed is issued when the gripper is unable to grip the stack
	GripFailed GripperCommand = 0x07

	// DetectHit is issued when one of the IR sensors is triggered. Specific for V2
	DetectHit        GripperCommand = 0x20
	StartHigher      GripperCommand = 0x21
	RightEdgeFound   GripperCommand = 0x22
	HighLowEdgeFound GripperCommand = 0x23

	// Special commands
	OpenGripper           GripperCommand = 0xF0
	CloseGripper          GripperCommand = 0xF1
	GetGripStatus         GripperCommand = 0xF3
	GetGripStatusResponse GripperCommand = 0xF4
)
