package gripper_serial

type GripperCommand = byte

// All gripper commands, these include some general commands, but also a few which are specifically for different
// gripper algos described in gripper_comm/main.go
// These specific commands are only used by those versions, so the final working version will have some unused commands.
const (
	// This -> gripper

	// CommandCompleted is issued when a move target has been achieved
	CommandCompleted GripperCommand = 0x00
	// StartSearch is issued when the gripper should start searching for the stack
	StartSearch GripperCommand = 0x01
	// StartGrip is issued when the gripper should start grabbing the stack (it assumes it's at the perfect place)
	StartGrip GripperCommand = 0x02
	// Reset is issued when the gripper should enter idle mode
	Reset GripperCommand = 0x03

	//// AnalyseCompleted is issued when the calculations are done, and the gripper can continue with the
	//// (optional) float32 given. Specific for V2
	//AnalyseCompleted GripperCommand = 0x0B

	// Gripper -> this

	// SearchCompleted is issued when the gripper has located the stack
	SearchCompleted GripperCommand = 0x04
	// SearchFailed is issued when the gripper is unable to locate the stack
	SearchFailed GripperCommand = 0x05
	// GripCompleted is issued when the gripper has gripped the stack
	GripCompleted GripperCommand = 0x06
	// GripFailed is issued when the gripper is unable to grip the stack
	GripFailed GripperCommand = 0x07
	// MoveDirCommand is issued when the gripper wants to move to a certain direction, it is up to the consumer to
	// interpret this direction (relative to center vs gripper)
	MoveDirCommand GripperCommand = 0x08

	// DetectHit is issued when one of the IR sensors is triggered. Specific for V2
	DetectHit GripperCommand = 0x09
	// WaitAnalyse is issued when the gripper has completed its search pattern, and is now waiting for the consumer
	// to analyse the data. Specific for V2
	WaitAnalyse GripperCommand = 0x0A
)
