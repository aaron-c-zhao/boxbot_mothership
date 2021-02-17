// Package lang contains all messages used in RAS. Might be replaced by full i18n in the future.
package lang

// Messages for on the display
const (
	StartingMessage    = "BoxBot is starting"
	RunningMessage     = "BoxBot is operating normally"
	EmptyMessage       = "BoxBot is out of carton"
	EmptyHint          = "Please place a new pallet"
	WaitingMessage     = "BoxBot is waiting for confirmation"
	WaitingHint        = "Please press the start button when ready"
	PausedMessage      = "BoxBot is currently paused"
	StuckMessage       = "BoxBot needs help"
	StuckHint          = "Press the start button when the issue is resolved"
	EmergencyMessage   = "BoxBot is currently stopped, due to an emergency"
	WatchdogMessage    = "BoxBot is currently stopped, due to a system failure"
	CalibratingMessage = "BoxBot is currently calibrating"
	StoppingMessage    = "BoxBot is stopping"
)

// Operator notifications
const (
	EmptyNotification     = "BoxBot is out of carton, please place a new pallet"
	StuckNotification     = "BoxBot is stuck, and needs help"
	EmergencyNotification = "BoxBot has made an emergency stop"
	WatchdogNotification  = "BoxBot has stopped due to a hardware failure"
	ErrorNotification     = "BoxBot has stopped due to a system malfunction"
)

// Error messages. Error codes:
/*
0x00: internal error -> restart
0x01: RCS failed to pause -> restart
0x02: RCS failed to resume -> restart
0x03: RCS failed to resume after emergency -> restart
0x04: RCS failed to resume after watchdog -> restart
0x05: RCS failed to stop -> maintenance required
0x06: RAS failed to start (constructor never made it) -> maintenance required
0x07: RCS failed to start (unable to create a new instance, ROS?) -> maintenance required
0x08: RCS failed to init -> maintenance required
0x09-0x0D: Power checks for input, motor1, motor2, motor3, or servo failed, respectively -> maintenance required
0x0E: Timeout while waiting for all RAS nodes to come online -> maintenance required
0x0F: Timeout while waiting for RCS node -> maintenance required
0x10: Unable to start watchdog after RCS init -> maintenance required
0x11: Unable to start watchdog after RCS init -> maintenance required
0x12,0x13: Unable to enable power to motor or servo (resp) at RCS init -> maintenance required
0x14,0x15: Unable to disable power to motor or servo (resp) at emergency start -> maintenance required
0x16,0x17: Unable to enable power to motor or servo (resp) at emergency end -> maintenance required
0x18,0x19: Unable to disable power to motor or servo (resp) at error start -> maintenance required
0x1A,0x1B: Unable to disable power to motor or servo (resp) at stop -> maintenance required
0x1C: FSM rcs resume at stuck fail -> maintenance
0x1D: FSM rcs resume at empty fail -> maintenance
0x1E: FSM rcs start at emergency fail -> maintenance
0x1F: FSM rcs stop at emergency fail -> maintenance
0x20: FSM rcs start at halt fail -> maintenance
0x21: FSM rcs stop at halt fail -> maintenance
0x22: FSM generic rcs error -> maintenance
*/
const (
	InternalError                  = "An internal error has occurred (0x00)"
	RCSPauseFailure                = "Unable to pause the system (0x01)"
	RCSResumeFailure               = "Unable to resume the system (0x02)"
	RCSResumeAfterEmergencyFailure = "Unable to resume the system (0x03)"
	RCSResumeAfterWatchdogFailure  = "Unable to resume the system (0x04)"
	RCSStopFail                    = "Unable to stop the system (0x05)"
	RASInitFail                    = "Unable to start the system (0x06)"
	RCSCreationFailed              = "Unable to start the system (0x07)"
	RCSInitFailed                  = "Unable to start the system (0x08)"
	PowerCheckInputFailed          = "System experienced a power failure (0x09)"
	PowerCheckMotor1Failed         = "System experienced a power failure (0x0A)"
	PowerCheckMotor2Failed         = "System experienced a power failure (0x0B)"
	PowerCheckMotor3Failed         = "System experienced a power failure (0x0C)"
	PowerCheckServoFailed          = "System experienced a power failure (0x0D)"
	RASInitTimeout                 = "Unable to start the system (0x0E)"
	RCSInitTimeout                 = "Unable to start the system (0x0F)"
	WatchdogInitFailed             = "Unable to start the system (0x10)"
	WatchdogStopFailed             = "Unable to stop the system (0x11)"
	RCSInitMotorRelayFail          = "Unable to start the system (0x12)"
	RCSInitServoRelayFail          = "Unable to start the system (0x13)"
	EmergencyMotorDisableRelayFail = "An internal error has occurred (0x14)"
	EmergencyServoDisableRelayFail = "An internal error has occurred (0x15)"
	EmergencyMotorEnableRelayFail  = "An internal error has occurred (0x16)"
	EmergencyServoEnableRelayFail  = "An internal error has occurred (0x17)"
	ErrMotorDisableRelayFail       = "An internal error has occurred (0x18)"
	ErrServoDisableRelayFail       = "An internal error has occurred (0x19)"
	StopMotorDisableRelayFail      = "An internal error has occurred (0x1A)"
	StopServoDisableRelayFail      = "An internal error has occurred (0x1B)"
	RCSResumeStuckFailure          = "Unable to resume the system (0x1C)"
	RCSResumeEmptyFailure          = "Unable to resume the system (0x1D)"
	RCSStartEmergencyFailure       = "Unable to handle emergency stop (0x1E)"
	RCSStopEmergencyFailure        = "Unable to handle emergency resume (0x1F)"
	RCSStartHaltFailure            = "Unable to handle halt stop (0x20)"
	RCSStopHaltFailure             = "Unable to handle halt resume (0x21)"
	RCSGenericError                = "Internal system (RCS) error (0x22)"
)

// Error resolve messages.
const (
	RestartSystem       = "Please restart the entire BoxBot system, and contact technical support if the problem persists"
	MaintenanceRequired = "Maintenance is required"
)
