package rcs_states

type RcsState = string
type RcsTransition = string

// All states of the RCS system
const (
	InitState                     = "INIT"
	InitRcsState                  = "INIT_RCS"
	RcsReady                      = "RSC_READY"
	CalibratingState              = "CALIBRATING"
	HomingState                   = "HOMING"
	EmergencyState                = "EMERGENCY"
	PausedState                   = "PAUSED"
	MovingToConveyorState         = "MOVING_TO_CONVEYOR"
	DroppingAtConveyorState       = "DROPPING_AT_CONVEYOR"
	MovingOutsideCameraRangeState = "MOVING_OUTSIDE_CAM_RANGE"
	SearchState                   = "SEARCHING"
	MovingToCartonState           = "MOVING_TO_CARTON"
	GrippingCartonState           = "GRIPPING_CARTON"
	EmptyState                    = "EMPTY"
	StuckState                    = "STUCK"
	HaltState                     = "HALT"
	ErrorState                    = "ERR"
	StopRcsState                  = "STOP_RCS"
)

// All transitions in the RCS system
const (
	StartInit     = "INIT_START"
	RcsInitFailed = "INIT_RCS_FAILED"
	RcsInit       = "INIT_RCS_DONE"
	RcsStart      = "START_RCS"
	RcsWait       = "WAIT_RCS"

	// Move to conveyor
	MoveToConveyor           = "MOVE_TO_CONVEYOR"
	MoveToConveyorDone       = "MOVE_TO_CONVEYOR_DONE"
	DropCartonsAtConveyor    = "DROP_CARTONS_AT_CONVEYOR"
	CartonsDroppedAtConveyor = "CARTONS_DROPPED_AT_CONVEYOR"
	FailedToDropCartons      = "DROP_CARTONS_FAILED"
	MoveToConveyorFailed     = "MOVE_TO_CONVEYOR_FAILED"

	// Main loop
	MoveOutsideCameraRange       = "MOVE_OUTSIDE_CAM_RANGE"
	MoveOutsideCameraRangeFailed = "MOVE_OUTSIDE_CAM_RANGE_FAILED"
	MoveOutsideCameraRangeDone   = "MOVE_OUTSIDE_CAM_RANGE_DONE"
	StartSearchCartons           = "START_CARTON_SEARCH"
	SearchCartonsDone            = "CARTON_SEARCH_DONE"
	NoCartonsFound               = "NO_CARTONS_FOUND_SEARCH"
	MoveToCarton                 = "MOVE_TO_CARTON"
	MovedToCarton                = "MOVED_TO_CARTON"
	MoveToCartonFailed           = "MOVE_TO_CARTON_FOUND"
	StartGrippingCarton          = "START_GRIPPING_CARTON"
	CartonGripped                = "CARTON_GRIPPED"
	GrippingCartonFailed         = "GRIPPING_CARTON_FAILED"
	AllCartonsMoved              = "ALL_CARTONS_MOVED"

	// Leave stuck
	OperatorFixedStuck   = "OPERATOR_FIXED_STUCK"
	OperatorAddedCartons = "OPERATOR_ADDED_CARTONS"

	// Generics events
	StartEmergency = "START_EMERGENCY"
	StopEmergency  = "STOP_EMERGENCY"
	StartHalt      = "START_HALT"
	StopHalt       = "STOP_HALT"
	StartStuck     = "START_STUCK"
	StartPause     = "START_PAUSE"
	StopPause      = "STOP_PAUSE"
	GenericErr     = "GENERIC_ERR"
	StartHoming    = "START_HOMING"
	HomingFailed   = "HOMING_FAILED"
	HomingDone     = "HOMING_DONE"

	// Stop events
	StopRcs       = "STOP_RCS"
	RcsStopFailed = "RCS_STOP_FAILED"
)
