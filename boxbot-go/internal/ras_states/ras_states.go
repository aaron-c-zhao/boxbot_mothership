package ras_states

type RasState = string
type RasTransition = string

// All states of the RAS system
const (
	InitState      = "INIT"
	InitRasState   = "INIT_RAS"
	InitRcsState   = "INIT_RCS"
	RunningState   = "RUNNING"
	EmptyState     = "EMPTY"
	WaitingState   = "WAITING"
	CalibrateState = "CALIBRATING"
	PausedState    = "PAUSED"
	EmergencyState = "EMERGENCY"
	WatchdogState  = "WATCHDOG"
	StuckState     = "STUCK"
	ErrorState     = "ERR"
	StopRcsState   = "STOP_RCS"
	StopRasState   = "STOP_RAS"
)

// All transitions in the RAS system
const (
	StartInit         = "INIT_START"
	RasInitFailed     = "INIT_RAS_FAIL"
	RcsInitFailed     = "INIT_RCS_FAIL"
	RasInit           = "INIT_RAS_DONE"
	RcsInit           = "INIT_RCS_DONE"
	NoPallet          = "NO_PALLET"
	EmptyPallet       = "EMPTY_PALLET"
	PalletPlaced      = "PALLET_PLACED"
	Calibrated        = "CALIBRATED"
	StartButton       = "START_BTN"
	PauseButton       = "PAUSE_BTN"
	EmergencyFlagHigh = "EFLAG_HIGH"
	EmergencyFlagLow  = "EFLAG_LOW"
	WatchdogAlertHigh = "WD_ALERT_HIGH"
	WatchdogAlertLow  = "WD_ALERT_LOW"
	RcsStuck          = "RCS_STUCK"
	GenericErr        = "GENERIC_ERR"
	RcsStopFailed     = "STOP_RCS_FAIL"
	RcsStop           = "STOP_RCS_DONE"
	StopSystem        = "STOP_SYSTEM"
)
