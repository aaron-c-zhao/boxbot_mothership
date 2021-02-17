#!/usr/bin/env python3

import odrive as od
import rospy
from odrive.enums import *
import time
import math
from boxbot_autonomous.msg import Odrive
from boxbot_odrive_control.srv import Descend

drive = None

# Error storage
error_origin = 0  # 1 = axis, 2 = motor, 3 = controller, 4 = encoder
error_flag = 0

# poisitions from descending
start_pos = 0
end_pos = 0

CPR = 8192


def check_errors():
	global drive, error_origin, error_flag

	rospy.logdebug('checking for axis errors')
	if drive.axis0.error & ~(AXIS_ERROR_MIN_ENDSTOP_PRESSED | AXIS_ERROR_MAX_ENDSTOP_PRESSED) != 0:
		error_origin = 1
		error_flag = drive.axis0.error & ~(
			AXIS_ERROR_MIN_ENDSTOP_PRESSED | AXIS_ERROR_MAX_ENDSTOP_PRESSED)
		rospy.logerr('axis has error %d', error_flag)

	rospy.logdebug('checking for motor errors')
	if drive.axis0.motor.error != 0:
		error_origin = 2
		error_flag = drive.axis0.motor.error
		rospy.logerr('motor has error %d', error_flag)

	rospy.logdebug('checking for controller errors')
	if drive.axis0.controller.error != 0:
		error_origin = 3
		error_flag = drive.axis0.controller.error
		rospy.logerr('controller has error %d', error_flag)

	rospy.logdebug('checking for encoder errors')
	if drive.axis0.encoder.error != 0:
		error_origin = 4
		error_flag = drive.axis0.encoder.error
		rospy.logerr('encoder has error %d', error_flag)

	return error_flag


def set_goal(data):
	rospy.loginfo("Set goal position to %s", data.position)
	drive.axis0.controller.input_pos = data.position


def home():
    if drive.axis0.is_homed:
        return
    rospy.loginfo("Start homing")
    drive.axis0.requested_state = AXIS_STATE_HOMING


def enable_end_stops(enable_min, enable_max):
    drive.axis0.min_endstop.config.enabled = enable_min
    drive.axis0.max_endstop.config.enabled = enable_max

# confirm the setup if it's NC or NO switch


def end_stops_config(pin_min, pin_max):
    # TODO: chang the gpio pin to the real pins
	drive.axis0.min_endstop.config.gpio_num = pin_min
	drive.axis0.max_endstop.config.gpio_num = pin_max

	drive.axis0.min_endstop.config.offset = 0.0
	drive.axis0.max_endstop.config.offset = 0.0

	drive.axis0.min_endstop.config.is_active_high = True  # to be confirmed
	drive.axis0.max_endstop.config.is_active_high = True  # to be confirmed

	drive.axis0.min_endstop.config.pullup = True  # to be confirmed
	drive.axis0.max_endstop.config.pullup = True  # to be confirmed

	drive.axis0.max_endstop.config.debounce_ms = 50.0
	drive.axis0.min_endstop.config.debounce_ms = 50.0


def traj_param_config(vel_limit, accel_limit, decel_limit):
    # set the limit required by trajectory control(TODO: now random numbers)
    drive.axis0.trap_traj.config.vel_limit = vel_limit
    drive.axis0.trap_traj.config.accel_limit = accel_limit
    drive.axis0.trap_traj.config.decel_limit = decel_limit
    # drive.axis0.controller.config.inertia =  # optional config for better performance


def home_config(vel_ramp_rate, homing_speed):
    drive.axis0.controller.config.vel_ramp_rate = vel_ramp_rate  # random number
    drive.axis0.controller.config.homing_speed = homing_speed


def handle_descend(req):
	global start_pos, end_pos

	if req.cmd == req.START:
		start_pos = drive.axis0.encoder.pos_estimate
		drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		drive.axis0.controller.input_vel = 2.0
		rospy.loginfo("Start descending")
		return 0.0
	else:
		drive.axis0.controller.input_vel = 0.0
		end_pos = drive.axis0.encoder.pos_estimate
		rospy.loginfo("Stop descending")
		# return the position diff
		return (end_pos - start_pos) * math.pi * 0.02865 / CPR


def init():
	global drive

	print("Looking for Odrive...")
	drive = od.find_any()

	print("Start calibration...")
	drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
	while drive.axis0.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	# configure the odrive
	end_stops_config(5, 6)  # pins better not be 1,2 may conflict with UART
	traj_param_config(10, 1.3, 1.3)
	home_config(0.5, 3)

	#home()


	# set input mode to trajectory control
	drive.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

	enable_end_stops(False, False)

	rospy.loginfo("Calibration done")

	rospy.init_node("odrive", anonymous=True)
	rospy.Subscriber("boxbot_odrive", Odrive, set_goal)
	s = rospy.Service("boxbot_descend", Descend, handle_descend)
	rospy.spin()


if __name__ == '__main__':
    init()
