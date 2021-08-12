/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{193};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_4 = 6,
	actuator_controls_5 = 7,
	actuator_controls_virtual_fw = 8,
	actuator_controls_virtual_mc = 9,
	actuator_outputs = 10,
	adc_report = 11,
	airspeed = 12,
	airspeed_validated = 13,
	airspeed_wind = 14,
	baro_bias_estimate = 15,
	battery_status = 16,
	camera_capture = 17,
	camera_trigger = 18,
	camera_trigger_secondary = 19,
	cellular_status = 20,
	collision_constraints = 21,
	collision_report = 22,
	commander_state = 23,
	control_allocator_status = 24,
	cpuload = 25,
	debug_array = 26,
	debug_key_value = 27,
	debug_value = 28,
	debug_vect = 29,
	differential_pressure = 30,
	distance_sensor = 31,
	ekf2_timestamps = 32,
	ekf_gps_drift = 33,
	esc_report = 34,
	esc_status = 35,
	estimator_attitude = 36,
	estimator_event_flags = 37,
	estimator_global_position = 38,
	estimator_innovation_test_ratios = 39,
	estimator_innovation_variances = 40,
	estimator_innovations = 41,
	estimator_local_position = 42,
	estimator_odometry = 43,
	estimator_optical_flow_vel = 44,
	estimator_selector_status = 45,
	estimator_sensor_bias = 46,
	estimator_states = 47,
	estimator_status = 48,
	estimator_status_flags = 49,
	estimator_visual_odometry_aligned = 50,
	estimator_wind = 51,
	event = 52,
	follow_target = 53,
	fw_virtual_attitude_setpoint = 54,
	generator_status = 55,
	geofence_result = 56,
	gimbal_device_attitude_status = 57,
	gimbal_device_information = 58,
	gimbal_device_set_attitude = 59,
	gimbal_manager_information = 60,
	gimbal_manager_set_attitude = 61,
	gimbal_manager_set_manual_control = 62,
	gimbal_manager_status = 63,
	gps_dump = 64,
	gps_inject_data = 65,
	heater_status = 66,
	home_position = 67,
	hover_thrust_estimate = 68,
	input_rc = 69,
	iridiumsbd_status = 70,
	irlock_report = 71,
	landing_gear = 72,
	landing_target_innovations = 73,
	landing_target_pose = 74,
	led_control = 75,
	log_message = 76,
	logger_status = 77,
	mag_worker_data = 78,
	manual_control_setpoint = 79,
	manual_control_switches = 80,
	mavlink_log = 81,
	mc_virtual_attitude_setpoint = 82,
	mission = 83,
	mission_result = 84,
	mount_orientation = 85,
	multirotor_motor_limits = 86,
	navigator_mission_item = 87,
	obstacle_distance = 88,
	obstacle_distance_fused = 89,
	offboard_control_mode = 90,
	onboard_computer_status = 91,
	optical_flow = 92,
	orb_multitest = 93,
	orb_test = 94,
	orb_test_large = 95,
	orb_test_medium = 96,
	orb_test_medium_multi = 97,
	orb_test_medium_queue = 98,
	orb_test_medium_queue_poll = 99,
	orb_test_medium_wrap_around = 100,
	orbit_status = 101,
	parameter_update = 102,
	ping = 103,
	position_controller_landing_status = 104,
	position_controller_status = 105,
	position_setpoint = 106,
	position_setpoint_triplet = 107,
	power_button_state = 108,
	power_monitor = 109,
	pwm_input = 110,
	px4io_status = 111,
	qshell_req = 112,
	qshell_retval = 113,
	radio_status = 114,
	rate_ctrl_status = 115,
	rc_channels = 116,
	rc_parameter_map = 117,
	rpm = 118,
	rtl_flight_time = 119,
	safety = 120,
	satellite_info = 121,
	sensor_accel = 122,
	sensor_accel_fifo = 123,
	sensor_baro = 124,
	sensor_combined = 125,
	sensor_correction = 126,
	sensor_gps = 127,
	sensor_gyro = 128,
	sensor_gyro_fft = 129,
	sensor_gyro_fifo = 130,
	sensor_mag = 131,
	sensor_preflight_mag = 132,
	sensor_selection = 133,
	sensors_status_imu = 134,
	system_power = 135,
	takeoff_status = 136,
	task_stack_info = 137,
	tecs_status = 138,
	telemetry_status = 139,
	test_motor = 140,
	timesync = 141,
	timesync_status = 142,
	trajectory_bezier = 143,
	trajectory_setpoint = 144,
	trajectory_waypoint = 145,
	transponder_report = 146,
	tune_control = 147,
	uavcan_parameter_request = 148,
	uavcan_parameter_value = 149,
	ulog_stream = 150,
	ulog_stream_ack = 151,
	vehicle_acceleration = 152,
	vehicle_actuator_setpoint = 153,
	vehicle_air_data = 154,
	vehicle_angular_acceleration = 155,
	vehicle_angular_acceleration_setpoint = 156,
	vehicle_angular_velocity = 157,
	vehicle_angular_velocity_groundtruth = 158,
	vehicle_attitude = 159,
	vehicle_attitude_groundtruth = 160,
	vehicle_attitude_setpoint = 161,
	vehicle_command = 162,
	vehicle_command_ack = 163,
	vehicle_constraints = 164,
	vehicle_control_mode = 165,
	vehicle_global_position = 166,
	vehicle_global_position_groundtruth = 167,
	vehicle_gps_position = 168,
	vehicle_imu = 169,
	vehicle_imu_status = 170,
	vehicle_land_detected = 171,
	vehicle_local_position = 172,
	vehicle_local_position_groundtruth = 173,
	vehicle_local_position_setpoint = 174,
	vehicle_magnetometer = 175,
	vehicle_mocap_odometry = 176,
	vehicle_odometry = 177,
	vehicle_rates_setpoint = 178,
	vehicle_roi = 179,
	vehicle_status = 180,
	vehicle_status_flags = 181,
	vehicle_thrust_setpoint = 182,
	vehicle_torque_setpoint = 183,
	vehicle_trajectory_bezier = 184,
	vehicle_trajectory_waypoint = 185,
	vehicle_trajectory_waypoint_desired = 186,
	vehicle_vision_attitude = 187,
	vehicle_visual_odometry = 188,
	vtol_vehicle_status = 189,
	wheel_encoders = 190,
	wind = 191,
	yaw_estimator_status = 192,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
