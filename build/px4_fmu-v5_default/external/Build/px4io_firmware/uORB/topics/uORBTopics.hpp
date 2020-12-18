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

static constexpr size_t ORB_TOPICS_COUNT{157};
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
	actuator_controls_virtual_fw = 6,
	actuator_controls_virtual_mc = 7,
	actuator_outputs = 8,
	adc_report = 9,
	airspeed = 10,
	airspeed_validated = 11,
	battery_status = 12,
	camera_capture = 13,
	camera_trigger = 14,
	camera_trigger_secondary = 15,
	cellular_status = 16,
	collision_constraints = 17,
	collision_report = 18,
	commander_state = 19,
	cpuload = 20,
	differential_pressure = 21,
	distance_sensor = 22,
	ekf2_timestamps = 23,
	ekf_gps_drift = 24,
	esc_report = 25,
	esc_status = 26,
	estimator_attitude = 27,
	estimator_global_position = 28,
	estimator_innovation_test_ratios = 29,
	estimator_innovation_variances = 30,
	estimator_innovations = 31,
	estimator_local_position = 32,
	estimator_odometry = 33,
	estimator_optical_flow_vel = 34,
	estimator_selector_status = 35,
	estimator_sensor_bias = 36,
	estimator_states = 37,
	estimator_status = 38,
	estimator_visual_odometry_aligned = 39,
	follow_target = 40,
	fw_virtual_attitude_setpoint = 41,
	generator_status = 42,
	geofence_result = 43,
	gps_dump = 44,
	gps_inject_data = 45,
	home_position = 46,
	hover_thrust_estimate = 47,
	input_rc = 48,
	iridiumsbd_status = 49,
	irlock_report = 50,
	landing_gear = 51,
	landing_target_innovations = 52,
	landing_target_pose = 53,
	led_control = 54,
	log_message = 55,
	logger_status = 56,
	manual_control_setpoint = 57,
	manual_control_switches = 58,
	mavlink_log = 59,
	mc_virtual_attitude_setpoint = 60,
	mission = 61,
	mission_result = 62,
	mount_orientation = 63,
	multirotor_motor_limits = 64,
	navigator_mission_item = 65,
	obstacle_distance = 66,
	obstacle_distance_fused = 67,
	offboard_control_mode = 68,
	onboard_computer_status = 69,
	optical_flow = 70,
	orbit_status = 71,
	parameter_update = 72,
	ping = 73,
	position_controller_landing_status = 74,
	position_controller_status = 75,
	position_setpoint = 76,
	position_setpoint_triplet = 77,
	power_button_state = 78,
	power_monitor = 79,
	pwm_input = 80,
	px4io_status = 81,
	qshell_req = 82,
	qshell_retval = 83,
	radio_status = 84,
	rate_ctrl_status = 85,
	rc_channels = 86,
	rc_parameter_map = 87,
	rpm = 88,
	safety = 89,
	satellite_info = 90,
	sensor_accel = 91,
	sensor_accel_fifo = 92,
	sensor_baro = 93,
	sensor_combined = 94,
	sensor_correction = 95,
	sensor_gps = 96,
	sensor_gyro = 97,
	sensor_gyro_fft = 98,
	sensor_gyro_fifo = 99,
	sensor_mag = 100,
	sensor_preflight_mag = 101,
	sensor_selection = 102,
	sensors_status_imu = 103,
	system_power = 104,
	task_stack_info = 105,
	tecs_status = 106,
	telemetry_status = 107,
	test_motor = 108,
	timesync = 109,
	timesync_status = 110,
	trajectory_bezier = 111,
	trajectory_setpoint = 112,
	trajectory_waypoint = 113,
	transponder_report = 114,
	tune_control = 115,
	uavcan_parameter_request = 116,
	uavcan_parameter_value = 117,
	ulog_stream = 118,
	ulog_stream_ack = 119,
	vehicle_acceleration = 120,
	vehicle_air_data = 121,
	vehicle_angular_acceleration = 122,
	vehicle_angular_velocity = 123,
	vehicle_angular_velocity_groundtruth = 124,
	vehicle_attitude = 125,
	vehicle_attitude_groundtruth = 126,
	vehicle_attitude_setpoint = 127,
	vehicle_command = 128,
	vehicle_command_ack = 129,
	vehicle_constraints = 130,
	vehicle_control_mode = 131,
	vehicle_global_position = 132,
	vehicle_global_position_groundtruth = 133,
	vehicle_gps_position = 134,
	vehicle_imu = 135,
	vehicle_imu_status = 136,
	vehicle_land_detected = 137,
	vehicle_local_position = 138,
	vehicle_local_position_groundtruth = 139,
	vehicle_local_position_setpoint = 140,
	vehicle_magnetometer = 141,
	vehicle_mocap_odometry = 142,
	vehicle_odometry = 143,
	vehicle_rates_setpoint = 144,
	vehicle_roi = 145,
	vehicle_status = 146,
	vehicle_status_flags = 147,
	vehicle_trajectory_bezier = 148,
	vehicle_trajectory_waypoint = 149,
	vehicle_trajectory_waypoint_desired = 150,
	vehicle_vision_attitude = 151,
	vehicle_visual_odometry = 152,
	vtol_vehicle_status = 153,
	wheel_encoders = 154,
	wind_estimate = 155,
	yaw_estimator_status = 156,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
