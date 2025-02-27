/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file /home/hojin/PX4-Autopilot/msg/vehicle_gps_position.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_vehicle_gps_position_fields[] = "uint64_t timestamp;uint64_t time_utc_usec;int32_t lat;int32_t lon;int32_t alt;int32_t alt_ellipsoid;float s_variance_m_s;float c_variance_rad;float eph;float epv;float hdop;float vdop;int32_t noise_per_ms;int32_t jamming_indicator;float vel_m_s;float vel_n_m_s;float vel_e_m_s;float vel_d_m_s;float cog_rad;int32_t timestamp_time_relative;float heading;float heading_offset;uint8_t fix_type;uint8_t jamming_state;bool vel_ned_valid;uint8_t satellites_used;uint8_t selected;uint8_t[3] _padding0;";

ORB_DEFINE(vehicle_gps_position, struct vehicle_gps_position_s, 101, __orb_vehicle_gps_position_fields, static_cast<uint8_t>(ORB_ID::vehicle_gps_position));


void print_message(const vehicle_gps_position_s &message)
{

	PX4_INFO_RAW(" vehicle_gps_position_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\ttime_utc_usec: %" PRIu64 "\n", message.time_utc_usec);
	PX4_INFO_RAW("\tlat: %" PRId32 "\n", message.lat);
	PX4_INFO_RAW("\tlon: %" PRId32 "\n", message.lon);
	PX4_INFO_RAW("\talt: %" PRId32 "\n", message.alt);
	PX4_INFO_RAW("\talt_ellipsoid: %" PRId32 "\n", message.alt_ellipsoid);
	PX4_INFO_RAW("\ts_variance_m_s: %.4f\n", (double)message.s_variance_m_s);
	PX4_INFO_RAW("\tc_variance_rad: %.4f\n", (double)message.c_variance_rad);
	PX4_INFO_RAW("\teph: %.4f\n", (double)message.eph);
	PX4_INFO_RAW("\tepv: %.4f\n", (double)message.epv);
	PX4_INFO_RAW("\thdop: %.4f\n", (double)message.hdop);
	PX4_INFO_RAW("\tvdop: %.4f\n", (double)message.vdop);
	PX4_INFO_RAW("\tnoise_per_ms: %" PRId32 "\n", message.noise_per_ms);
	PX4_INFO_RAW("\tjamming_indicator: %" PRId32 "\n", message.jamming_indicator);
	PX4_INFO_RAW("\tvel_m_s: %.4f\n", (double)message.vel_m_s);
	PX4_INFO_RAW("\tvel_n_m_s: %.4f\n", (double)message.vel_n_m_s);
	PX4_INFO_RAW("\tvel_e_m_s: %.4f\n", (double)message.vel_e_m_s);
	PX4_INFO_RAW("\tvel_d_m_s: %.4f\n", (double)message.vel_d_m_s);
	PX4_INFO_RAW("\tcog_rad: %.4f\n", (double)message.cog_rad);
	PX4_INFO_RAW("\ttimestamp_time_relative: %" PRId32 "\n", message.timestamp_time_relative);
	PX4_INFO_RAW("\theading: %.4f\n", (double)message.heading);
	PX4_INFO_RAW("\theading_offset: %.4f\n", (double)message.heading_offset);
	PX4_INFO_RAW("\tfix_type: %u\n", message.fix_type);
	PX4_INFO_RAW("\tjamming_state: %u\n", message.jamming_state);
	PX4_INFO_RAW("\tvel_ned_valid: %s\n", (message.vel_ned_valid ? "True" : "False"));
	PX4_INFO_RAW("\tsatellites_used: %u\n", message.satellites_used);
	PX4_INFO_RAW("\tselected: %u\n", message.selected);
	
}
