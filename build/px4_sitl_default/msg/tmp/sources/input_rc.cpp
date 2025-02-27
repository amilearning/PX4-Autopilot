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

/* Auto-generated by genmsg_cpp from file /home/hojin/PX4-Autopilot/msg/input_rc.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_input_rc_fields[] = "uint64_t timestamp;uint64_t timestamp_last_signal;int32_t rssi;uint16_t rc_lost_frame_count;uint16_t rc_total_frame_count;uint16_t rc_ppm_frame_length;uint16_t[18] values;uint8_t channel_count;bool rc_failsafe;bool rc_lost;uint8_t input_source;uint8_t[6] _padding0;";

ORB_DEFINE(input_rc, struct input_rc_s, 66, __orb_input_rc_fields, static_cast<uint8_t>(ORB_ID::input_rc));


void print_message(const input_rc_s &message)
{

	PX4_INFO_RAW(" input_rc_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\ttimestamp_last_signal: %" PRIu64 "\n", message.timestamp_last_signal);
	PX4_INFO_RAW("\trssi: %" PRId32 "\n", message.rssi);
	PX4_INFO_RAW("\trc_lost_frame_count: %u\n", message.rc_lost_frame_count);
	PX4_INFO_RAW("\trc_total_frame_count: %u\n", message.rc_total_frame_count);
	PX4_INFO_RAW("\trc_ppm_frame_length: %u\n", message.rc_ppm_frame_length);
	PX4_INFO_RAW("\tvalues: [%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]\n", message.values[0], message.values[1], message.values[2], message.values[3], message.values[4], message.values[5], message.values[6], message.values[7], message.values[8], message.values[9], message.values[10], message.values[11], message.values[12], message.values[13], message.values[14], message.values[15], message.values[16], message.values[17]);
	PX4_INFO_RAW("\tchannel_count: %u\n", message.channel_count);
	PX4_INFO_RAW("\trc_failsafe: %s\n", (message.rc_failsafe ? "True" : "False"));
	PX4_INFO_RAW("\trc_lost: %s\n", (message.rc_lost ? "True" : "False"));
	PX4_INFO_RAW("\tinput_source: %u\n", message.input_source);
	
}
