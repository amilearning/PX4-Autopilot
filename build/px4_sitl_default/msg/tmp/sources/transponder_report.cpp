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

/* Auto-generated by genmsg_cpp from file /home/hojin/PX4-Autopilot/msg/transponder_report.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_transponder_report_fields[] = "uint64_t timestamp;double lat;double lon;uint32_t icao_address;float altitude;float heading;float hor_velocity;float ver_velocity;uint16_t flags;uint16_t squawk;uint8_t altitude_type;char[9] callsign;uint8_t emitter_type;uint8_t tslc;uint8_t[18] uas_id;uint8_t[2] _padding0;";

ORB_DEFINE(transponder_report, struct transponder_report_s, 78, __orb_transponder_report_fields, static_cast<uint8_t>(ORB_ID::transponder_report));


void print_message(const transponder_report_s &message)
{

	PX4_INFO_RAW(" transponder_report_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\tlat: %.6f\n", message.lat);
	PX4_INFO_RAW("\tlon: %.6f\n", message.lon);
	PX4_INFO_RAW("\ticao_address: %" PRIu32 "\n", message.icao_address);
	PX4_INFO_RAW("\taltitude: %.4f\n", (double)message.altitude);
	PX4_INFO_RAW("\theading: %.4f\n", (double)message.heading);
	PX4_INFO_RAW("\thor_velocity: %.4f\n", (double)message.hor_velocity);
	PX4_INFO_RAW("\tver_velocity: %.4f\n", (double)message.ver_velocity);
	PX4_INFO_RAW("\tflags: %u (0b", message.flags);
	for (int i = (sizeof(message.flags) * 8) - 1; i >= 0; i--) { PX4_INFO_RAW("%lu%s", (unsigned long) message.flags >> i & 1, ((unsigned)i < (sizeof(message.flags) * 8) - 1 && i % 4 == 0 && i > 0) ? "'" : ""); }
	PX4_INFO_RAW(")\n");
	PX4_INFO_RAW("\tsquawk: %u\n", message.squawk);
	PX4_INFO_RAW("\taltitude_type: %u\n", message.altitude_type);
	PX4_INFO_RAW("\tcallsign: \"%.9s\" \n", message.callsign);
	PX4_INFO_RAW("\temitter_type: %u\n", message.emitter_type);
	PX4_INFO_RAW("\ttslc: %u\n", message.tslc);
	PX4_INFO_RAW("\tuas_id: [%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]\n", message.uas_id[0], message.uas_id[1], message.uas_id[2], message.uas_id[3], message.uas_id[4], message.uas_id[5], message.uas_id[6], message.uas_id[7], message.uas_id[8], message.uas_id[9], message.uas_id[10], message.uas_id[11], message.uas_id[12], message.uas_id[13], message.uas_id[14], message.uas_id[15], message.uas_id[16], message.uas_id[17]);
	
}
