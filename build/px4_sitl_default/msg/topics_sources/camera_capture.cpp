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

/* Auto-generated by genmsg_cpp from file /home/hojin/PX4-Autopilot/msg/camera_capture.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_camera_capture_fields[] = "uint64_t timestamp;uint64_t timestamp_utc;double lat;double lon;uint32_t seq;float alt;float ground_distance;float[4] q;int8_t result;uint8_t[3] _padding0;";

ORB_DEFINE(camera_capture, struct camera_capture_s, 61, __orb_camera_capture_fields, static_cast<uint8_t>(ORB_ID::camera_capture));


void print_message(const camera_capture_s &message)
{

	PX4_INFO_RAW(" camera_capture_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\ttimestamp_utc: %" PRIu64 "\n", message.timestamp_utc);
	PX4_INFO_RAW("\tlat: %.6f\n", message.lat);
	PX4_INFO_RAW("\tlon: %.6f\n", message.lon);
	PX4_INFO_RAW("\tseq: %" PRIu32 "\n", message.seq);
	PX4_INFO_RAW("\talt: %.4f\n", (double)message.alt);
	PX4_INFO_RAW("\tground_distance: %.4f\n", (double)message.ground_distance);
	{
		matrix::Eulerf euler{matrix::Quatf{message.q}};
		PX4_INFO_RAW("\tq: [%.4f, %.4f, %.4f, %.4f]  (Roll: %.1f deg, Pitch: %.1f deg, Yaw: %.1f deg)\n", (double)message.q[0], (double)message.q[1], (double)message.q[2], (double)message.q[3], (double)math::degrees(euler(0)), (double)math::degrees(euler(1)), (double)math::degrees(euler(2)));
	
	}
	PX4_INFO_RAW("\tresult: %d\n", message.result);
	
}
