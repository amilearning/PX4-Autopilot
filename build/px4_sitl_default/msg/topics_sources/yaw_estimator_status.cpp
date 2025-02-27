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

/* Auto-generated by genmsg_cpp from file /home/hojin/PX4-Autopilot/msg/yaw_estimator_status.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/yaw_estimator_status.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_yaw_estimator_status_fields[] = "uint64_t timestamp;uint64_t timestamp_sample;float yaw_composite;float yaw_variance;float[5] yaw;float[5] innov_vn;float[5] innov_ve;float[5] weight;";

ORB_DEFINE(yaw_estimator_status, struct yaw_estimator_status_s, 104, __orb_yaw_estimator_status_fields, static_cast<uint8_t>(ORB_ID::yaw_estimator_status));


void print_message(const yaw_estimator_status_s &message)
{

	PX4_INFO_RAW(" yaw_estimator_status_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	
	PX4_INFO_RAW("\ttimestamp_sample: %" PRIu64 "  (%" PRIu64 " us before timestamp)\n", message.timestamp_sample, message.timestamp - message.timestamp_sample);
	
	PX4_INFO_RAW("\tyaw_composite: %.4f\n", (double)message.yaw_composite);
	PX4_INFO_RAW("\tyaw_variance: %.4f\n", (double)message.yaw_variance);
	PX4_INFO_RAW("\tyaw: [%.4f, %.4f, %.4f, %.4f, %.4f]\n", (double)message.yaw[0], (double)message.yaw[1], (double)message.yaw[2], (double)message.yaw[3], (double)message.yaw[4]);
	PX4_INFO_RAW("\tinnov_vn: [%.4f, %.4f, %.4f, %.4f, %.4f]\n", (double)message.innov_vn[0], (double)message.innov_vn[1], (double)message.innov_vn[2], (double)message.innov_vn[3], (double)message.innov_vn[4]);
	PX4_INFO_RAW("\tinnov_ve: [%.4f, %.4f, %.4f, %.4f, %.4f]\n", (double)message.innov_ve[0], (double)message.innov_ve[1], (double)message.innov_ve[2], (double)message.innov_ve[3], (double)message.innov_ve[4]);
	PX4_INFO_RAW("\tweight: [%.4f, %.4f, %.4f, %.4f, %.4f]\n", (double)message.weight[0], (double)message.weight[1], (double)message.weight[2], (double)message.weight[3], (double)message.weight[4]);

}
