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

/* Auto-generated by genmsg_cpp from file /home/hojin/PX4-Autopilot/msg/vehicle_trajectory_waypoint.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_vehicle_trajectory_waypoint_fields[] = "uint64_t timestamp;uint8_t type;uint8_t[7] _padding0;trajectory_waypoint[5] waypoints;";

ORB_DEFINE(vehicle_trajectory_waypoint, struct vehicle_trajectory_waypoint_s, 296, __orb_vehicle_trajectory_waypoint_fields, static_cast<uint8_t>(ORB_ID::vehicle_trajectory_waypoint));
ORB_DEFINE(vehicle_trajectory_waypoint_desired, struct vehicle_trajectory_waypoint_s, 296, __orb_vehicle_trajectory_waypoint_fields, static_cast<uint8_t>(ORB_ID::vehicle_trajectory_waypoint_desired));


void print_message(const vehicle_trajectory_waypoint_s &message)
{

	PX4_INFO_RAW(" vehicle_trajectory_waypoint_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\ttype: %u\n", message.type);
		PX4_INFO_RAW("\tpx4/trajectory_waypoint[5] waypoints[0]");
 print_message(message.waypoints[0]);
PX4_INFO_RAW("\tpx4/trajectory_waypoint[5] waypoints[1]");
 print_message(message.waypoints[1]);
PX4_INFO_RAW("\tpx4/trajectory_waypoint[5] waypoints[2]");
 print_message(message.waypoints[2]);
PX4_INFO_RAW("\tpx4/trajectory_waypoint[5] waypoints[3]");
 print_message(message.waypoints[3]);
PX4_INFO_RAW("\tpx4/trajectory_waypoint[5] waypoints[4]");
 print_message(message.waypoints[4]);

}
