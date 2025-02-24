/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
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


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/rover_mecanum_setpoint.h>


static inline constexpr int ucdr_topic_size_rover_mecanum_setpoint()
{
	return 36;
}

static inline bool ucdr_serialize_rover_mecanum_setpoint(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const rover_mecanum_setpoint_s& topic = *static_cast<const rover_mecanum_setpoint_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.forward_speed_setpoint) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.forward_speed_setpoint, sizeof(topic.forward_speed_setpoint));
	buf.iterator += sizeof(topic.forward_speed_setpoint);
	buf.offset += sizeof(topic.forward_speed_setpoint);
	static_assert(sizeof(topic.forward_speed_setpoint_normalized) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.forward_speed_setpoint_normalized, sizeof(topic.forward_speed_setpoint_normalized));
	buf.iterator += sizeof(topic.forward_speed_setpoint_normalized);
	buf.offset += sizeof(topic.forward_speed_setpoint_normalized);
	static_assert(sizeof(topic.lateral_speed_setpoint) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.lateral_speed_setpoint, sizeof(topic.lateral_speed_setpoint));
	buf.iterator += sizeof(topic.lateral_speed_setpoint);
	buf.offset += sizeof(topic.lateral_speed_setpoint);
	static_assert(sizeof(topic.lateral_speed_setpoint_normalized) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.lateral_speed_setpoint_normalized, sizeof(topic.lateral_speed_setpoint_normalized));
	buf.iterator += sizeof(topic.lateral_speed_setpoint_normalized);
	buf.offset += sizeof(topic.lateral_speed_setpoint_normalized);
	static_assert(sizeof(topic.yaw_rate_setpoint) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_rate_setpoint, sizeof(topic.yaw_rate_setpoint));
	buf.iterator += sizeof(topic.yaw_rate_setpoint);
	buf.offset += sizeof(topic.yaw_rate_setpoint);
	static_assert(sizeof(topic.yaw_rate_setpoint_normalized) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_rate_setpoint_normalized, sizeof(topic.yaw_rate_setpoint_normalized));
	buf.iterator += sizeof(topic.yaw_rate_setpoint_normalized);
	buf.offset += sizeof(topic.yaw_rate_setpoint_normalized);
	static_assert(sizeof(topic.yaw_setpoint) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_setpoint, sizeof(topic.yaw_setpoint));
	buf.iterator += sizeof(topic.yaw_setpoint);
	buf.offset += sizeof(topic.yaw_setpoint);
	return true;
}

static inline bool ucdr_deserialize_rover_mecanum_setpoint(ucdrBuffer& buf, rover_mecanum_setpoint_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.forward_speed_setpoint) == 4, "size mismatch");
	memcpy(&topic.forward_speed_setpoint, buf.iterator, sizeof(topic.forward_speed_setpoint));
	buf.iterator += sizeof(topic.forward_speed_setpoint);
	buf.offset += sizeof(topic.forward_speed_setpoint);
	static_assert(sizeof(topic.forward_speed_setpoint_normalized) == 4, "size mismatch");
	memcpy(&topic.forward_speed_setpoint_normalized, buf.iterator, sizeof(topic.forward_speed_setpoint_normalized));
	buf.iterator += sizeof(topic.forward_speed_setpoint_normalized);
	buf.offset += sizeof(topic.forward_speed_setpoint_normalized);
	static_assert(sizeof(topic.lateral_speed_setpoint) == 4, "size mismatch");
	memcpy(&topic.lateral_speed_setpoint, buf.iterator, sizeof(topic.lateral_speed_setpoint));
	buf.iterator += sizeof(topic.lateral_speed_setpoint);
	buf.offset += sizeof(topic.lateral_speed_setpoint);
	static_assert(sizeof(topic.lateral_speed_setpoint_normalized) == 4, "size mismatch");
	memcpy(&topic.lateral_speed_setpoint_normalized, buf.iterator, sizeof(topic.lateral_speed_setpoint_normalized));
	buf.iterator += sizeof(topic.lateral_speed_setpoint_normalized);
	buf.offset += sizeof(topic.lateral_speed_setpoint_normalized);
	static_assert(sizeof(topic.yaw_rate_setpoint) == 4, "size mismatch");
	memcpy(&topic.yaw_rate_setpoint, buf.iterator, sizeof(topic.yaw_rate_setpoint));
	buf.iterator += sizeof(topic.yaw_rate_setpoint);
	buf.offset += sizeof(topic.yaw_rate_setpoint);
	static_assert(sizeof(topic.yaw_rate_setpoint_normalized) == 4, "size mismatch");
	memcpy(&topic.yaw_rate_setpoint_normalized, buf.iterator, sizeof(topic.yaw_rate_setpoint_normalized));
	buf.iterator += sizeof(topic.yaw_rate_setpoint_normalized);
	buf.offset += sizeof(topic.yaw_rate_setpoint_normalized);
	static_assert(sizeof(topic.yaw_setpoint) == 4, "size mismatch");
	memcpy(&topic.yaw_setpoint, buf.iterator, sizeof(topic.yaw_setpoint));
	buf.iterator += sizeof(topic.yaw_setpoint);
	buf.offset += sizeof(topic.yaw_setpoint);
	return true;
}
