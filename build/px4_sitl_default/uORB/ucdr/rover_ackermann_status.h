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
#include <uORB/topics/rover_ackermann_status.h>


static inline constexpr int ucdr_topic_size_rover_ackermann_status()
{
	return 36;
}

static inline bool ucdr_serialize_rover_ackermann_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const rover_ackermann_status_s& topic = *static_cast<const rover_ackermann_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.measured_forward_speed) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.measured_forward_speed, sizeof(topic.measured_forward_speed));
	buf.iterator += sizeof(topic.measured_forward_speed);
	buf.offset += sizeof(topic.measured_forward_speed);
	static_assert(sizeof(topic.adjusted_forward_speed_setpoint) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.adjusted_forward_speed_setpoint, sizeof(topic.adjusted_forward_speed_setpoint));
	buf.iterator += sizeof(topic.adjusted_forward_speed_setpoint);
	buf.offset += sizeof(topic.adjusted_forward_speed_setpoint);
	static_assert(sizeof(topic.steering_setpoint_normalized) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.steering_setpoint_normalized, sizeof(topic.steering_setpoint_normalized));
	buf.iterator += sizeof(topic.steering_setpoint_normalized);
	buf.offset += sizeof(topic.steering_setpoint_normalized);
	static_assert(sizeof(topic.adjusted_steering_setpoint_normalized) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.adjusted_steering_setpoint_normalized, sizeof(topic.adjusted_steering_setpoint_normalized));
	buf.iterator += sizeof(topic.adjusted_steering_setpoint_normalized);
	buf.offset += sizeof(topic.adjusted_steering_setpoint_normalized);
	static_assert(sizeof(topic.measured_lateral_acceleration) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.measured_lateral_acceleration, sizeof(topic.measured_lateral_acceleration));
	buf.iterator += sizeof(topic.measured_lateral_acceleration);
	buf.offset += sizeof(topic.measured_lateral_acceleration);
	static_assert(sizeof(topic.pid_throttle_integral) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pid_throttle_integral, sizeof(topic.pid_throttle_integral));
	buf.iterator += sizeof(topic.pid_throttle_integral);
	buf.offset += sizeof(topic.pid_throttle_integral);
	static_assert(sizeof(topic.pid_lat_accel_integral) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pid_lat_accel_integral, sizeof(topic.pid_lat_accel_integral));
	buf.iterator += sizeof(topic.pid_lat_accel_integral);
	buf.offset += sizeof(topic.pid_lat_accel_integral);
	return true;
}

static inline bool ucdr_deserialize_rover_ackermann_status(ucdrBuffer& buf, rover_ackermann_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.measured_forward_speed) == 4, "size mismatch");
	memcpy(&topic.measured_forward_speed, buf.iterator, sizeof(topic.measured_forward_speed));
	buf.iterator += sizeof(topic.measured_forward_speed);
	buf.offset += sizeof(topic.measured_forward_speed);
	static_assert(sizeof(topic.adjusted_forward_speed_setpoint) == 4, "size mismatch");
	memcpy(&topic.adjusted_forward_speed_setpoint, buf.iterator, sizeof(topic.adjusted_forward_speed_setpoint));
	buf.iterator += sizeof(topic.adjusted_forward_speed_setpoint);
	buf.offset += sizeof(topic.adjusted_forward_speed_setpoint);
	static_assert(sizeof(topic.steering_setpoint_normalized) == 4, "size mismatch");
	memcpy(&topic.steering_setpoint_normalized, buf.iterator, sizeof(topic.steering_setpoint_normalized));
	buf.iterator += sizeof(topic.steering_setpoint_normalized);
	buf.offset += sizeof(topic.steering_setpoint_normalized);
	static_assert(sizeof(topic.adjusted_steering_setpoint_normalized) == 4, "size mismatch");
	memcpy(&topic.adjusted_steering_setpoint_normalized, buf.iterator, sizeof(topic.adjusted_steering_setpoint_normalized));
	buf.iterator += sizeof(topic.adjusted_steering_setpoint_normalized);
	buf.offset += sizeof(topic.adjusted_steering_setpoint_normalized);
	static_assert(sizeof(topic.measured_lateral_acceleration) == 4, "size mismatch");
	memcpy(&topic.measured_lateral_acceleration, buf.iterator, sizeof(topic.measured_lateral_acceleration));
	buf.iterator += sizeof(topic.measured_lateral_acceleration);
	buf.offset += sizeof(topic.measured_lateral_acceleration);
	static_assert(sizeof(topic.pid_throttle_integral) == 4, "size mismatch");
	memcpy(&topic.pid_throttle_integral, buf.iterator, sizeof(topic.pid_throttle_integral));
	buf.iterator += sizeof(topic.pid_throttle_integral);
	buf.offset += sizeof(topic.pid_throttle_integral);
	static_assert(sizeof(topic.pid_lat_accel_integral) == 4, "size mismatch");
	memcpy(&topic.pid_lat_accel_integral, buf.iterator, sizeof(topic.pid_lat_accel_integral));
	buf.iterator += sizeof(topic.pid_lat_accel_integral);
	buf.offset += sizeof(topic.pid_lat_accel_integral);
	return true;
}
