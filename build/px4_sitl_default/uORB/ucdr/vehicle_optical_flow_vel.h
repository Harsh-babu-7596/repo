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
#include <uORB/topics/vehicle_optical_flow_vel.h>


static inline constexpr int ucdr_topic_size_vehicle_optical_flow_vel()
{
	return 100;
}

static inline bool ucdr_serialize_vehicle_optical_flow_vel(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const vehicle_optical_flow_vel_s& topic = *static_cast<const vehicle_optical_flow_vel_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.timestamp_sample) == 8, "size mismatch");
	const uint64_t timestamp_sample_adjusted = topic.timestamp_sample + time_offset;
	memcpy(buf.iterator, &timestamp_sample_adjusted, sizeof(topic.timestamp_sample));
	buf.iterator += sizeof(topic.timestamp_sample);
	buf.offset += sizeof(topic.timestamp_sample);
	static_assert(sizeof(topic.vel_body) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.vel_body, sizeof(topic.vel_body));
	buf.iterator += sizeof(topic.vel_body);
	buf.offset += sizeof(topic.vel_body);
	static_assert(sizeof(topic.vel_ne) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.vel_ne, sizeof(topic.vel_ne));
	buf.iterator += sizeof(topic.vel_ne);
	buf.offset += sizeof(topic.vel_ne);
	static_assert(sizeof(topic.vel_body_filtered) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.vel_body_filtered, sizeof(topic.vel_body_filtered));
	buf.iterator += sizeof(topic.vel_body_filtered);
	buf.offset += sizeof(topic.vel_body_filtered);
	static_assert(sizeof(topic.vel_ne_filtered) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.vel_ne_filtered, sizeof(topic.vel_ne_filtered));
	buf.iterator += sizeof(topic.vel_ne_filtered);
	buf.offset += sizeof(topic.vel_ne_filtered);
	static_assert(sizeof(topic.flow_rate_uncompensated) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.flow_rate_uncompensated, sizeof(topic.flow_rate_uncompensated));
	buf.iterator += sizeof(topic.flow_rate_uncompensated);
	buf.offset += sizeof(topic.flow_rate_uncompensated);
	static_assert(sizeof(topic.flow_rate_compensated) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.flow_rate_compensated, sizeof(topic.flow_rate_compensated));
	buf.iterator += sizeof(topic.flow_rate_compensated);
	buf.offset += sizeof(topic.flow_rate_compensated);
	static_assert(sizeof(topic.gyro_rate) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.gyro_rate, sizeof(topic.gyro_rate));
	buf.iterator += sizeof(topic.gyro_rate);
	buf.offset += sizeof(topic.gyro_rate);
	static_assert(sizeof(topic.gyro_bias) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.gyro_bias, sizeof(topic.gyro_bias));
	buf.iterator += sizeof(topic.gyro_bias);
	buf.offset += sizeof(topic.gyro_bias);
	static_assert(sizeof(topic.ref_gyro) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.ref_gyro, sizeof(topic.ref_gyro));
	buf.iterator += sizeof(topic.ref_gyro);
	buf.offset += sizeof(topic.ref_gyro);
	return true;
}

static inline bool ucdr_deserialize_vehicle_optical_flow_vel(ucdrBuffer& buf, vehicle_optical_flow_vel_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.timestamp_sample, buf.iterator, sizeof(topic.timestamp_sample));
	if (topic.timestamp_sample == 0) topic.timestamp_sample = hrt_absolute_time();
	else topic.timestamp_sample = math::min(topic.timestamp_sample - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp_sample);
	buf.offset += sizeof(topic.timestamp_sample);
	static_assert(sizeof(topic.vel_body) == 8, "size mismatch");
	memcpy(&topic.vel_body, buf.iterator, sizeof(topic.vel_body));
	buf.iterator += sizeof(topic.vel_body);
	buf.offset += sizeof(topic.vel_body);
	static_assert(sizeof(topic.vel_ne) == 8, "size mismatch");
	memcpy(&topic.vel_ne, buf.iterator, sizeof(topic.vel_ne));
	buf.iterator += sizeof(topic.vel_ne);
	buf.offset += sizeof(topic.vel_ne);
	static_assert(sizeof(topic.vel_body_filtered) == 8, "size mismatch");
	memcpy(&topic.vel_body_filtered, buf.iterator, sizeof(topic.vel_body_filtered));
	buf.iterator += sizeof(topic.vel_body_filtered);
	buf.offset += sizeof(topic.vel_body_filtered);
	static_assert(sizeof(topic.vel_ne_filtered) == 8, "size mismatch");
	memcpy(&topic.vel_ne_filtered, buf.iterator, sizeof(topic.vel_ne_filtered));
	buf.iterator += sizeof(topic.vel_ne_filtered);
	buf.offset += sizeof(topic.vel_ne_filtered);
	static_assert(sizeof(topic.flow_rate_uncompensated) == 8, "size mismatch");
	memcpy(&topic.flow_rate_uncompensated, buf.iterator, sizeof(topic.flow_rate_uncompensated));
	buf.iterator += sizeof(topic.flow_rate_uncompensated);
	buf.offset += sizeof(topic.flow_rate_uncompensated);
	static_assert(sizeof(topic.flow_rate_compensated) == 8, "size mismatch");
	memcpy(&topic.flow_rate_compensated, buf.iterator, sizeof(topic.flow_rate_compensated));
	buf.iterator += sizeof(topic.flow_rate_compensated);
	buf.offset += sizeof(topic.flow_rate_compensated);
	static_assert(sizeof(topic.gyro_rate) == 12, "size mismatch");
	memcpy(&topic.gyro_rate, buf.iterator, sizeof(topic.gyro_rate));
	buf.iterator += sizeof(topic.gyro_rate);
	buf.offset += sizeof(topic.gyro_rate);
	static_assert(sizeof(topic.gyro_bias) == 12, "size mismatch");
	memcpy(&topic.gyro_bias, buf.iterator, sizeof(topic.gyro_bias));
	buf.iterator += sizeof(topic.gyro_bias);
	buf.offset += sizeof(topic.gyro_bias);
	static_assert(sizeof(topic.ref_gyro) == 12, "size mismatch");
	memcpy(&topic.ref_gyro, buf.iterator, sizeof(topic.ref_gyro));
	buf.iterator += sizeof(topic.ref_gyro);
	buf.offset += sizeof(topic.ref_gyro);
	return true;
}
