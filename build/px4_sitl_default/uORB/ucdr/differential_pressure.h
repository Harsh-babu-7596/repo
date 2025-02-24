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
#include <uORB/topics/differential_pressure.h>


static inline constexpr int ucdr_topic_size_differential_pressure()
{
	return 32;
}

static inline bool ucdr_serialize_differential_pressure(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const differential_pressure_s& topic = *static_cast<const differential_pressure_s*>(data);
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
	static_assert(sizeof(topic.device_id) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.device_id, sizeof(topic.device_id));
	buf.iterator += sizeof(topic.device_id);
	buf.offset += sizeof(topic.device_id);
	static_assert(sizeof(topic.differential_pressure_pa) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.differential_pressure_pa, sizeof(topic.differential_pressure_pa));
	buf.iterator += sizeof(topic.differential_pressure_pa);
	buf.offset += sizeof(topic.differential_pressure_pa);
	static_assert(sizeof(topic.temperature) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.temperature, sizeof(topic.temperature));
	buf.iterator += sizeof(topic.temperature);
	buf.offset += sizeof(topic.temperature);
	static_assert(sizeof(topic.error_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.error_count, sizeof(topic.error_count));
	buf.iterator += sizeof(topic.error_count);
	buf.offset += sizeof(topic.error_count);
	return true;
}

static inline bool ucdr_deserialize_differential_pressure(ucdrBuffer& buf, differential_pressure_s& topic, int64_t time_offset = 0)
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
	static_assert(sizeof(topic.device_id) == 4, "size mismatch");
	memcpy(&topic.device_id, buf.iterator, sizeof(topic.device_id));
	buf.iterator += sizeof(topic.device_id);
	buf.offset += sizeof(topic.device_id);
	static_assert(sizeof(topic.differential_pressure_pa) == 4, "size mismatch");
	memcpy(&topic.differential_pressure_pa, buf.iterator, sizeof(topic.differential_pressure_pa));
	buf.iterator += sizeof(topic.differential_pressure_pa);
	buf.offset += sizeof(topic.differential_pressure_pa);
	static_assert(sizeof(topic.temperature) == 4, "size mismatch");
	memcpy(&topic.temperature, buf.iterator, sizeof(topic.temperature));
	buf.iterator += sizeof(topic.temperature);
	buf.offset += sizeof(topic.temperature);
	static_assert(sizeof(topic.error_count) == 4, "size mismatch");
	memcpy(&topic.error_count, buf.iterator, sizeof(topic.error_count));
	buf.iterator += sizeof(topic.error_count);
	buf.offset += sizeof(topic.error_count);
	return true;
}
