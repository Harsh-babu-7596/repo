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
#include <uORB/topics/estimator_bias3d.h>


static inline constexpr int ucdr_topic_size_estimator_bias3d()
{
	return 80;
}

static inline bool ucdr_serialize_estimator_bias3d(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const estimator_bias3d_s& topic = *static_cast<const estimator_bias3d_s*>(data);
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
	static_assert(sizeof(topic.bias) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.bias, sizeof(topic.bias));
	buf.iterator += sizeof(topic.bias);
	buf.offset += sizeof(topic.bias);
	static_assert(sizeof(topic.bias_var) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.bias_var, sizeof(topic.bias_var));
	buf.iterator += sizeof(topic.bias_var);
	buf.offset += sizeof(topic.bias_var);
	static_assert(sizeof(topic.innov) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.innov, sizeof(topic.innov));
	buf.iterator += sizeof(topic.innov);
	buf.offset += sizeof(topic.innov);
	static_assert(sizeof(topic.innov_var) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.innov_var, sizeof(topic.innov_var));
	buf.iterator += sizeof(topic.innov_var);
	buf.offset += sizeof(topic.innov_var);
	static_assert(sizeof(topic.innov_test_ratio) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.innov_test_ratio, sizeof(topic.innov_test_ratio));
	buf.iterator += sizeof(topic.innov_test_ratio);
	buf.offset += sizeof(topic.innov_test_ratio);
	return true;
}

static inline bool ucdr_deserialize_estimator_bias3d(ucdrBuffer& buf, estimator_bias3d_s& topic, int64_t time_offset = 0)
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
	static_assert(sizeof(topic.bias) == 12, "size mismatch");
	memcpy(&topic.bias, buf.iterator, sizeof(topic.bias));
	buf.iterator += sizeof(topic.bias);
	buf.offset += sizeof(topic.bias);
	static_assert(sizeof(topic.bias_var) == 12, "size mismatch");
	memcpy(&topic.bias_var, buf.iterator, sizeof(topic.bias_var));
	buf.iterator += sizeof(topic.bias_var);
	buf.offset += sizeof(topic.bias_var);
	static_assert(sizeof(topic.innov) == 12, "size mismatch");
	memcpy(&topic.innov, buf.iterator, sizeof(topic.innov));
	buf.iterator += sizeof(topic.innov);
	buf.offset += sizeof(topic.innov);
	static_assert(sizeof(topic.innov_var) == 12, "size mismatch");
	memcpy(&topic.innov_var, buf.iterator, sizeof(topic.innov_var));
	buf.iterator += sizeof(topic.innov_var);
	buf.offset += sizeof(topic.innov_var);
	static_assert(sizeof(topic.innov_test_ratio) == 12, "size mismatch");
	memcpy(&topic.innov_test_ratio, buf.iterator, sizeof(topic.innov_test_ratio));
	buf.iterator += sizeof(topic.innov_test_ratio);
	buf.offset += sizeof(topic.innov_test_ratio);
	return true;
}
