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
#include <uORB/topics/rate_ctrl_status.h>


static inline constexpr int ucdr_topic_size_rate_ctrl_status()
{
	return 24;
}

static inline bool ucdr_serialize_rate_ctrl_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const rate_ctrl_status_s& topic = *static_cast<const rate_ctrl_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.rollspeed_integ) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.rollspeed_integ, sizeof(topic.rollspeed_integ));
	buf.iterator += sizeof(topic.rollspeed_integ);
	buf.offset += sizeof(topic.rollspeed_integ);
	static_assert(sizeof(topic.pitchspeed_integ) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pitchspeed_integ, sizeof(topic.pitchspeed_integ));
	buf.iterator += sizeof(topic.pitchspeed_integ);
	buf.offset += sizeof(topic.pitchspeed_integ);
	static_assert(sizeof(topic.yawspeed_integ) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.yawspeed_integ, sizeof(topic.yawspeed_integ));
	buf.iterator += sizeof(topic.yawspeed_integ);
	buf.offset += sizeof(topic.yawspeed_integ);
	static_assert(sizeof(topic.wheel_rate_integ) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.wheel_rate_integ, sizeof(topic.wheel_rate_integ));
	buf.iterator += sizeof(topic.wheel_rate_integ);
	buf.offset += sizeof(topic.wheel_rate_integ);
	return true;
}

static inline bool ucdr_deserialize_rate_ctrl_status(ucdrBuffer& buf, rate_ctrl_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.rollspeed_integ) == 4, "size mismatch");
	memcpy(&topic.rollspeed_integ, buf.iterator, sizeof(topic.rollspeed_integ));
	buf.iterator += sizeof(topic.rollspeed_integ);
	buf.offset += sizeof(topic.rollspeed_integ);
	static_assert(sizeof(topic.pitchspeed_integ) == 4, "size mismatch");
	memcpy(&topic.pitchspeed_integ, buf.iterator, sizeof(topic.pitchspeed_integ));
	buf.iterator += sizeof(topic.pitchspeed_integ);
	buf.offset += sizeof(topic.pitchspeed_integ);
	static_assert(sizeof(topic.yawspeed_integ) == 4, "size mismatch");
	memcpy(&topic.yawspeed_integ, buf.iterator, sizeof(topic.yawspeed_integ));
	buf.iterator += sizeof(topic.yawspeed_integ);
	buf.offset += sizeof(topic.yawspeed_integ);
	static_assert(sizeof(topic.wheel_rate_integ) == 4, "size mismatch");
	memcpy(&topic.wheel_rate_integ, buf.iterator, sizeof(topic.wheel_rate_integ));
	buf.iterator += sizeof(topic.wheel_rate_integ);
	buf.offset += sizeof(topic.wheel_rate_integ);
	return true;
}
