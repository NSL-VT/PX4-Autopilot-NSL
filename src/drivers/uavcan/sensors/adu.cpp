/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 * @author Binay Rijal <binaypr@vt.edu>
 * modified from "airspeed.cpp"
*/

#include <drivers/drv_hrt.h>
#include "adu.hpp"
#include <math.h>
#include <lib/geo/geo.h> // For CONSTANTS_*

const char *const UavcanAduBridge::NAME = "adu";

UavcanAduBridge::UavcanAduBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_adu", ORB_ID(sensor_flow_angle)),
	_sub_aoa_data(node),
	_sub_ss_data(node)
{
	for (int i = 0; i < MAX_ADUS; i++) _ss_vals[i] = NAN;
}

int UavcanAduBridge::init()
{
	int res = _sub_aoa_data.start(AOACbBinder(this, &UavcanAduBridge::aoa_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	int res2 = _sub_ss_data.start(SSCbBinder(this, &UavcanAduBridge::ss_sub_cb));

	if (res2 < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res2);
		return res2;
	}
	return 0;
}


void UavcanAduBridge::ss_sub_cb(const
	uavcan::ReceivedDataStructure<uavcan::equipment::air_data::Sideslip> &msg)
{
	/* Store sideslip angle using the node ID as the key */
	uint8_t node_id = msg.getSrcNodeID().get();
	_ss_vals[node_id-1] = msg.sideslip_angle;
}

void UavcanAduBridge::aoa_sub_cb(const
	uavcan::ReceivedDataStructure<uavcan::equipment::air_data::AngleOfAttack> &msg)
{
	uint8_t node_id = msg.getSrcNodeID().get();

	sensor_flow_angle_s report{};

	report.timestamp = hrt_absolute_time();
	report.node_id = node_id;
	report.beta_vane = _ss_vals[node_id-1];
	report.alpha_angle = msg.aoa;

	publish(node_id, &report);
}
