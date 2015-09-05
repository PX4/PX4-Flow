/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_config.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/equipment/range_sensor/Measurement.hpp>
#include <threedr/equipment/flow/optical_flow/LegacyRawSample.hpp>
#include <uavcan/node/timer.hpp>

#include "uavcan_if.h"

/**
 * @file uavcan_main.hpp
 *
 * Defines basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

/**
 * A UAVCAN node.
 */
class UavcanNode
{
	/*
	 * This memory is reserved for uavcan to use as over flow for message
	 * Coming from multiple sources that my not be considered at development
	 * time.
	 *
	 * The call to getNumFreeBlocks will tell how many blocks there are
	 * free -and multiply it times getBlockSize to get the number of bytes
	 *
	 */
	static constexpr unsigned MemPoolSize        = 2048;

	/*
	 * This memory is reserved for uavcan to use for queuing CAN frames.
	 * At 1Mbit there is approximately one CAN frame every 200 uS.
	 * The number of buffers sets how long you can go without calling
	 * node_spin_xxxx. Since our task is the only one running and the
	 * driver will light the fd when there is a CAN frame we can nun with
	 * a minimum number of buffers to conserver memory. Each buffer is
	 * 32 bytes. So 5 buffers costs 160 bytes and gives us a maximum required
	 * poll rate of ~1 mS
	 *
	 */
	static constexpr unsigned RxQueueLenPerIface = 5;

	/*
	 * This memory is uses for the tasks stack size
	 */

        static constexpr unsigned StackSize          = 2500;

public:
	typedef uavcan::Node<MemPoolSize> Node;
	typedef uavcan_stm32::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	typedef uavcan::protocol::file::BeginFirmwareUpdate BeginFirmwareUpdate;

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

        static int      start(uavcan::NodeID node_id, uint32_t bitrate);
        int             stop();

	Node		&get_node() { return _node; }

	static UavcanNode *instance() { return _instance; }

        int             run();
        int             publish(legacy_12c_data_t *pdata);
        int             publish(range_data_t *pdata);

	/* The bit rate that can be passed back to the bootloader */

	int32_t active_bitrate;


private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id);
        int             teardown();

	bool			_task_should_exit = false;	///< flag to indicate to tear down the CAN driver

	static UavcanNode	*_instance;			///< singleton pointer
	Node			_node;				///< library instance

	typedef uavcan::MethodBinder<UavcanNode*,
	    void (UavcanNode::*) (const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request> &,
	     uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &)>
	    BeginFirmwareUpdateCallBack;

	uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateCallBack> _fw_update_listner;
        uavcan::GlobalTimeSyncSlave _time_sync_slave;
        uavcan::Publisher<::threedr::equipment::flow::optical_flow::LegacyRawSample> _flow_pulisher;
        uavcan::Publisher<uavcan::equipment::range_sensor::Measurement> _range_pulisher;
	void cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request> &req,
	                             uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp);

public:

	/* A timer used to reboot after the response is sent */

	uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent &)> _reset_timer;

};
