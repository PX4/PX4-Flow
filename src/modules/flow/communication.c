/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
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

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "usbd_cdc_vcp.h"
#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "settings.h"
#include "usart.h"
#include "debug.h"
#include "communication.h"

extern uint32_t get_boot_time_us(void);
extern void systemreset(bool to_bootloader);

mavlink_system_t mavlink_system;

static uint32_t m_parameter_i = 0;

static mavlink_param_union_t mavlink_value(const param_info_item* p) {
	mavlink_param_union_t value;
	switch (p->type) {
		case PARAM_FLOAT:
			value.type = MAV_PARAM_TYPE_REAL32;
			value.param_float = *(float*) p->variable;
			break;
		case PARAM_UINT:
			value.type = MAV_PARAM_TYPE_UINT32;
			value.param_uint32 = *(uint32_t*) p->variable;
			break;
		case PARAM_BOOL:
			value.type = MAV_PARAM_TYPE_UINT32;
			value.param_uint32 = *(bool*) p->variable;
			break;
	}
	return value;
}

/**
 * @brief Initialize MAVLINK system
 */
void communication_init(void)
{
	mavlink_system.sysid = param_system_id; // System ID, 1-255
	mavlink_system.compid = param_component_id; // Component/Subsystem ID, 1-255
}

/**
 * @brief Send System State
 */
void communication_system_state_send(void)
{
	/* send heartbeat to announce presence of this system */
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, param_system_type, param_autopilot_type, 0, 0, 0);
	mavlink_msg_heartbeat_send(MAVLINK_COMM_2, param_system_type, param_autopilot_type, 0, 0, 0);
}

/**
 * @brief Send one low-priority parameter message
 */
void communication_parameter_send(void)
{
	/* send parameters one by one */
	if (m_parameter_i < params_count)
	{
		const param_info_item* p = &param_info[m_parameter_i];
		mavlink_param_union_t value = mavlink_value(p);
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
			p->name, value.param_float, value.type, params_count, m_parameter_i);
		mavlink_msg_param_value_send(MAVLINK_COMM_2,
			p->name, value.param_float, value.type, params_count, m_parameter_i);
		m_parameter_i++;
	}
}

static int32_t param_index_by_name(const char* name)
{
	for (unsigned i = 0; i < params_count; i++)
	{
		const param_info_item* param = &param_info[i];
		/* Check if matched */
		if (strncmp(name, param->name, 16) == 0)
		{
			return i;
		}
	}
	return -1;
}

void handle_mavlink_message(mavlink_channel_t chan,
		mavlink_message_t* msg)
{
	/* all messages from usart2 are directly forwarded */
	if(chan == MAVLINK_COMM_1)
	{
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		uint32_t len;

		// Copy to USART 3
		len = mavlink_msg_to_send_buffer(buf, msg);
		mavlink_send_uart_bytes(MAVLINK_COMM_0, buf, len);

		if (param_usb_send_forward)
			mavlink_send_uart_bytes(MAVLINK_COMM_2, buf, len);

		return;
	}

	/* forwarded received messages from usb and usart3 to usart2 */
	if(chan == MAVLINK_COMM_0 || chan == MAVLINK_COMM_2)
	{
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		uint32_t len;

		// Copy to USART 2
		len = mavlink_msg_to_send_buffer(buf, msg);
		mavlink_send_uart_bytes(MAVLINK_COMM_1, buf, len);
	}

	/* handling messages */
	switch (msg->msgid)
	{
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		{
			mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			/* Check if this message is for this system */
			if ((uint8_t) set.target_system == (uint8_t) param_system_id
				&& (uint8_t) set.target_component == (uint8_t) param_component_id)
			{
				
				int param_index = set.param_index;
				if (param_index == -1) {
					param_index = param_index_by_name(set.param_id);
				}
				
				if (param_index >= 0 && (unsigned)param_index < params_count) {
					const param_info_item* param = &param_info[param_index];
					mavlink_param_union_t value = mavlink_value(param);
					mavlink_msg_param_value_send(chan, param->name,
						value.param_float, value.type, params_count, param_index);
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
			/* Start sending parameters */
			m_parameter_i = 0;
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET:
		{
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			/* Check if this message is for this system */
			if ((uint8_t) set.target_system == (uint8_t) param_system_id
				&& (uint8_t) set.target_component == (uint8_t) param_component_id)
			{
				int param_index = param_index_by_name(set.param_id);
				
				if (param_index >= 0)
				{
					const param_info_item* param = &param_info[param_index];
					/* Check if matched */
					if (strncmp(set.param_id, param->name, 16) == 0)
					{						 
						mavlink_param_union_t old_value = mavlink_value(param);
						mavlink_param_union_t new_value = { .param_float = set.param_value };
						
						if (param->access && old_value.param_uint32 != new_value.param_uint32)
						{
							switch (param->type) {
								case PARAM_FLOAT:
									*(float*) param->variable = new_value.param_float;
									break;
								case PARAM_UINT:
									*(uint32_t*) param->variable = new_value.param_uint32;
									break;
								case PARAM_BOOL:
									*(bool*) param->variable = new_value.param_uint32;
									break;
							}
							if (param->on_change) param->on_change();
						}
						
						mavlink_param_union_t value = mavlink_value(param);
						mavlink_msg_param_value_send(MAVLINK_COMM_0, param->name,
							value.param_float, value.type, params_count, param_index);
						mavlink_msg_param_value_send(MAVLINK_COMM_2, param->name,
							value.param_float, value.type, params_count, param_index);
					}
				}
			}
		}
		break;

		case MAVLINK_MSG_ID_PING:
		{
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(msg, &ping);
			if (ping.target_system == 0 && ping.target_component == 0)
			{
				/* Respond to ping */
				uint64_t r_timestamp = get_boot_time_us();
				mavlink_msg_ping_send(chan, ping.seq, msg->sysid, msg->compid, r_timestamp);
			}
		}
		break;

		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
			mavlink_command_long_t cmd;
			mavlink_msg_command_long_decode(msg, &cmd);

			switch (cmd.command) {
				case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				if (((int)(cmd.param1)) == 1) {
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_ACCEPTED);
					/* reboot */
					systemreset(false);

				} else if (((int)(cmd.param1)) == 3) {
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_ACCEPTED);
					/* reboot to bootloader */
					systemreset(true);

				} else {
					/* parameters are wrong */
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_FAILED);
					// XXX add INVALID INPUT to MAV_RESULT
				}
				break;

				default:
					mavlink_msg_command_ack_send(chan, cmd.command, MAV_RESULT_UNSUPPORTED);
				break;
			}
		}
		break;

		default:
			break;
	}
}

/**
 * @brief Receive from usart3
 */
void communication_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status = { 0 };

	while (usart3_char_available())
	{
		uint8_t c = usart3_rx_ringbuffer_pop();

		/* Try to get a new message */
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
		{
			/* Handle message */
			handle_mavlink_message(MAVLINK_COMM_0, &msg);
		}
		/* And get the next one */
	}
}

/**
 * @brief Receive from usart2
 */
void communication_receive_forward(void)
{
	mavlink_message_t msg;
	mavlink_status_t status = { 0 };

	while (usart2_char_available())
	{
		uint8_t c = usart2_rx_ringbuffer_pop();

		/* Try to get a new message */
		if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status))
		{
			/* Handle message */
			handle_mavlink_message(MAVLINK_COMM_1, &msg);
		}
		/* And get the next one */
	}
}

/**
 * @brief Receive from usb
 */
void communication_receive_usb(void)
{
	mavlink_message_t msg;
	mavlink_status_t status = { 0 };
	uint8_t character;

	while (VCP_get_char(&character))
	{
		/* Try to get a new message */
		if (mavlink_parse_char(MAVLINK_COMM_2, character, &msg, &status))
		{
			/* Handle message */
			handle_mavlink_message(MAVLINK_COMM_2, &msg);
		}
		/* And get the next one */
	}
}

/**
 * @brief Send multiple chars (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use
 * @param ch Character to send
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length)
{
	if (chan == MAVLINK_COMM_0)
	{
		/* send to UART3 */
		usart3_tx_ringbuffer_push(ch, length);
	}
	if (chan == MAVLINK_COMM_1)
	{
		/* send to UART2 */
		usart2_tx_ringbuffer_push(ch, length);
	}
	if (chan == MAVLINK_COMM_2)
	{
		/* send to USB serial port */
		for (int i = 0; i < length; i++)
		{
			VCP_put_char(ch[i]);
		}
	}
}

/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t* mavlink_get_channel_status(uint8_t channel)
{
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
	return &m_mavlink_status[channel];
}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t* mavlink_get_channel_buffer(uint8_t channel)
{
	static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
	return &m_mavlink_buffer[channel];
}

