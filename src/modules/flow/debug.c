/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
#include "no_warnings.h"
#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "utils.h"
#include "settings.h"
#include "debug.h"

typedef enum
{
  DEBUG_STRING,
  DEBUG_INT,
  DEBUG_FLOAT
} DebugMsgType_TypeDef;

/* send buffers */
static DebugMsgType_TypeDef m_debug_buf_type[DEBUG_COUNT];
static char const * m_debug_buf_pointer[DEBUG_COUNT]; //string pointer buffer
static int32_t m_debug_buf_int[DEBUG_COUNT];
static float m_debug_buf_float[DEBUG_COUNT];

static uint8_t m_debug_index_write = 0;
static uint8_t m_debug_index_read = 0;
static uint16_t m_debug_count = 0;
static bool m_debug_was_full = false;

/**
 * @brief Add a debug message to the send buffer
 * @note This function is interrupt service routine (ISR) SAFE, as it only access RAM.
 *
 * @param string Message
 * @return 1 if successfully added, 0 else
 */
static uint8_t debug_message_buffer(const char* string)
{
	if (m_debug_index_read - m_debug_index_write == 1 || (m_debug_index_read
			== 0 && m_debug_index_write == DEBUG_COUNT - 1))
	{
		/* buffer full, can't send */
		m_debug_was_full = true;
		return 0;
	}
	m_debug_index_write = (m_debug_index_write + 1) % DEBUG_COUNT;
	m_debug_count++;

	/* buffer pointer to string in program code */
	m_debug_buf_pointer[m_debug_index_write] = string;

	return 1;
}

/**
 * @brief Add a string debug message to the send buffer
 *
 * @param string Message
 * @return 1 if successfully added, 0 else
 */
uint8_t debug_string_message_buffer(const char* string)
{
	if (debug_message_buffer(string))
	{
		/* Could write, save message to buffer */
		m_debug_buf_type[m_debug_index_write] = DEBUG_STRING;
		return 1;
	}
	else
	{
		/* Could not write, do nothing */
		return 0;
	}
}

/**
 * @brief Add a string-integer debug message to the send buffers
 *
 * @param string Message
 * @return 1 if successfully added, 0 else
 */
uint8_t debug_int_message_buffer(const char* string, int32_t num)
{
	if (debug_message_buffer(string))
	{
		/* Could write, save integer to buffer */
		m_debug_buf_int[m_debug_index_write] = num;
		m_debug_buf_type[m_debug_index_write] = DEBUG_INT;
		return 1;
	}
	else
	{
		/* Could not write, do nothing */
		return 0;
	}
}

/**
 * @brief Add a string-float debug message to the send buffers
 *
 * @param string Message
 * @return 1 if successfully added, 0 else
 */
uint8_t debug_float_message_buffer(const char* string, float num)
{
	if (debug_message_buffer(string))
	{
		/* Could write, save float to buffer */
		m_debug_buf_float[m_debug_index_write] = num;
		m_debug_buf_type[m_debug_index_write] = DEBUG_FLOAT;
		return 1;
	}
	else
	{
		/* Could not write, do nothing */
		return 0;
	}
}

/**
 * @brief Send one of the buffered messages
 */
void debug_message_send_one(void)
{
	if (m_debug_index_write == m_debug_index_read)
	{
		/* buffer empty */
		return;
	}
	m_debug_index_read = (m_debug_index_read + 1) % DEBUG_COUNT;

	char msg[DEBUG_MAX_LEN] = {};

	switch(m_debug_buf_type[m_debug_index_read])
	{
		case(DEBUG_STRING):
			strncpy(msg, m_debug_buf_pointer[m_debug_index_read], DEBUG_MAX_LEN);
			break;

		case(DEBUG_INT):
			strncat(msg, m_debug_buf_pointer[m_debug_index_read], DEBUG_MAX_LEN);
			strncat(msg, " ", DEBUG_MAX_LEN);
			strncat(msg, flow_ftoa(m_debug_buf_int[m_debug_index_read]), DEBUG_MAX_LEN);
			msg[strlen(msg) - 2] = '\0'; // TODO workaround: cut ".0" of float
			break;

		case(DEBUG_FLOAT):
			strncat(msg, m_debug_buf_pointer[m_debug_index_read], DEBUG_MAX_LEN);
			strncat(msg, " ", DEBUG_MAX_LEN);
			strncat(msg, flow_ftoa(m_debug_buf_float[m_debug_index_read]), DEBUG_MAX_LEN);
			break;

		default:
			return;
	}


	if(m_debug_was_full)
	{
		msg[0] = '+';
		m_debug_was_full = false;
	}

	msg[DEBUG_MAX_LEN - 1] = '\0'; //enforce string termination

	mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, msg);

	if (FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_DEBUG]))
		mavlink_msg_statustext_send(MAVLINK_COMM_2, 0, msg);
}

