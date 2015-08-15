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

#pragma once

#include <px4_config.h>
#include <visibility.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c_frame.h"
#include "hrt.h"


typedef struct legacy_12c_data_t {
    uint64_t    time_stamp_utc;
    i2c_frame   frame;
    i2c_integral_frame integral_frame;
} legacy_12c_data_t;

__BEGIN_DECLS
__EXPORT int uavcannode_run(void);
__EXPORT int uavcannode_publish(legacy_12c_data_t *pdata);
__EXPORT int uavcannode_main(bool start_not_stop);
__END_DECLS


#if defined(CONFIG_ARCH_BOARD_PX4FLOW_V2)
#define  uavcan_start() uavcannode_main(true)
#define  uavcan_stop()  uavcannode_main(false)
#define  uavcan_run()   uavcannode_run()
#define  uavcan_publish(d) uavcannode_publish(d)
#define  uavcan_export memcpy
#define  uavcan_define_export(name , section)  legacy_12c_data_t name
#define  uavcan_use_export(name)  &name
#define  uavcan_timestamp_export(name)  name.time_stamp_utc = hrt_absolute_time()
#else
#define  uavcan_start()
#define  uavcan_stop()
#define  uavcan_run()
#define  uavcan_publish(d)
#define  uavcan_export(d,s,l)
#define  uavcan_define_export(name, section)
#define  uavcan_use_export(name) NULL
#define  uavcan_timestamp_export(name)
#endif
