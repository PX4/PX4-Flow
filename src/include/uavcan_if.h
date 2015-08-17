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


typedef enum  range_sensor_type_t {
      SENSOR_TYPE_UNDEFINED = 0,
      SENSOR_TYPE_SONAR     = 1,
      SENSOR_TYPE_LIDAR     = 2,
      SENSOR_TYPE_RADAR     = 3
} range_sensor_type_t;

typedef enum  range_sensor_reding_type_t {
      READING_TYPE_UNDEFINED   = 0,   // Range is unknown
      READING_TYPE_VALID_RANGE = 1,   // Range field contains valid distance
      READING_TYPE_TOO_CLOSE   = 2,   // Range field contains min range for the sensor
      READING_TYPE_TOO_FAR     = 3,   // Range field contains max range for the sensor
} range_sensor_reding_type_t;

typedef struct range_data_t {
    uint64_t                   time_stamp_utc;
    int8_t                     roll;
    int8_t                     pitch;
    int8_t                     yaw;
    bool                       orientation_defined;
    uint8_t                    sensor_id;
    float                      field_of_view;
    range_sensor_type_t        sensor_type;
    range_sensor_reding_type_t reading_type;
    float                      range;
} range_data_t;

__BEGIN_DECLS
__EXPORT int uavcannode_run(void);
__EXPORT int uavcannode_publish_flow(legacy_12c_data_t *pdata);
__EXPORT int uavcannode_publish_range(range_data_t *pdata);
__EXPORT int uavcannode_main(bool start_not_stop);
__END_DECLS


#if defined(CONFIG_ARCH_BOARD_PX4FLOW_V2)
#define  uavcan_start() uavcannode_main(true)
#define  uavcan_stop()  uavcannode_main(false)
#define  uavcan_run()   uavcannode_run()
#define  uavcan_publish(what, rate, name) if (++name##_cnt >= rate) { \
                                          name##_cnt = 0; \
                                          (void) uavcannode_publish_##what(&name); \
                                       }
#define  uavcan_export memcpy
#define  uavcan_define_export(name, type, section)  type name; \
                                                    static uint8_t name##_cnt = 0;

#define  uavcan_use_export(name)  &name
#define  uavcan_timestamp_export(name)  name.time_stamp_utc = hrt_absolute_time()
#define  uavcan_assign(lhs, rhs)  lhs = rhs

#else
#define  uavcan_start()
#define  uavcan_stop()
#define  uavcan_run()
#define  uavcan_publish(what, rate, name)
#define  uavcan_export(d,s,l)
#define  uavcan_define_export(name, type, section)
#define  uavcan_use_export(name) NULL
#define  uavcan_timestamp_export(name)
#define  uavcan_assign(lhs, rhs)
#endif
