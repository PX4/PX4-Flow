#include <px4_config.h>
#include <visibility.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c_frame.h"
#include "hrt.h"
#include "fmu_comm.h"
#include "result_accumulator.h"
#include "uavcan_if.h"

static result_accumulator_ctx accumulator;
static legacy_12c_data_t i2c_data;
static range_data_t range_data;

__EXPORT void fmu_comm_init() {
  uavcannode_main(true);
  result_accumulator_init(&accumulator);
}

__EXPORT void fmu_comm_run() {
  uavcannode_run();
}

__EXPORT void fmu_comm_update(float dt, float x_rate, float y_rate, float z_rate, int16_t gyro_temp,
  uint8_t qual, float pixel_flow_x, float pixel_flow_y, float rad_per_pixel,
  bool distance_valid, float ground_distance, uint32_t distance_age) {
  range_data.time_stamp_utc = i2c_data.time_stamp_utc = hrt_absolute_time();
  
  /* feed the accumulator and recalculate */
  result_accumulator_feed(&accumulator, dt, x_rate, y_rate, z_rate, gyro_temp,
              qual, pixel_flow_x, pixel_flow_y, rad_per_pixel, 
              distance_valid, ground_distance, distance_age);
    
  if (accumulator.frame_count >= 40) {
    result_accumulator_fill_i2c_data(&accumulator, &i2c_data.frame, &i2c_data.integral_frame);
    // TODO: fill range_data
    uavcannode_publish_flow(&i2c_data);
    uavcannode_publish_range(&range_data);
    result_accumulator_reset(&accumulator);
  }
}