/****************************************************************************
 *
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

#include <px4_config.h>

#include <px4_log.h>

#include <bsp/board.h>

#include "uavcannode_main.hpp"
#include "boot_app_shared.h"

#include <px4_macros.h>

#define FW_GIT STRINGIFY(GIT_VERSION)

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */


/*
 * This is the AppImageDescriptor used
 * by the make_can_boot_descriptor.py tool to set
 * the application image's descriptor so that the
 * uavcan bootloader has the ability to validate the
 * image crc, size etc of this application
*/

boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = {APP_DESCRIPTOR_SIGNATURE},
	.image_crc = 0,
	.image_size = 0,
	.vcs_commit = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.reserved = {0xff , 0xff , 0xff , 0xff , 0xff , 0xff }
};


__EXPORT uint64_t hrt_absolute_time(void)
{
  return uavcan_stm32::clock::getUtc().toUSec();
}

static UavcanNode::CanInitHelper can;

static UavcanNode * newNode()
{
  static UavcanNode node(can.driver, uavcan_stm32::SystemClock::instance());
  return &node;
}

/*
 * UavcanNode
 */
UavcanNode *UavcanNode::_instance;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	active_bitrate(0),
	_node(can_driver, system_clock),
	_fw_update_listner(_node),
	_time_sync_slave(_node),
	_flow_pulisher(_node),
	_range_pulisher(_node),
	_reset_timer(_node)
{

}

UavcanNode::~UavcanNode()
{
	_instance = nullptr;

}

int UavcanNode::stop()
{
    _instance = nullptr;
    return PX4_OK;
}

int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{


	if (_instance != nullptr) {
		PX4_INFO("Already started");
		return -1;
	}

	/*
	 * CAN driver init
	 */
	static bool can_initialized = false;

	if (!can_initialized) {

	    // Confgure the HW IO

	    board_initialize();

	    const int can_init_res = can.init(bitrate);

		if (can_init_res < 0) {
		        PX4_ERR("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}

		can_initialized = true;
	}

	/*
	 * Node init
	 */
	_instance = newNode();

	const int node_init_res = _instance->init(node_id);

	if (node_init_res < 0) {
		_instance = nullptr;
		PX4_ERR("Node init failed %i", node_init_res);
		return node_init_res;
	}


	/* Keep the bit rate for reboots on BenginFirmware updates */

	_instance->active_bitrate = bitrate;

	return PX4_OK;
}

void UavcanNode::fill_node_info()
{
	/* software version */
	uavcan::protocol::SoftwareVersion swver;

	// Extracting the first 8 hex digits of FW_GIT and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, FW_GIT, 8);
	assert(fw_git_short[8] == '\0');
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	swver.major = AppDescriptor.major_version;
	swver.minor = AppDescriptor.minor_version;
	swver.image_crc = AppDescriptor.image_crc;

	PX4_INFO("SW version vcs_commit: 0x%08x", unsigned(swver.vcs_commit));

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;

	hwver.major = HW_VERSION_MAJOR;
	hwver.minor = HW_VERSION_MINOR;

	uint8_t udid[BOARD_SERIALNUMBER_SIZE] = {};
	board_get_serialnumber(udid);
	uavcan::copy(udid, udid + sizeof(udid), hwver.unique_id.begin());

	_node.setHardwareVersion(hwver);
}

static void cb_reboot(const uavcan::TimerEvent &)
{
    board_reset(false);
}

void UavcanNode::cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request>
		&req,
		uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp)
{
	static bool inprogress = false;

	rsp.error = rsp.ERROR_UNKNOWN;

	if (req.image_file_remote_path.path.size()) {
		rsp.error = rsp.ERROR_IN_PROGRESS;

		if (!inprogress) {
			inprogress = true;
			bootloader_app_shared_t shared;
			shared.bus_speed = active_bitrate;
			shared.node_id = _node.getNodeID().get();
			bootloader_app_shared_write(&shared, App);
			board_led_rgb(255, 128 , 0 , 5);
			_reset_timer.setCallback(cb_reboot);
			_reset_timer.startOneShotWithDelay(uavcan::MonotonicDuration::fromMSec(1000));
			rsp.error = rsp.ERROR_OK;
		}
	}
}

int UavcanNode::init(uavcan::NodeID node_id)
{
	int ret = -1;

	_node.setName(UAVCAN_NAME);

	_node.setNodeID(node_id);

	fill_node_info();

	const int srv_start_res = _fw_update_listner.start(BeginFirmwareUpdateCallBack(this,
				  &UavcanNode::cb_beginfirmware_update));

	if (srv_start_res < 0) {
		return ret;
	}

	return _node.start();
}


/*
 * Restart handler
 */
class RestartRequestHandler: public uavcan::IRestartRequestHandler
{
	bool handleRestartRequest(uavcan::NodeID request_source) override
	{
		PX4_INFO("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
		board_reset(false);
		return true; // Will never be executed BTW
	}
} restart_request_handler;



int UavcanNode::publish(range_data_t *pdata)
{
  ::uavcan::equipment::range_sensor::Measurement m;
  m.timestamp.usec = pdata->time_stamp_utc;
  m.sensor_id = pdata->sensor_id;
  m.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[0] = pdata->roll  * m.beam_orientation_in_body_frame.ANGLE_MULTIPLIER;
  m.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1] = pdata->pitch * m.beam_orientation_in_body_frame.ANGLE_MULTIPLIER;
  m.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[2] = pdata->yaw   * m.beam_orientation_in_body_frame.ANGLE_MULTIPLIER;
  m.beam_orientation_in_body_frame.orientation_defined = true;
  m.field_of_view = pdata->field_of_view;
  m.sensor_type = pdata->sensor_type;
  m.reading_type= pdata->reading_type;
  m.range = pdata->range;
  _range_pulisher.broadcast(m);
  return PX4_OK;
}
int UavcanNode::publish(legacy_12c_data_t *pdata)
{
  ::threedr::equipment::flow::optical_flow::LegacyRawSample r;
  r.time.usec = pdata->time_stamp_utc;
  r.frame.frame_count = pdata->frame.frame_count;
  r.frame.pixel_flow_x_sum = pdata->frame.pixel_flow_x_sum;
  r.frame.pixel_flow_y_sum = pdata->frame.pixel_flow_y_sum;
  r.frame.flow_comp_m_x = pdata->frame.flow_comp_m_x;
  r.frame.flow_comp_m_y = pdata->frame.flow_comp_m_y;
  r.frame.qual = pdata->frame.qual;
  r.frame.gyro_x_rate = pdata->frame.gyro_x_rate;
  r.frame.gyro_y_rate = pdata->frame.gyro_y_rate;
  r.frame.gyro_z_rate = pdata->frame.gyro_z_rate;
  r.frame.gyro_range = pdata->frame.gyro_range;
  r.integral.frame_count_since_last_readout = pdata->integral_frame.frame_count_since_last_readout;
  r.integral.pixel_flow_x_integral = pdata->integral_frame.pixel_flow_x_integral;
  r.integral.pixel_flow_y_integral = pdata->integral_frame.pixel_flow_y_integral;
  r.integral.gyro_x_rate_integral = pdata->integral_frame.gyro_x_rate_integral;
  r.integral.gyro_y_rate_integral = pdata->integral_frame.gyro_y_rate_integral;
  r.integral.gyro_z_rate_integral = pdata->integral_frame.gyro_z_rate_integral;
  r.integral.integration_timespan = pdata->integral_frame.integration_timespan;
  r.integral.sonar_timestamp = pdata->integral_frame.sonar_timestamp;
  r.integral.ground_distance = pdata->integral_frame.ground_distance;
  r.integral.gyro_temperature = pdata->integral_frame.gyro_temperature;
  r.integral.qual = pdata->integral_frame.qual;
  _flow_pulisher.broadcast(r);
  return PX4_OK;

}


int UavcanNode::run()
{
        static bool once = false;
        int rv = 0;
        if (!once) {
          once = true;
          get_node().setRestartRequestHandler(&restart_request_handler);
          const int slave_init_res = _time_sync_slave.start();
          if (slave_init_res < 0) {
              PX4_INFO("Failed to start time_sync_slave");
            _task_should_exit = true;
          }

          _node.setModeOperational();
        }

        if (!_task_should_exit) {
            rv = _node.spinOnce();
        } else {
            teardown();
            PX4_INFO("exiting.");
            rv = 0;
        }

	return rv;
}

int
UavcanNode::teardown()
{
	return 0;
}


/*
 * App entry point
 */
static void print_usage()
{
	PX4_INFO("usage: \n"
	      "\tuavcannode {start|status|stop|arm|disarm}");
}

static int uavcannode_start()
{
        board_app_initialize();

	// CAN bitrate
	int32_t bitrate = 0;
	// Node ID
	int32_t node_id = 0;

	// Did the bootloader auto baud and get a node ID Allocated

	bootloader_app_shared_t shared;
	int valid  = bootloader_app_shared_read(&shared, BootLoader);

	if (valid == 0) {

		bitrate = shared.bus_speed;
		node_id = shared.node_id;

		// Invalidate to prevent deja vu

		bootloader_app_shared_invalidate();

	} else {
TODO(Need non vol Paramter sotrage)
		// Node ID
		node_id = 123;
		bitrate = 1000000;
	}

	if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
		PX4_INFO("Invalid Node ID %li", node_id);
		return 1;
	}

	// Start
	PX4_INFO("Node ID %lu, bitrate %lu", node_id, bitrate);
	return UavcanNode::start(node_id, bitrate);
}

__EXPORT int uavcannode_publish_flow(legacy_12c_data_t *pdata)
{

  UavcanNode *const inst = UavcanNode::instance();

    if (!inst) {
            PX4_ERR( "application not running");
            return 1;
    }
    return inst->publish(pdata);
}

__EXPORT int uavcannode_publish_range(range_data_t *pdata)
{

  UavcanNode *const inst = UavcanNode::instance();

    if (!inst) {
            PX4_ERR( "application not running");
            return 1;
    }
    return inst->publish(pdata);
}

__EXPORT int uavcannode_run()
{
  UavcanNode *const inst = UavcanNode::instance();

  if (!inst) {
          PX4_ERR( "application not running");
          return 1;
  }
  return inst->run();

}


int uavcannode_main(bool start_not_stop)
{
	if (start_not_stop) {

		if (UavcanNode::instance()) {
		    PX4_ERR("already started");
		    return 1;
		}

		return uavcannode_start();
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
                PX4_ERR( "application not running");
                return 1;
	}


	if (!start_not_stop) {
		inst->stop();
                return 0;
	}

	print_usage();
        return 1;
}
