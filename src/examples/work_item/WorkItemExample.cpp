/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "WorkItemExample.hpp"
#include "mip_sdk/src/mip/mip_parser.h"


serial_port device_port;
// serial_port *device_port_ptr;

////////////////////////////////////////////////////////////////////////////////
// MIP Interface Time Access Function
////////////////////////////////////////////////////////////////////////////////

timestamp_type get_current_timestamp()
{
	return hrt_absolute_time();
//     clock_t curr_time;
//     curr_time = clock();

//     return (timestamp_type)((double)(curr_time - start_time)/(double)CLOCKS_PER_SEC*1000.0);
}

////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Recv Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip::C::mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, timestamp_type* timestamp_out)
{
    (void)device;

    *timestamp_out = get_current_timestamp();
    if( !serial_port_read(&device_port, buffer, max_length, out_length) )
        return false;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Send Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip::C::mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length)
{
	size_t bytes_written;
	for (size_t i = 0; i < length; i++)
	{
		PX4_INFO("%d",data[i]);
	}

    return serial_port_write(&device_port, data, length, &bytes_written);
}

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{

}

WorkItemExample::~WorkItemExample()
{
	if(serial_port_is_open(&device_port))
        	serial_port_close(&device_port);
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	// // execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	ScheduleOnInterval(100_ms);

	return true;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if(!_is_initialized){
		if(!serial_port_open(&device_port, "/dev/ttyS2", 57600))
			PX4_ERR("ERROR: Could not open device port!");

		//
		//Initialize the MIP interface
		//

		mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(57600), 1000000);

		//
		//Ping the device (note: this is good to do to make sure the device is present)
		//

		if(mip_base_ping(&device) != MIP_ACK_OK){
			PX4_WARN("Couldn't connect to device");
		}
		PX4_INFO("MIP_Size %d",device._max_update_pkts);

		_is_initialized = true;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	// if(mip_base_ping(&device) != MIP_ACK_OK){
	// 	PX4_WARN("Couldn't connect to device");
	// }
	// PX4_INFO("MIP_Size %d",device._max_update_pkts);

	unsigned int bytes_written{0};
	// serial_port_write(&device_port,"Hello World\n",13,&bytes_written);

	char buf[24];
	size_t num_bytes;
	if(serial_port_read(&device_port, buf,24,&num_bytes)){
		serial_port_write(&device_port,buf,num_bytes,&bytes_written);
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}


	// Example
	//  grab latest accelerometer data
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}


	// Example
	//  publish some data
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);


	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemExample::print_status()
{
	PX4_INFO_RAW("Serial Port Open %d Handle %d\n",device_port.is_open, device_port.handle);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
