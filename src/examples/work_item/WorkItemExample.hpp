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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/mip_interface.h"
#include "mip_sdk/src/mip/utils/serial_port.h"

using namespace mip::C;

using namespace time_literals;

// bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, timestamp_type* timestamp_out);

class WorkItemExample : public ModuleBase<WorkItemExample>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WorkItemExample();
	~WorkItemExample() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;
	void initialize_cv7();
	void service_cv7();

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	bool _armed{false};
	uint8_t parse_buffer[1024];
	bool _is_initialized{false};
	hrt_abstime _last_print{0};


	/******************************/
	uint8_t _state{0};
	mip::C::mip_interface device;
	//Sensor-to-vehicle frame transformation (Euler Angles)
	float sensor_to_vehicle_transformation_euler[3] = {0.0, 0.0, 0.0};

	// Handlers
	mip_dispatch_handler sensor_data_handlers[5];
	mip_dispatch_handler filter_data_handlers[4];

	//Device data stores
	mip_shared_reference_timestamp_data sensor_reference_time;
	mip_shared_gps_timestamp_data sensor_gps_time;
	mip_sensor_scaled_accel_data  sensor_accel;
	mip_sensor_scaled_gyro_data   sensor_gyro;
	mip_sensor_scaled_mag_data    sensor_mag;

	mip_filter_timestamp_data filter_time;
	mip_filter_status_data        filter_status;
	mip_filter_euler_angles_data  filter_euler_angles;

	bool filter_state_ahrs = false;

};
