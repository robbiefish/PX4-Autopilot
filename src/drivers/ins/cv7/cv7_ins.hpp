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
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_selection.h>


#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/mip_interface.h"
#include "mip_sdk/src/mip/utils/serial_port.h"

#include "LogWriter.hpp"

using namespace mip::C;

using namespace time_literals;

// bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, timestamp_type* timestamp_out);

class CvIns : public ModuleBase<CvIns>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CvIns(const char *device, int32_t rotation);
	~CvIns() override;

	static void handleAccel(void *user, const mip_field *field, timestamp_type timestamp);
	static void handleGyro(void *user, const mip_field *field, timestamp_type timestamp);
	static void handleMag(void *user, const mip_field *field, timestamp_type timestamp);
	static void handleBaro(void *user, const mip_field *field, timestamp_type timestamp);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	void set_sensor_rate(mip_descriptor_rate *sensor_descriptors, uint16_t len);

	int connect_at_baud(int32_t baud);

	int print_status() override;

	LogWriter &get_logger()
	{
		return _logger;
	}

	hrt_abstime _time_last_valid_imu_us{0};
private:
	void Run() override;
	void initialize_cv7();
	void service_cv7();
	void initialize_logger();

	enum cv7_mode {
		mode_imu = 0,
		mode_ahrs = 1,
		mode_ins = 2
	};

	struct cv7_configuration {
		enum cv7_mode _selected_mode = mode_imu;
		uint16_t _sens_imu_update_rate_hz = 500;
		uint16_t _sens_other_update_rate_hz = 50;
		enum Rotation _rot = ROTATION_NONE;
		uint32_t _device_id{0};
	};

	cv7_configuration _config;

	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope _px4_gyro{0};
	PX4Magnetometer _px4_mag{0};

	sensor_baro_s _sensor_baro{0};

	// Publications
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules CvIns when updated
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
	LogWriter _logger;
	bool _armed{false};
	uint8_t parse_buffer[1512];
	bool _is_initialized{false};
	hrt_abstime _last_print{0};



	/******************************/
	uint8_t _state{0};
	mip::C::mip_interface device;


	// Handlers
	mip_dispatch_handler sensor_data_handlers[5];
	mip_dispatch_handler filter_data_handlers[4];

	//Device data stores
	mip_shared_reference_timestamp_data sensor_reference_time;
	mip_shared_gps_timestamp_data sensor_gps_time;
	mip_sensor_scaled_accel_data  sensor_accel{0};
	mip_sensor_scaled_gyro_data   sensor_gyro{0};
	mip_sensor_scaled_mag_data    sensor_mag{0};

	mip_filter_timestamp_data     filter_time;
	mip_filter_status_data        filter_status;
	mip_filter_euler_angles_data  filter_euler_angles;

	bool filter_state_ahrs = false;
	const char *_uart_device;

};
