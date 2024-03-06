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

#include <lib/drivers/device/Device.hpp>

#include "cv7_ins.hpp"
#include "mip_sdk/src/mip/mip_parser.h"
#include "CircularBuffer.hpp"

// #define LOG_TRANSACTIONS

static CvIns *cv7_ins{nullptr};

serial_port device_port;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;


void CvIns::handleAccel(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_accel_data data;

	if (extract_mip_sensor_scaled_accel_data_from_field(field, &data)) {
		PX4_DEBUG("Accel Data: %f %f %f", (double)data.scaled_accel[0], (double)data.scaled_accel[1],
			  (double)data.scaled_accel[2]);
		ref->_px4_accel.update(timestamp, data.scaled_accel[0]*CONSTANTS_ONE_G, data.scaled_accel[1]*CONSTANTS_ONE_G,
				       data.scaled_accel[2]*CONSTANTS_ONE_G);
		ref->_time_last_valid_imu_us = timestamp;
	}
}

void CvIns::handleGyro(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_gyro_data data;

	if (extract_mip_sensor_scaled_gyro_data_from_field(field, &data)) {
		PX4_DEBUG("Gyro Data:  %f, %f, %f", (double)data.scaled_gyro[0], (double)data.scaled_gyro[1],
			  (double)data.scaled_gyro[2]);
		ref->_px4_gyro.update(timestamp, data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
		ref->_time_last_valid_imu_us = timestamp;
	}
}

void CvIns::handleMag(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_mag_data data;

	if (extract_mip_sensor_scaled_mag_data_from_field(field, &data)) {
		PX4_DEBUG("Mag Data:   %f, %f, %f", (double)data.scaled_mag[0], (double)data.scaled_mag[1], (double)data.scaled_mag[2]);
		ref->_px4_mag.update(timestamp, data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
	}
}

void CvIns::handleBaro(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_pressure_data data;

	if (extract_mip_sensor_scaled_pressure_data_from_field(field, &data)) {
		PX4_DEBUG("Baro Data:   %f", (double)data.scaled_pressure);
		ref->_sensor_baro.timestamp = timestamp;
		ref->_sensor_baro.timestamp_sample = timestamp;
		ref->_sensor_baro.pressure = data.scaled_pressure * 100.f; // convert [Pa] to [mBar]
		ref->_sensor_baro_pub.publish(ref->_sensor_baro);
	}
}

void handle_filter_event_source(void *user, const mip_field *field, timestamp_type timestamp)
{
	mip_shared_event_source_data data;

	if (extract_mip_shared_event_source_data_from_field(field, &data)) {
		if (data.trigger_id == FILTER_ROLL_EVENT_ACTION_ID) {
			PX4_WARN("WARNING: Roll event triggered!\n");

		} else if (data.trigger_id == FILTER_PITCH_EVENT_ACTION_ID) {
			PX4_WARN("WARNING: Pitch event triggered!\n");
		}
	}
}

timestamp_type get_current_timestamp()
{
	return hrt_absolute_time();
}

bool mip::C::mip_interface_user_recv_from_device(mip_interface *device, uint8_t *buffer, size_t max_length,
		size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = get_current_timestamp();

	if (!serial_port_read(&device_port, buffer, max_length, out_length)) {
		return false;
	}

#ifdef LOG_TRANSACTIONS

	if (cv7_ins) {
		cv7_ins->get_logger().enqueue_rx(buffer, *out_length);
	}

#endif

	return true;
}

bool mip::C::mip_interface_user_send_to_device(mip_interface *device, const uint8_t *data, size_t length)
{
	size_t bytes_written;

#ifdef LOG_TRANSACTIONS

	if (cv7_ins) {
		cv7_ins->get_logger().enqueue_tx(data, length);
	}

#endif

	return serial_port_write(&device_port, data, length, &bytes_written);
}

void CvIns::exit_gracefully(const char *msg)
{
	PX4_ERR("%s: Not Stopping Application", msg);
	// //Close com port
	// if(serial_port_is_open(&device_port))
	// 	serial_port_close(&device_port);
}

CvIns::CvIns() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	loadRotation();

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_3DMCV7;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	_config._device_id = device_id.devid;
	_px4_accel.set_device_id(_config._device_id);
	_px4_gyro.set_device_id(_config._device_id);
	_px4_mag.set_device_id(_config._device_id);

	// Set the default values for the baro (which may not change)
	_sensor_baro.device_id = _config._device_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;
}

CvIns::~CvIns()
{
	if (serial_port_is_open(&device_port)) {
		serial_port_close(&device_port);
	}

#ifdef LOG_TRANSACTIONS
	_logger.thread_stop();
#endif
	PX4_INFO("Destructor");
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CvIns::init()
{
	// Run on fixed interval
	ScheduleOnInterval(1250_us); // 800 Hz

	return true;
}

void CvIns::setSensorRate(mip_descriptor_rate *sensor_descriptors, uint16_t len)
{
	// Get the base rate
	uint16_t sensor_base_rate;

	if (mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not get sensor base rate format!");
		return;
	}


	for (uint16_t i = 0; i < len; i++) {
		// Compute the desired decimation and update all of the sensors in this set
		const uint16_t sensor_decimation = sensor_base_rate / sensor_descriptors[i].decimation ;

		sensor_descriptors[i].decimation = sensor_decimation;
	}

	// Write the settings
	if (mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, len, sensor_descriptors) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set sensor message format!");
		return;
	}
}

void CvIns::loadRotation()
{
	// TODO: Load the param and setup the euler angles appropriately
}

int CvIns::connect_at_baud(int32_t baud){
	// Close
	if(serial_port_is_open(&device_port)){
		serial_port_close(&device_port);
	}
	PX4_INFO("Attempting to conect at %" PRIu32 " baud", baud);

	if (!serial_port_open(&device_port, "/dev/ttyS2", baud)) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}

	mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms);

	PX4_INFO("mip_base_ping");

	if (mip_base_ping(&device) != MIP_ACK_OK) {
		usleep(100_ms);
		if (mip_base_ping(&device) != MIP_ACK_OK) {
			PX4_INFO(" - Failed to Ping");
			return PX4_ERROR;
		}
	}
	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

#define BUAD_RATE 115200
void CvIns::initialize_cv7()
{
	if (_is_initialized) {
		return;
	}

	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 460800;

	if(connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR){
		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};
		for (auto &baudrate : BAUDRATES) {

			if(connect_at_baud(DEFAULT_BAUDRATE) == PX4_OK){
				PX4_INFO("found baudrate %" PRIu32, baudrate);
				break;
			}
		}
		PX4_WARN("Could not connect to the device, exiting");
		return;
	}


	PX4_INFO("mip_base_set_idle");

	if (mip_base_set_idle(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set the device to idle!");
		return;
	}

	PX4_INFO("Setting to default device settings");

	//Load the device default settings (so the device is in a known state)
	if (mip_3dm_default_device_settings(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not load default device settings!");
		return;
	}

	PX4_INFO("Connecting at default baudrate");

	if(connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR){
		exit_gracefully("ERROR: Could not reconnect at expected baud!");
		return;
	}

	PX4_INFO("Setting the baud to desired baud rate");

	usleep(500_ms);

	if(mip_3dm_write_uart_baudrate(&device, DESIRED_BAUDRATE) != MIP_ACK_OK){
		exit_gracefully("ERROR: Could not set the baudrate!");
		return;
	}

	tcflush(device_port.handle, TCIOFLUSH);

	usleep(500_ms);

	for(int i=0;i<10;i++){
		PX4_INFO("Connection Attempt:");
		if(connect_at_baud(DESIRED_BAUDRATE) == PX4_OK){
			break;
		}
		if(i >= 9){
			exit_gracefully("ERROR: Could not reconnect at desired baud!");
			return;
		}
	}



	// //Load the device default settings (so the device is in a known state)
	// if (mip_3dm_default_device_settings(&device) != MIP_ACK_OK) {
	// 	exit_gracefully("ERROR: Could not load default device settings!");
	// 	return;
	// }

	switch (_config._selected_mode) {
	case mode_imu: {
			// Scaled Gyro and Accel at a high rate
			mip_descriptor_rate imu_sensors[4] = {
				{ MIP_DATA_DESC_SENSOR_ACCEL_SCALED, _config._sens_imu_update_rate_hz },
				{ MIP_DATA_DESC_SENSOR_GYRO_SCALED,  _config._sens_imu_update_rate_hz },
				{ MIP_DATA_DESC_SENSOR_MAG_SCALED,   _config._sens_other_update_rate_hz },
				{ MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, _config._sens_other_update_rate_hz}
			};
			setSensorRate(imu_sensors, 4);
		}
		break;

	case mode_ahrs: {
			//
			//Setup FILTER data format
			//

			uint16_t filter_base_rate;

			if (mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate) != MIP_ACK_OK) {
				exit_gracefully("ERROR: Could not get filter base rate format!");
				return;
			}

			const uint16_t filter_sample_rate = 10; // Hz
			const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

			const mip_descriptor_rate filter_descriptors[3] = {
				{ MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
				{ MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
				{ MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
			};

			if (mip_3dm_write_message_format(&device, MIP_FILTER_DATA_DESC_SET, 3, filter_descriptors) != MIP_ACK_OK) {
				exit_gracefully("ERROR: Could not set filter message format!");
				return;
			}
		}
		break;

	case mode_ins:
		break;

	default:
		break;
	}

	loadRotation();

	if (mip_3dm_write_sensor_2_vehicle_transform_euler(&device, _config.sensor_to_vehicle_transformation_euler[0],
			_config.sensor_to_vehicle_transformation_euler[1], _config.sensor_to_vehicle_transformation_euler[2]) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation!");
		return;
	}


	//
	//Setup the filter aiding measurements (GNSS position/velocity)
	//

#if 0

	if (mip_filter_write_aiding_measurement_enable(&device,
			MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set filter aiding measurement enable!");
		return;
	}

#endif


	//
	//Reset the filter (note: this is good to do after filter setup is complete)
	//

	if (mip_filter_reset(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not reset the filter!");
		return;
	}


	//
	// Register data callbacks
	//
#if 0
	//Sensor Data
	mip_interface_register_extractor(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET,
					 MIP_DATA_DESC_SHARED_REFERENCE_TIME,     extract_mip_shared_reference_timestamp_data_from_field,
					 &sensor_reference_time);
	mip_interface_register_extractor(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET,
					 MIP_DATA_DESC_SHARED_GPS_TIME,     extract_mip_shared_gps_timestamp_data_from_field, &sensor_gps_time);
	mip_interface_register_extractor(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET,
					 MIP_DATA_DESC_SENSOR_ACCEL_SCALED, extract_mip_sensor_scaled_accel_data_from_field,  &sensor_accel);
	mip_interface_register_extractor(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET,
					 MIP_DATA_DESC_SENSOR_GYRO_SCALED,  extract_mip_sensor_scaled_gyro_data_from_field,   &sensor_gyro);
	mip_interface_register_extractor(&device, &sensor_data_handlers[4], MIP_SENSOR_DATA_DESC_SET,
					 MIP_DATA_DESC_SENSOR_MAG_SCALED,   extract_mip_sensor_scaled_mag_data_from_field,    &sensor_mag);
#else
	// Register some callbacks.
	mip_interface_register_field_callback(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET,
					      MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &handleAccel, this);
	mip_interface_register_field_callback(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET,
					      MIP_DATA_DESC_SENSOR_GYRO_SCALED, &handleGyro, this);
	mip_interface_register_field_callback(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET,
					      MIP_DATA_DESC_SENSOR_MAG_SCALED, &handleMag, this);
	mip_interface_register_field_callback(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET,
					      MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, &handleBaro, this);
#endif
	//Filter Data
	mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET,
					 MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, extract_mip_filter_timestamp_data_from_field, &filter_time);
	mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET,
					 MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,        &filter_status);
	mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET,
					 MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);

	mip_interface_register_field_callback(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET,
					      MIP_DATA_DESC_SHARED_EVENT_SOURCE, handle_filter_event_source,  NULL);


	if (mip_3dm_write_datastream_control(&device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not enable the data stream");
		return;
	}

	//
	//Resume the device
	//

	if (mip_base_resume(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not resume the device!");
		return;
	}

	_is_initialized = true;

}
#include <byteswap.h>


void CvIns::service_cv7()
{

	if (_is_initialized) {
		if ((_time_last_valid_imu_us != 0) && (hrt_elapsed_time(&_time_last_valid_imu_us) < 3_s)) {
			// update sensor_selection if configured in INS mode
			if ((_px4_accel.get_device_id() != 0) && (_px4_gyro.get_device_id() != 0)) {
				sensor_selection_s sensor_selection{};
				sensor_selection.accel_device_id = _px4_accel.get_device_id();
				sensor_selection.gyro_device_id = _px4_gyro.get_device_id();
				sensor_selection.timestamp = hrt_absolute_time();
				_sensor_selection_pub.publish(sensor_selection);
			}
		}
	}

	mip_interface_update(&device, false);

	// write_logs();

	switch (_state) {
	case 0:
		if (hrt_elapsed_time(&_last_print) > 10_s) {
			_last_print = hrt_absolute_time();
			PX4_INFO("Waiting for Filter to enter AHRS mode");
			PX4_INFO_RAW("Accel: %f %f %f\n", (double)sensor_accel.scaled_accel[0], (double)sensor_accel.scaled_accel[1],
				     (double)sensor_accel.scaled_accel[2]);

			float x = bswap_32(sensor_accel.scaled_accel[0]);
			float y = bswap_32(sensor_accel.scaled_accel[1]);
			float z = bswap_32(sensor_accel.scaled_accel[2]);

			PX4_INFO_RAW("Accel: %f %f %f\n", (double)x, (double)y,	(double)z);

		}

		if (filter_status.filter_state == MIP_FILTER_MODE_AHRS) {
			PX4_INFO("Entered AHRS mode, switching state");
			_state = 1;
		}

		break;

	case 1:
		if (hrt_elapsed_time(&_last_print) > 250_ms) {
			_last_print = hrt_absolute_time();
			PX4_INFO_RAW("Timestamp %llu Sensor Time %llu\n", _last_print, sensor_reference_time.nanoseconds);
			PX4_INFO_RAW("Accel: %f %f %f\n", (double)sensor_accel.scaled_accel[0], (double)sensor_accel.scaled_accel[1],
				     (double)sensor_accel.scaled_accel[2]);
			PX4_INFO_RAW("Gyro: %f %f %f\n", (double)sensor_gyro.scaled_gyro[0], (double)sensor_gyro.scaled_gyro[1],
				     (double)sensor_gyro.scaled_gyro[2]);
			PX4_INFO_RAW("Mag: %f %f %f\n", (double)sensor_mag.scaled_mag[0], (double)sensor_mag.scaled_mag[1],
				     (double)sensor_mag.scaled_mag[2]);
			PX4_INFO_RAW("R: %f P: %f Y: %f\n", (double)filter_euler_angles.roll, (double)filter_euler_angles.pitch,
				     (double)filter_euler_angles.yaw);
		}

		break;

	default:
		break;
	}
}

void CvIns::initialize_logger()
{
	if (_logger.is_initialized()) {
		return;
	}

	PX4_INFO("Creating the Logger");
	_logger.set_file_name("sess001.ulg");
	_logger.thread_start();
	PX4_INFO("Created the Logger");
}

void CvIns::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

#ifdef LOG_TRANSACTIONS
	initialize_logger();
#endif

	initialize_cv7();

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	service_cv7();

	perf_end(_loop_perf);
}

int CvIns::task_spawn(int argc, char *argv[])
{
	CvIns *instance = new CvIns();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		// Get a local reference
		cv7_ins = instance;

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

int CvIns::print_status()
{
	PX4_INFO_RAW("Serial Port Open %d Handle %d\n", device_port.is_open, device_port.handle);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int CvIns::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CvIns::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cv7_ins", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cv7_ins_main(int argc, char *argv[])
{
	return CvIns::main(argc, argv);
}
