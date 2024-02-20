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

#include "cv7_ins.hpp"
#include "mip_sdk/src/mip/mip_parser.h"
#include "CircularBuffer.hpp"

bool is_logging = true;
#define LOG_SIZE 256
uint8_t local_buf[LOG_SIZE];
uint32_t count = 0;

#define LOG_TRANSACTIONS

#ifdef LOG_TRANSACTIONS
RingBufCPP<uint8_t, LOG_SIZE> tx_buf;
RingBufCPP<uint8_t, LOG_SIZE> rx_buf;
int tx_writer = -1;
int rx_writer = -1;
#endif

serial_port device_port;
// serial_port *device_port_ptr;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;


void handleAccel(void *user, const mip_field *field, timestamp_type timestamp)
{
	(void)user;
	mip_sensor_scaled_accel_data data;

	if (extract_mip_sensor_scaled_accel_data_from_field(field, &data)) {
		PX4_INFO("Accel Data: %f %f %f", (double)data.scaled_accel[0], (double)data.scaled_accel[1],
			 (double)data.scaled_accel[2]);
		// TODO: Publish data here
	}
}

void handleGyro(void *user, const mip_field *field, timestamp_type timestamp)
{
	(void)user;
	mip_sensor_scaled_gyro_data data;

	if (extract_mip_sensor_scaled_gyro_data_from_field(field, &data)) {
		PX4_INFO("Gyro Data:  %f, %f, %f", (double)data.scaled_gyro[0], (double)data.scaled_gyro[1],
			 (double)data.scaled_gyro[2]);
	}

	// TODO: Publish data here
}

void handleMag(void *user, const mip_field *field, timestamp_type timestamp)
{
	(void)user;
	mip_sensor_scaled_mag_data data;

	if (extract_mip_sensor_scaled_mag_data_from_field(field, &data)) {
		PX4_INFO("Mag Data:   %f, %f, %f", (double)data.scaled_mag[0], (double)data.scaled_mag[1], (double)data.scaled_mag[2]);
	}

	// TODO: Publish data here
}

////////////////////////////////////////////////////////////////////////////////
// Filter Event Source Field Handler
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
// MIP Interface Time Access Function
////////////////////////////////////////////////////////////////////////////////

timestamp_type get_current_timestamp()
{
	return hrt_absolute_time();
}

////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Recv Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip::C::mip_interface_user_recv_from_device(mip_interface *device, uint8_t *buffer, size_t max_length,
		size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = get_current_timestamp();

	if (!serial_port_read(&device_port, buffer, max_length, out_length)) {
		return false;
	}

#ifdef LOG_TRANSACTIONS

	for (size_t i = 0; i < *out_length; i++) {
		rx_buf.add(buffer[i], true);
	}

#endif

	return true;
}

////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Send Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip::C::mip_interface_user_send_to_device(mip_interface *device, const uint8_t *data, size_t length)
{
	size_t bytes_written;

#ifdef LOG_TRANSACTIONS

	// Copy data into a log to write to the SD Card
	for (size_t i = 0; i < length; i++) {
		tx_buf.add(data[i], true);
	}

#endif

	return serial_port_write(&device_port, data, length, &bytes_written);
}

void write_logs()
{
#ifndef LOG_TRANSACTIONS
	return;
#else
#define CHUNK 64
	static bool init = false;
	static int missed_write_counter = 0;

	if (!init) {
		tx_writer = ::open(PX4_STORAGEDIR "/log/sess100/log001.ulg", O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);

		if (tx_writer < 0) {
			PX4_WARN("Couldn't open FD %d", get_errno());
		}

		rx_writer = ::open(PX4_STORAGEDIR "/log/sess100/log002.ulg", O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);
		::fsync(tx_writer);
		::fsync(rx_writer);
		init = true;
	}

	if (tx_writer && ((tx_buf.numElements() > CHUNK) || missed_write_counter > 100)) {
		uint8_t buffer[CHUNK];
		uint32_t len = 0;
		int s = tx_buf.numElements() < CHUNK ? tx_buf.numElements() : CHUNK;

		for (int i = 0; i < s; i++) {
			tx_buf.pull(buffer[i]);
			len++;
		}

		::write(tx_writer, buffer, len);
		::fsync(tx_writer);
		missed_write_counter = 0;

	} else {
		missed_write_counter++;
	}

	if (rx_writer && rx_buf.numElements() > CHUNK) {
		uint8_t buffer[CHUNK];
		uint32_t len = 0;

		for (int i = 0; i < CHUNK; i++) {
			rx_buf.pull(buffer[i]);
			len++;
		}

		::write(rx_writer, buffer, len);
		::fsync(rx_writer);
	}

#endif
}

void exit_gracefully(const char *msg)
{
	PX4_ERR("%s", msg);
}

CvIns::CvIns() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{

}

CvIns::~CvIns()
{
	if (serial_port_is_open(&device_port)) {
		serial_port_close(&device_port);
	}

	::close(tx_writer);
	::close(rx_writer);

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CvIns::init()
{
	// Run on fixed interval
	ScheduleOnInterval(5_ms);

	return true;
}

void CvIns::initialize_cv7()
{
	if (_is_initialized) {
		return;
	}

	if (!serial_port_open(&device_port, "/dev/ttyS2", 115200)) {
		PX4_ERR("ERROR: Could not open device port!");
		return;
	}

	//
	//Initialize the MIP interface
	//

	mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(115200) * 1_ms, 250_ms);

	//
	//Ping the device (note: this is good to do to make sure the device is present)
	//
	PX4_INFO("mip_base_ping");

	if (mip_base_ping(&device) != MIP_ACK_OK) {
		exit_gracefully("Couldn't connect to device");
	}

	PX4_INFO("MIP_Size %d", device._max_update_pkts);

	_is_initialized = true;

	//
	//Idle the device (note: this is good to do during setup)
	//
	PX4_INFO("mip_base_set_idle");

	if (mip_base_set_idle(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set the device to idle!");
	}

#if 0

	for (size_t i = 0; i < count; i++) {
		PX4_INFO_RAW("%02X", local_buf[i]);
	}

	PX4_INFO("");

	return;
#endif


	//
	//Load the device default settings (so the device is in a known state)
	//

	if (mip_3dm_default_device_settings(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not load default device settings!");
	}

	//
	//Setup Sensor data format to XXX Hz
	//

	uint16_t sensor_base_rate;

	//Note: Querying the device base rate is only one way to calculate the descriptor decimation.
	//We could have also set it directly with information from the datasheet (shown in GNSS setup).

	if (mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not get sensor base rate format!");
	}

	const uint16_t sensor_sample_rate = 10; // Hz
	const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

	const mip_descriptor_rate sensor_descriptors[4] = {
		{ MIP_DATA_DESC_SHARED_GPS_TIME,     sensor_decimation },
		{ MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation },
		{ MIP_DATA_DESC_SENSOR_GYRO_SCALED,  sensor_decimation },
		{ MIP_DATA_DESC_SENSOR_MAG_SCALED,   sensor_decimation },
	};

	if (mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 4, sensor_descriptors) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set sensor message format!");
	}


	//
	//Setup FILTER data format
	//

	uint16_t filter_base_rate;

	if (mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not get filter base rate format!");
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
	}


	//
	// Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
	// (Note 1: we are reusing the event and action structs, since the settings for pitch/roll are so similar)
	// (Note 2: we are using the same value for event and action ids.  This is not necessary, but done here for convenience)
	//

	//EVENTS

	//Roll
	union mip_3dm_event_trigger_command_parameters event_params;
	event_params.threshold.type       = MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW;
	event_params.threshold.desc_set   = MIP_FILTER_DATA_DESC_SET;
	event_params.threshold.field_desc = MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES;
	event_params.threshold.param_id   = 1;
	event_params.threshold.high_thres = -0.7853981;
	event_params.threshold.low_thres  = 0.7853981;

	if (mip_3dm_write_event_trigger(&device, FILTER_ROLL_EVENT_ACTION_ID, MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD,
					&event_params) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set pitch event parameters!");
	}

	//Pitch
	event_params.threshold.param_id = 2;

	if (mip_3dm_write_event_trigger(&device, FILTER_PITCH_EVENT_ACTION_ID, MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD,
					&event_params) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set roll event parameters!");
	}

	//ACTIONS

	//Roll
	union mip_3dm_event_action_command_parameters event_action;
	event_action.message.desc_set       = MIP_FILTER_DATA_DESC_SET;
	event_action.message.num_fields     = 1;
	event_action.message.descriptors[0] = MIP_DATA_DESC_SHARED_EVENT_SOURCE;
	event_action.message.decimation     = 0;

	if (mip_3dm_write_event_action(&device, FILTER_ROLL_EVENT_ACTION_ID, FILTER_ROLL_EVENT_ACTION_ID,
				       MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, &event_action) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set roll action parameters!");
	}

	//Pitch
	if (mip_3dm_write_event_action(&device, FILTER_PITCH_EVENT_ACTION_ID, FILTER_PITCH_EVENT_ACTION_ID,
				       MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, &event_action) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set pitch action parameters!");
	}

	//ENABLE EVENTS

	//Roll
	if (mip_3dm_write_event_control(&device, FILTER_ROLL_EVENT_ACTION_ID,
					MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not enable roll event!");
	}

	//Pitch
	if (mip_3dm_write_event_control(&device, FILTER_PITCH_EVENT_ACTION_ID,
					MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not enable pitch event!");
	}


	//
	//Setup the sensor to vehicle transformation
	//

	if (mip_3dm_write_sensor_2_vehicle_transform_euler(&device, sensor_to_vehicle_transformation_euler[0],
			sensor_to_vehicle_transformation_euler[1], sensor_to_vehicle_transformation_euler[2]) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation!");
	}


	//
	//Setup the filter aiding measurements (GNSS position/velocity)
	//

	if (mip_filter_write_aiding_measurement_enable(&device,
			MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not set filter aiding measurement enable!");
	}


	//
	//Reset the filter (note: this is good to do after filter setup is complete)
	//

	if (mip_filter_reset(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not reset the filter!");
	}


	//
	// Register data callbacks
	//

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

	// Register some callbacks.
	// mip_interface_register_field_callback(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &handleAccel, NULL);
	// mip_interface_register_field_callback(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED , &handleGyro , NULL);
	// mip_interface_register_field_callback(&device, &sensor_data_handlers[4], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED  , &handleMag  , NULL);

	//Filter Data
	mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET,
					 MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, extract_mip_filter_timestamp_data_from_field, &filter_time);
	mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET,
					 MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,        &filter_status);
	mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET,
					 MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);

	mip_interface_register_field_callback(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET,
					      MIP_DATA_DESC_SHARED_EVENT_SOURCE, handle_filter_event_source,  NULL);

	//
	//Resume the device
	//

	if (mip_base_resume(&device) != MIP_ACK_OK) {
		exit_gracefully("ERROR: Could not resume the device!");
	}

}
#include <byteswap.h>


void CvIns::service_cv7()
{

	mip_interface_update(&device, false);

	write_logs();

	switch (_state) {
	case 0:
		if (hrt_elapsed_time(&_last_print) > 250_ms) {
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

void CvIns::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

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
