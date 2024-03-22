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

#ifdef USING_MODALIO_UART
ModalIoSerial device_uart;
#else
serial_port device_port;
#endif

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;

#ifndef USING_MODALIO_UART
int set_uart_baud(int speed)
{
	struct termios		_cfg;
	/* Fill the struct for the new configuration */
	tcgetattr(device_port.handle, &_cfg);

	/* Disable output post-processing */
	_cfg.c_oflag &= ~OPOST;

	_cfg.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	_cfg.c_cflag &= ~CSIZE;
	_cfg.c_cflag |= CS8;                 /* 8-bit characters */
	_cfg.c_cflag &= ~PARENB;             /* no parity bit */
	_cfg.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
	_cfg.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	_cfg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	_cfg.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	if (cfsetispeed(&_cfg, speed) < 0 || cfsetospeed(&_cfg, speed) < 0) {
		PX4_ERR("Error configuring port");
		return PX4_ERROR;
	}

	if (tcsetattr(device_port.handle, TCSANOW, &_cfg) < 0) {
		PX4_ERR("Error configuring port");
		return PX4_ERROR;
	}

	return PX4_OK;

}
#endif

void CvIns::handleAccel(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_accel_data data;

	if (extract_mip_sensor_scaled_accel_data_from_field(field, &data)) {
		// Update the data structure
		ref->_accel.sample = data;
		ref->_accel.updated = true;

		// PX4_DEBUG("Accel Data: %f %f %f", (double)data.scaled_accel[0], (double)data.scaled_accel[1],
		// 	  (double)data.scaled_accel[2]);

		// auto t = hrt_absolute_time(); // timestamp;
		// ref->_px4_accel.update(t, data.scaled_accel[0]*CONSTANTS_ONE_G, data.scaled_accel[1]*CONSTANTS_ONE_G,
		// 		       data.scaled_accel[2]*CONSTANTS_ONE_G);
		// ref->_last_imu_time = t;
	}
}

void CvIns::handleGyro(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_gyro_data data;

	if (extract_mip_sensor_scaled_gyro_data_from_field(field, &data)) {
		// Update the data structure
		ref->_gyro.sample = data;
		ref->_gyro.updated = true;

		// PX4_DEBUG("Gyro Data:  %f, %f, %f", (double)data.scaled_gyro[0], (double)data.scaled_gyro[1],
		// 	  (double)data.scaled_gyro[2]);
		// auto t = hrt_absolute_time(); // timestamp;
		// ref->_px4_gyro.update(t, data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
		// ref->_last_imu_time = t;
	}
}

void CvIns::handleMag(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_mag_data data;

	if (extract_mip_sensor_scaled_mag_data_from_field(field, &data)) {
		// Update the data structure
		ref->_mag.sample = data;
		ref->_mag.updated = true;

		// PX4_DEBUG("Mag Data:   %f, %f, %f", (double)data.scaled_mag[0], (double)data.scaled_mag[1], (double)data.scaled_mag[2]);
		// auto t =  timestamp;; //hrt_absolute_time()
		// ref->_px4_mag.update(t, data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
	}
}

void CvIns::handleBaro(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_pressure_data data;

	// PX4_INFO("[BARO] Now %" PRIu64 " Then %" PRIu64 " Elapsed %" PRIu64, hrt_absolute_time(), timestamp, hrt_elapsed_time(&timestamp));

	if (extract_mip_sensor_scaled_pressure_data_from_field(field, &data)) {
		// Update the data structure
		ref->_baro.sample = data;
		ref->_baro.updated = true;

		// ref->_sensor_baro.timestamp = hrt_absolute_time();
		// ref->_sensor_baro.timestamp_sample = hrt_absolute_time();
		// ref->_sensor_baro.pressure = data.scaled_pressure * 100.f; // convert [Pa] to [mBar]
		// ref->_sensor_baro_pub.publish(ref->_sensor_baro);
	}
}

hrt_abstime CvIns::get_sample_timestamp(timestamp_type decode_timestamp, mip_shared_reference_timestamp_data ref_time){
	hrt_abstime t{0};
	// TODO: Handle the offset between system time and this timestamp
	t = ref_time.nanoseconds / 1000;

	// Compute the offset first time through
	if(_cv7_offset_time == 0){
		PX4_INFO("Device Time Setup");
		PX4_INFO("Now %" PRIu64, decode_timestamp);
		PX4_INFO("Device Time %" PRIu64, t);
		_cv7_offset_time = decode_timestamp - t - 1900_us;
		PX4_INFO("Computed Offset %" PRId64, _cv7_offset_time);
	}

	// Adjsut the time to px4 relative time
	t += _cv7_offset_time;

	return t;
}

void CvIns::handleTimestamp(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_shared_reference_timestamp_data data;

	if (extract_mip_shared_reference_timestamp_data_from_field(field, &data)) {

		// Convert to a useful time for PX4
		// auto t = ref->get_sample_timestamp(timestamp, data);		// Causes timestamps duplications (buffer too full?)
		// auto t = timestamp - 1900_us;				// Packets are then ~4ms old and timestamp duplications
		auto t = hrt_absolute_time() - 1900_us;				// Packets are then ~2ms old
		// auto t = hrt_absolute_time();				// Packets are old but system thinks they are new
		// auto t = timestamp;						// Packets at time of arrival and timestamp duplications

		// Send all of the data with the common timestamp

		if(ref->_accel.updated){
			ref->_px4_accel.update(t, ref->_accel.sample.scaled_accel[0]*CONSTANTS_ONE_G, ref->_accel.sample.scaled_accel[1]*CONSTANTS_ONE_G,
					       ref->_accel.sample.scaled_accel[2]*CONSTANTS_ONE_G);
			ref->_last_imu_time = t;
			ref->_accel.updated = false;
		}

		if(ref->_gyro.updated){
			ref->_px4_gyro.update(t, ref->_gyro.sample.scaled_gyro[0], ref->_gyro.sample.scaled_gyro[1], ref->_gyro.sample.scaled_gyro[2]);
			ref->_last_imu_time = t;
			ref->_gyro.updated = false;
		}

		if(ref->_mag.updated){
			ref->_px4_mag.update(t, ref->_mag.sample.scaled_mag[0], ref->_mag.sample.scaled_mag[1], ref->_mag.sample.scaled_mag[2]);
			ref->_mag.updated = false;
		}

		if(ref->_baro.updated){
			ref->_sensor_baro.timestamp = timestamp;
			ref->_sensor_baro.timestamp_sample = t;
			ref->_sensor_baro.pressure = ref->_baro.sample.scaled_pressure * 100.f; // convert [Pa] to [mBar]
			ref->_sensor_baro_pub.publish(ref->_sensor_baro);
			ref->_baro.updated = false;
		}
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

bool mip::C::mip_interface_user_recv_from_device(mip_interface *device, uint8_t *buffer, size_t max_length,
		size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = hrt_absolute_time();

#ifdef USING_MODALIO_UART
	int res = device_uart.uart_read(buffer,max_length);

	if(res == -1 && errno != EAGAIN){
		// PX4_INFO("RX 1 %d(%d)",res,max_length);
		*out_length = 0;
		return false;
	}
	if(res >= 0){
		*out_length = res;
		// PX4_INFO("RX 2 %d(%d)",*out_length,max_length);
	}
	// PX4_INFO("RX 3 %d(%d)",*out_length,max_length);
#else

	if (!serial_port_read(&device_port, buffer, max_length, out_length)) {
		return false;
	}
#endif
#ifdef LOG_TRANSACTIONS

	if (cv7_ins) {
		cv7_ins->get_logger().enqueue_rx(buffer, *out_length);
	}

#endif
	cv7_ins->_debug_rx_bytes[0] = math::min<uint32_t>(cv7_ins->_debug_rx_bytes[0],*out_length);
	cv7_ins->_debug_rx_bytes[1] += *out_length;
	cv7_ins->_debug_rx_bytes[2] = math::max<uint32_t>(cv7_ins->_debug_rx_bytes[2],*out_length);
	cv7_ins->_debug_rx_bytes[3]++;
	return true;
}

bool mip::C::mip_interface_user_send_to_device(mip_interface *device, const uint8_t *data, size_t length)
{


#ifdef LOG_TRANSACTIONS

	if (cv7_ins) {
		cv7_ins->get_logger().enqueue_tx(data, length);
	}

#endif
#ifdef USING_MODALIO_UART
	PX4_INFO("TX %d",length);
	int res = device_uart.uart_write(const_cast<uint8_t*>(data),length);
	if(res >= 0){
		return true;
	}
	return false;
#else
	size_t bytes_written{0};
	return serial_port_write(&device_port, data, length, &bytes_written);
	#endif
}

CvIns::CvIns(const char *uart_port, int32_t rot) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	_uart_device = uart_port;
	_config.rot = static_cast<Rotation>(rot);
	// TODO: Figure out how to set to arbitrary rates, currently it limited based on decimation
	// // Clamp rate to allowable ranges
	// _config.sens_imu_update_rate_hz = math::constrain<uint16_t>(_param_imu_gyro_ratemax.get(),100,1000);

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_3DMCV7;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	_config.device_id = device_id.devid;
	// Default to ROTATION_NONE
	_px4_accel.set_device_id(_config.device_id);
	_px4_gyro.set_device_id(_config.device_id);
	_px4_mag.set_device_id(_config.device_id);

	// Set the default values for the baro (which may not change)
	_sensor_baro.device_id = _config.device_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;
}

CvIns::~CvIns()
{
	#ifdef USING_MODALIO_UART
	if (device_uart.is_open()) {
		device_uart.uart_close();
	}
	#else
	if (serial_port_is_open(&device_port)) {
		serial_port_close(&device_port);
	}
	#endif

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
	ScheduleOnInterval(2000_us);

	return true;
}

void CvIns::set_sensor_rate(mip_descriptor_rate *sensor_descriptors, uint16_t len)
{
	// Get the base rate
	uint16_t sensor_base_rate;

	if (mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not get sensor base rate format!");
		return;
	}

	PX4_INFO("The CV7 base rate is %d", sensor_base_rate);

	for (uint16_t i = 0; i < len; i++) {
		// Compute the desired decimation and update all of the sensors in this set
		float sensor_decimation = static_cast<float>(sensor_base_rate) / static_cast<float>(sensor_descriptors[i].decimation);

		sensor_descriptors[i].decimation = static_cast<uint16_t>(sensor_decimation);
	}

	// Write the settings
	mip_cmd_result res = mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, len, sensor_descriptors);

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor message format! Result of %d", res);
		return;
	}
}

int CvIns::connect_at_baud(int32_t baud)
{
	#ifdef USING_MODALIO_UART
	if (device_uart.is_open()) {
		if (device_uart.uart_set_baud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (device_uart.uart_open(_uart_device, baud) == PX4_ERROR ) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}
	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_uart.is_open() ? "CONNECTED" : "NOT CONNECTED"), baud);
	#else
	if (device_port.is_open) {
		if (set_uart_baud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (!serial_port_open(&device_port, _uart_device, baud)) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}
	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_port.is_open ? "CONNECTED" : "NOT CONNECTED"), baud);
	#endif




	// Re-init the interface with the correct timeouts
	mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms);

	PX4_INFO("mip_base_ping");

	if (mip_base_ping(&device) != MIP_ACK_OK) {
		PX4_INFO(" - Failed to Ping 1");
		usleep(200_ms);

		if (mip_base_ping(&device) != MIP_ACK_OK) {
			PX4_INFO(" - Failed to Ping 2");
			return PX4_ERROR;
		}
	}

	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

void CvIns::initialize_cv7()
{
	if (_is_initialized) {
		return;
	}

	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 921600;

	if (connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR) {

		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};
		bool is_connected = false;

		for (auto &baudrate : BAUDRATES) {
			if (connect_at_baud(baudrate) == PX4_OK) {
				PX4_INFO("found baudrate %" PRIu32, baudrate);
				is_connected = true;
				break;
			}
		}

		if (!is_connected) {
			_is_init_failed = true;
			PX4_WARN("Could not connect to the device, exiting");
			return;
		}
	}


	PX4_INFO("mip_base_set_idle");

	if (mip_base_set_idle(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the device to idle!");
		return;
	}

	PX4_INFO("Setting to default device settings");

	//Load the device default settings (so the device is in a known state)
	if (mip_3dm_default_device_settings(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not load default device settings!");
		return;
	}

	PX4_INFO("Connecting at default baudrate");

	if (connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR) {
		PX4_ERR("ERROR: Could not reconnect at expected baud!");
		return;
	}

	PX4_INFO("Setting the baud to desired baud rate");

	usleep(500_ms);

	if (mip_3dm_write_uart_baudrate(&device, DESIRED_BAUDRATE) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the baudrate!");
		_is_init_failed = true;
		return;
	}

	#ifdef USING_MODALIO_UART
	tcflush(device_uart.uart_get_fd(), TCIOFLUSH);
	#else
	tcflush(device_port.handle, TCIOFLUSH);
	#endif

	usleep(500_ms);

	for (int i = 0; i < 10; i++) {
		PX4_INFO("Connection Attempt: %d", i);

		if (connect_at_baud(DESIRED_BAUDRATE) == PX4_OK) {
			break;
		}

		if (i >= 9) {
			PX4_ERR("ERROR: Could not reconnect at desired baud!");
			_is_init_failed = true;
			return;
		}
	}

	switch (_config.selected_mode) {
	case mode_imu: {
			// Scaled Gyro and Accel at a high rate
			mip_descriptor_rate imu_sensors[5] = {
				{ MIP_DATA_DESC_SENSOR_ACCEL_SCALED, _config.sens_imu_update_rate_hz},
				{ MIP_DATA_DESC_SENSOR_GYRO_SCALED, _config.sens_imu_update_rate_hz},
				{ MIP_DATA_DESC_SENSOR_MAG_SCALED,  _config.sens_other_update_rate_hz},
				{ MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, _config.sens_other_update_rate_hz},
				{ MIP_DATA_DESC_SHARED_REFERENCE_TIME, _config.sens_imu_update_rate_hz},

			};

			set_sensor_rate(imu_sensors, 5);

			//
			// Register data callbacks
			//
			mip_interface_register_field_callback(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &handleAccel, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_GYRO_SCALED, &handleGyro, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_MAG_SCALED, &handleMag, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, &handleBaro, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[4], MIP_SHARED_DATA_DESC_SET,
							      MIP_DATA_DESC_SHARED_REFERENCE_TIME, &handleTimestamp, this);

			mip_interface_register_extractor(&device, &sensor_data_handlers[5], MIP_SHARED_DATA_DESC_SET,
							 MIP_DATA_DESC_SHARED_REFERENCE_TIME,     extract_mip_shared_reference_timestamp_data_from_field,
							 &sensor_reference_time);

			// uint16_t shared_data_base_rate{1000};

			// mip_cmd_result res = mip_3dm_get_base_rate(&device, MIP_SHARED_DATA_DESC_SET, &shared_data_base_rate);
			// if (res != MIP_ACK_OK) {
			// 	PX4_ERR("ERROR: Could not get filter base rate format! Result of %d -- continuing", res);
			// }
			// 	// { MIP_DATA_DESC_SENSOR_TIME_STAMP_INTERNAL, 50 },
			// const uint16_t shared_sample_rate = 10; // Hz
			// const uint16_t filter_decimation = shared_data_base_rate / shared_sample_rate;

			// const mip_descriptor_rate shared_sample_descriptors[1] = {
			// 	{ MIP_DATA_DESC_SHARED_REFERENCE_TIME,         filter_decimation }
			// };

			// mip_cmd_result res = mip_3dm_write_message_format(&device, MIP_SHARED_DATA_DESC_SET, 1, shared_sample_descriptors);
			// if ( res!= MIP_ACK_OK) {
			// 	PX4_ERR("ERROR: Could not set shared message format! Result of %d", res);
			// 	return;
			// }

		}
		break;

	case mode_ahrs: {
			//
			//Setup FILTER data format
			//

			uint16_t filter_base_rate;

			if (mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate) != MIP_ACK_OK) {
				PX4_ERR("ERROR: Could not get filter base rate format!");
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
				PX4_ERR("ERROR: Could not set filter message format!");
				return;
			}


			// Filter Data
			mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET,
							 MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, extract_mip_filter_timestamp_data_from_field, &filter_time);
			mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET,
							 MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,        &filter_status);
			mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET,
							 MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);
			mip_interface_register_field_callback(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET,
							      MIP_DATA_DESC_SHARED_EVENT_SOURCE, handle_filter_event_source,  NULL);

			//
			//Reset the filter (note: this is good to do after filter setup is complete)
			//

			if (mip_filter_reset(&device) != MIP_ACK_OK) {
				PX4_ERR("ERROR: Could not reset the filter!");
				return;
			}
		}
		break;

	case mode_ins: {
			//
			//Setup the filter aiding measurements (GNSS position/velocity)
			//

			if (mip_filter_write_aiding_measurement_enable(&device,
					MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK) {
				PX4_ERR("ERROR: Could not set filter aiding measurement enable!");
				return;
			}

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
		}
		break;

	default:
		break;
	}

	//
	// Setup the rotation based on PX4 standard rotation sets
	//

	if (mip_3dm_write_sensor_2_vehicle_transform_euler(&device, math::radians<float>(rot_lookup[_config.rot].roll),
			math::radians<float>(rot_lookup[_config.rot].pitch),
			math::radians<float>(rot_lookup[_config.rot].yaw)) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor-to-vehicle transformation!");
		return;
	}


	if (mip_3dm_write_datastream_control(&device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not enable the data stream");
		return;
	}

	//
	//Resume the device
	//

	if (mip_base_resume(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not resume the device!");
		return;
	}

	_is_initialized = true;

}

void CvIns::service_cv7()
{
	mip_interface_update(&device, false);

	switch (_config.selected_mode) {
	case mode_ins:
		// Feed any aiding information into the driver here
		break;

	case mode_ahrs:
	case mode_imu:
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

	// Initialization failed, stop the module
	if (_is_init_failed) {
		request_stop();
		perf_end(_loop_perf);
		return;
	}

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
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *dev = "/dev/ttyS2";
	int32_t rot = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "d:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = myoptarg;
			break;

		case 'r':
			rot = atoi(myoptarg);

			if (rot >= ROTATION_MAX) {
				rot = ROTATION_NONE;
			}

			break;
		}
	}

	if (dev == nullptr || strlen(dev) == 0) {
		print_usage("no device specified");
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}

	CvIns *instance = new CvIns(dev, rot);

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
	#ifdef USING_MODALIO_UART
	PX4_INFO_RAW("Serial Port Open %d Handle %d Device %s\n", device_uart.is_open(), device_uart.uart_get_fd(), _uart_device);
	#else
	PX4_INFO_RAW("Serial Port Open %d Handle %d Device %s\n", device_port.is_open, device_port.handle, _uart_device);
	#endif
	PX4_INFO_RAW("Min %lu\n",_debug_rx_bytes[0]);
	PX4_INFO_RAW("Total %lu\n",_debug_rx_bytes[1]);
	PX4_INFO_RAW("Max %lu\n",_debug_rx_bytes[2]);
	PX4_INFO_RAW("Avg %f\n",static_cast<double>(_debug_rx_bytes[1]*1.f / _debug_rx_bytes[3]*1.f));
	_debug_rx_bytes[0] = UINT32_MAX;
	for (int i = 1; i < 4; i++)
	{
		_debug_rx_bytes[i] = 0;
	}

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
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS2", "<file:dev>", "CV7 Port", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, ROTATION_MAX, "See enum Rotation for values", true);

	return 0;
}

extern "C" __EXPORT int cv7_ins_main(int argc, char *argv[])
{
	return CvIns::main(argc, argv);
}
