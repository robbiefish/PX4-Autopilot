#include <px4_platform_common/px4_config.h>
#include <termios.h>

#include "CircularBuffer.hpp"
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>


#define BUFFER_SIZE 256
#define MAX_WRITE_CHUNK 128

using namespace time_literals;

class LogWriter
{
private:
	pthread_attr_t loop_attr;
	pthread_t _thread{0};      ///< worker task id
	px4::atomic_bool _thread_should_exit{false};
	RingBufCPP<uint8_t, BUFFER_SIZE> _tx_buf;
	RingBufCPP<uint8_t, BUFFER_SIZE> _rx_buf;
	px4_sem_t _sem_data;
	pthread_mutex_t	_mutex = PTHREAD_MUTEX_INITIALIZER;
	char _file_name[24];

	static void *trampoline(void *context);
	int _tx_writer{-1};
	int _rx_writer{-1};

	const char *directory_path = PX4_STORAGEDIR "/log/sess999";

	int make_directory_structure()
	{
		int res = ::mkdir(directory_path, S_IRWXU | S_IRWXG | S_IRWXO);
		int error_number = get_errno();

		// Couldn't create the folder because it already exists
		if (res == -1 && error_number == EEXIST) {
			return PX4_OK;
		}

		// Other errors
		if (res == -1) {
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	void create_file_name(char *name)
	{
		strcpy(name, "sess001.ulg");
		return;
	}

	int open_writers()
	{

		char full_path[124] {0};
		sprintf(full_path, "%s/tx_%s", directory_path, _file_name);

		_tx_writer = ::open(full_path, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);

		if (_tx_writer < 0) {
			printf("Couldn't open TX FD %d\n", get_errno());
			::close(_tx_writer);
			return PX4_ERROR;
		}

		// Clear the buffer and build the RX path
		memset(&full_path[0], 0x00, 124);
		sprintf(full_path, "%s/rx_%s", directory_path, _file_name);
		_rx_writer = ::open(full_path, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);

		if (_rx_writer < 0) {
			printf("Couldn't open RX FD %d\n", get_errno());
			::close(_rx_writer);
			::close(_tx_writer);
			return PX4_ERROR;
		}

		::fsync(_tx_writer);
		::fsync(_rx_writer);

		return PX4_OK;
	}

	/// @brief Pend on the semaphore (but wake every second to check if thread should exit)
	void wait_for_data()
	{
		// If there is data available to write, don't pend on new data
		if (_tx_buf.numElements() > 0 || _rx_buf.numElements() > 0) {
			return;
		}

		// timed wait doesn't appear to work, use polling
		// and a defined sleep instead
		int ret = px4_sem_trywait(&_sem_data);

		if (ret != 0) {
			usleep(1_ms);
		}
	}

	void thread_run()
	{
		printf("Entering Log Writer Thread\n");

		if (make_directory_structure() != PX4_OK) {
			printf("Couldn't create the folder structure, exiting with %d\n", get_errno());
			return;
		}

		printf("Created the directory\n");

		if (open_writers() != PX4_OK) {
			printf("Couldn't open a write descriptor, exiting with %d", get_errno());
			return;
		}

		do {
			wait_for_data();
#if 1
			uint8_t buffer[MAX_WRITE_CHUNK];
			uint32_t len = 0;

			LockGuard lg{_mutex};

			if (_tx_buf.numElements() > 0) {
				len = 0;
				int s = _tx_buf.numElements() < MAX_WRITE_CHUNK ? _tx_buf.numElements() : MAX_WRITE_CHUNK;

				for (int i = 0; i < s; i++) {
					_tx_buf.pull(buffer[i]);
					len++;
				}

				::write(_tx_writer, buffer, len);
			}

			if (_rx_buf.numElements() > 0) {
				len = 0;
				int s = _rx_buf.numElements() < MAX_WRITE_CHUNK ? _rx_buf.numElements() : MAX_WRITE_CHUNK;

				for (int i = 0; i < s; i++) {
					_rx_buf.pull(buffer[i]);
					len++;
				}

				::write(_rx_writer, buffer, len);
			}

#endif
		} while (!_thread_should_exit.load());

		::fsync(_tx_writer);
		::fsync(_rx_writer);
		::close(_tx_writer);
		::close(_rx_writer);
		printf("Exiting Log Writer Thread\n");
	}

public:
	LogWriter();
	~LogWriter();
	void set_file_name(const char *fn)
	{
		strcpy(_file_name, fn);
	}
	void thread_stop()
	{
		_thread_should_exit.store(true);
		pthread_join(_thread, nullptr);
	}
	void thread_start();
	void enqueue_tx(const uint8_t *buffer, size_t len)
	{
		LockGuard lg{_mutex};

		// Copy data into a log to write to the SD Card
		for (size_t i = 0; i < len; i++) {
			_tx_buf.add(buffer[i], true);
		}

		px4_sem_post(&_sem_data);
	}
	void enqueue_rx(uint8_t *buffer, size_t len)
	{
		LockGuard lg{_mutex};

		// Copy data into a log to write to the SD Card
		for (size_t i = 0; i < len; i++) {
			_rx_buf.add(buffer[i], true);
		}

		px4_sem_post(&_sem_data);
	}
};

LogWriter::LogWriter()
{
	px4_sem_init(&_sem_data, 0, 0);
	/* _sem_data use case is a signal */
	px4_sem_setprotocol(&_sem_data, SEM_PRIO_NONE);
}

LogWriter::~LogWriter()
{
	pthread_mutex_destroy(&_mutex);
	px4_sem_destroy(&_sem_data);
}

void LogWriter::thread_start()
{

	pthread_attr_init(&loop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&loop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_LOG_WRITER - 1;
	(void)pthread_attr_setschedparam(&loop_attr, &param);

	pthread_attr_setstacksize(&loop_attr, 1200);
	pthread_create(&_thread, &loop_attr, LogWriter::trampoline, (void *)this);
	pthread_attr_destroy(&loop_attr);
}


void *LogWriter::trampoline(void *context)
{
	LogWriter *self = reinterpret_cast<LogWriter *>(context);
	self->thread_run();
	return nullptr;
}
