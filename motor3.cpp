/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file motor3.cpp
 *
 * @author ecmnet <ecm@gmx.de>
 * @author Vasily Evseenko <svpcom@gmail.com>
 *
 * Driver for the Lightware SF1xx lidar range finder series.
 * Default I2C address 0x66 is used.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/rpm3_status.h>
#include <board_config.h>

/* Configuration Constants */
#define MOTOR3_BUS 		PX4_I2C_BUS_EXPANSION2
#define MOTOR3_BASEADDR 	0x66
#define MOTOR3_DEVICE_PATH	"/dev/motor3"
#define RPM_MEASURE_DEVICE_PATH "/dev/RPM0"


#ifndef CONFIG_SCHED_WORKQUEUE
#error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MOTOR3 : public device::I2C
{
public:
	MOTOR3(int bus = MOTOR3_BUS, int address = MOTOR3_BASEADDR);
	virtual ~MOTOR3();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	int                 _conversion_interval;
	work_s				_work;
	ringbuffer::RingBuffer  *_reports;
	bool				_sensor_ok;
	int				_measure_ticks;
	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t		_esc_rpm_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;



	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int motor3_main(int argc, char *argv[]);

MOTOR3::MOTOR3(int bus, int address) :
	I2C("MOTOR3", MOTOR3_DEVICE_PATH, bus, address, 400000),
	_conversion_interval(-1),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_esc_rpm_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "motor3_read")),
	_comms_errors(perf_alloc(PC_COUNT, "motor3_com_err"))

{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

MOTOR3::~MOTOR3()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}
	if(_esc_rpm_topic != nullptr){
		orb_unadvertise(_esc_rpm_topic);
	}
	if (_class_instance != -1) {
		unregister_class_devname(RPM_MEASURE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int MOTOR3::init()
{
	int ret = PX4_ERROR;
		
	_conversion_interval = 50000;


	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	// _reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));
	_reports = new ringbuffer::RingBuffer(2, sizeof(rpm3_status_s));

	set_device_address(MOTOR3_BASEADDR);

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RPM_MEASURE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */

	struct rpm3_status_s esc_rpm_report = {};
	_esc_rpm_topic = orb_advertise_multi(ORB_ID(rpm3_status), &esc_rpm_report,
				&_orb_class_instance, ORB_PRIO_HIGH);
	if (_esc_rpm_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}
	// Select altitude register
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
		DEVICE_LOG("(%dHz) with address %d found", (int)(1e6f / _conversion_interval), MOTOR3_BASEADDR);
	}

	return ret;
}

int MOTOR3::probe()
{
	return measure();
}

int MOTOR3::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			ATOMIC_ENTER;

			if (!_reports->resize(arg)) {
				ATOMIC_LEAVE;
				return -ENOMEM;
			}

			ATOMIC_LEAVE;

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t MOTOR3::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct rpm3_status_s);
	struct rpm3_status_s *rbuf = reinterpret_cast<struct rpm3_status_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int MOTOR3::measure()
{
	int ret;

	/*
	 * Send the command '0' -- read altitude
	 */

	uint8_t cmd = 0;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int MOTOR3::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};
	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint32_t rpm_i2c = val[0] << 8 | val[1];
	uint32_t rpm3, rpm = float(rpm_i2c);

	rpm3 = (64000000/12)/rpm;

	static int rpm_record;
	static int rpmcount = 0;

	if (rpm3 == rpm_record) {
		 rpmcount =  rpmcount + 1;
		 rpm_record = rpm3;
	}
	else
	{
		 rpmcount = 0;
		 rpm_record = rpm3;
	}

	if (rpmcount >10) { rpm3 = 10; }

	struct rpm3_status_s rpm3_statuss;
	rpm3_statuss.esc_rpm = rpm3;
	rpm3_statuss.timestamp = hrt_absolute_time();
	if (_esc_rpm_topic != nullptr) {
		orb_publish(ORB_ID(rpm3_status), _esc_rpm_topic, &rpm3_statuss);
	}

	_reports->force(&rpm3_statuss);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void MOTOR3::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MOTOR3::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void MOTOR3::stop()
{
	work_cancel(HPWORK, &_work);
}

void MOTOR3::cycle_trampoline(void *arg)
{
	MOTOR3 *dev = (MOTOR3 *)arg;

	dev->cycle();
}

void MOTOR3::cycle()
{
	/* Collect results */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MOTOR3::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));

}

void MOTOR3::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace motor3
{

MOTOR3	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void start()
{
	int fd = -1;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new MOTOR3(MOTOR3_BUS);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(MOTOR3_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		::close(fd);
		goto fail;
	}

	::close(fd);
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void test()
{
	struct rpm3_status_s report_test;
	ssize_t sz;
	int ret;

	int fd = open(MOTOR3_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'motor3 start' if the driver is not running", MOTOR3_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report_test, sizeof(report_test));

	if (sz != sizeof(report_test)) {
		err(1, "immediate read failed");
	}

	print_message(report_test);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}
		/* now go get it */
		sz = read(fd, &report_test, sizeof(report_test));

		if (sz != sizeof(report_test)) {
			err(1, "periodic read failed");
		}
		print_message(report_test);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	::close(fd);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void reset()
{
	int fd = open(MOTOR3_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	::close(fd);
	exit(0);
}

/**
 * Print a little info about the driver.
 */
void info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */

int motor3_main(int argc, char *argv[])
{
	// check for optional arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int i2cdevice = MOTOR3_BUS;


	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			i2cdevice = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting distance sensor orientation to %d", (int)i2cdevice);
			break;

		default:
			PX4_WARN("Unknown option!");
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		motor3::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		motor3::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		motor3::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		motor3::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		motor3::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}
