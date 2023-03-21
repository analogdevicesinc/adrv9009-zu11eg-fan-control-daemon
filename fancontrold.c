/**
 * Copyright (C) 2019 Analog Devices, Inc.
 *
 * Licensed under the ADI BSD.
 *
 **/
#define _GNU_SOURCE
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <stdint.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/reboot.h>
#include <dirent.h>
#include <syslog.h>
#include <iio.h>

#define emerg(fmt, ...) \
	syslog(LOG_EMERG, \
	    "%s, %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

#define error(fmt, ...) \
	syslog(LOG_ERR, \
	    "%s, %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

#define warn(fmt, ...)	\
	syslog(LOG_WARNING, \
	    "%s, %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

#define info(fmt, ...) \
	syslog(LOG_INFO, \
	    "%s, %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

#define debug(fmt, ...) \
do { \
	if (verbose) \
		syslog(LOG_DEBUG, \
		   "%s, %d: "fmt, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define ARRAY_SIZE(arr)		(sizeof(arr) / sizeof(*(arr)))

#define DEFAULT_SLEEP_TIME	1
#define TRUE			1
#define FALSE			0

#define PWM_25			64
#define PWM_50			128
#define PWM_75			191
#define PWM_100			255
/*
 * These thresholds are defined accordingly with
 * https://wiki.analog.com/resources/fpga/docs/axi_fan_control
 */
#define THRESH_PWM_000		5000
#define THRESH_PWM_025_L	20000
#define THRESH_PWM_025_H	40000
#define THRESH_PWM_050_L	60000
#define THRESH_PWM_050_H	70000
#define THRESH_PWM_075_L	80000
#define THRESH_PWM_075_H	90000
#define THRESH_PWM_100		95000
#define DEFAULT_DEVNAME		"axi_fan_control"
#define HWMON_PATH		"/sys/class/hwmon"
#define BUFLEN_256		256
#define BUFLEN_512		512
#define BUFLEN_768		768
#define DONT_CARE		-1
#define HMC7044_SLEEP_REG	0
#define HMC7044_SLEEP_VAL	"1"

static int running = TRUE;
static int verbose = FALSE;
/* default to 5 faulty fan reads (in a row) to try to shutdown the system */
static uint32_t fault_hyst = 5;

struct monitor_iio_dev_lookup {
	const char *name;
	/* temperature channel idx to look for */
	const int channel_idx;
	/* atrribute name */
	const char *attr;
};

struct monitor_iio_dev_shutdown {
	/* device name */
	const char *name;
	/* attribute name which controls the state of the device */
	const char *attr_name;
	/* value to disable the device */
	const char *attr_val;
	/* register to be used on direct register access cases */
	const int reg;
};

struct monitor_dev_shutdown {
	struct monitor_iio_dev_shutdown iio_dev;
	void (*iio_shutdown) (const struct monitor_iio_dev_shutdown *iio_dev);
};

static void monitor_iio_dev_shutdown(
		const struct monitor_iio_dev_shutdown *iio_dev);
static void monitor_iio_dev_debug_shutdown(
		const struct monitor_iio_dev_shutdown *iio_dev);
static void clean_up(void);

static const struct monitor_dev_shutdown monitor_shutdown_list[] = {
	{{"adrv9009-phy", "ensm_mode", "radio_off", DONT_CARE},
		monitor_iio_dev_shutdown},
	{{"adrv9009-phy-b", "ensm_mode", "radio_off", DONT_CARE},
		monitor_iio_dev_shutdown},
	{{"hmc7044", NULL, HMC7044_SLEEP_VAL, HMC7044_SLEEP_REG},
		monitor_iio_dev_debug_shutdown},
};

/* For now just hard code the iio device to monitor */
static const struct monitor_iio_dev_lookup iio_list[] = {
	{"adrv9009-phy", 0, "input"},
	{"adrv9009-phy-b", 0, "input"},
};

struct monitor_iio_dev {
	struct iio_channel *ch;
	const char *attr;
};

struct monitor {
	char *path;
	struct iio_context *ctx;
	struct monitor_iio_dev *iio_devs;
	uint32_t numb_devs;
	uint8_t pwm;
} fan_monitor;

static void usage(char *argv[])
{
	printf("Usage: %s [OPTIONS]... [HWMON_NAME]\n", argv[0]);
	printf(" Copyright (C) 2019 Analog Devices, Inc.\n"
		" This is free software; see the source for copying conditions.\n"
		" There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A\n"
		" PARTICULAR PURPOSE.\n\n");
	printf("  -v, --verbose\t\tVerbose.\n");
	printf("  -s, --sleep\t\tSleep time between temperature checks.\n");
	printf("  -f, --fault-cnt\tNumber of consecutive FAN FAULT reads before trying to shutdown the system\n");
	printf("  -h, --help\t\tPrint this help.\n");
	exit(EXIT_FAILURE);
}

static char *sysfs_read_attr(const char *path, const char *attr)
{
	char buf[NAME_MAX];
	char attr_path[BUFLEN_768];
	FILE *p;
	char *p_ret = NULL;

	snprintf(attr_path, sizeof(attr_path), "%s/%s", path, attr);
	p = fopen(attr_path, "r");
	if (!p) {
		error("fopen: %s\n", strerror(errno));
		return NULL;
	}

	p_ret = fgets(buf, sizeof(buf), p);
	fclose(p);
	if (!p_ret)
		return NULL;

	/* remove \n */
	p_ret = strndup(buf, strlen(buf) - 1);
	return p_ret;
}

static void sysfs_write_attr(const char *path, const char *attr,
			     const uint8_t pwm)
{
	char attr_path[BUFLEN_768];
	FILE *p;

	snprintf(attr_path, sizeof(attr_path), "%s/%s", path, attr);
	p = fopen(attr_path, "w");
	if (!p) {
		error("fopen: %s\n", strerror(errno));
		return;
	}

	if (fprintf(p, "%u", pwm) < 0)
		error("Failed to set sysfs attr: %s\n", strerror(errno));

	fclose(p);
}

static long monitor_iio_dev_attr_get(const struct monitor_iio_dev *dev)
{
	char buf[BUFLEN_256];
	ssize_t cnt = 0;
	long temp;
	char *endptr;

	cnt = iio_channel_attr_read(dev->ch, dev->attr, buf, sizeof(buf));
	if (cnt < 0) {
		error("Failed to read attr: %s\n", strerror(-cnt));
		return 0;
	}

	errno = 0;
	temp = strtol(buf, &endptr, 10);
	/* check possible failures */
	if (errno) {
		error("Failed to convert temp: %s\n", strerror(errno));
		return 0;
	} else if (endptr == buf) {
		error("No digits found\n");
		return 0;
	}

	return temp;
}

static uint8_t monitor_get_pwm(const long temperature)
{
	uint8_t pwm = 0;

	if (temperature <= THRESH_PWM_000) {
		pwm = 0;
	} else if (temperature >= THRESH_PWM_025_L &&
		   temperature <= THRESH_PWM_025_H) {
		pwm = PWM_25;
	} else if (temperature >= THRESH_PWM_050_L &&
		   temperature <= THRESH_PWM_050_H) {
		pwm = PWM_50;
	} else if (temperature >= THRESH_PWM_075_L &&
		   temperature <= THRESH_PWM_075_H) {
		pwm = PWM_75;
	} else if (temperature >= THRESH_PWM_100) {
		pwm = PWM_100;
	} else {
		/* do not change it */
		pwm = fan_monitor.pwm;
	}

	return pwm;
}

static void monitor_iio_dev_shutdown(
		const struct monitor_iio_dev_shutdown *iio_shutdown)
{
	struct iio_device *dev;

	if (!iio_shutdown->name || !iio_shutdown->attr_name ||
	    !iio_shutdown->attr_val) {
		error("IIO device not properly initialized\n");
		return;
	}

	dev = iio_context_find_device(fan_monitor.ctx, iio_shutdown->name);
	if (!dev) {
		warn("Could not find iio dev %s\n", iio_shutdown->name);
		return;
	}

	if (iio_device_attr_write_raw(dev, iio_shutdown->attr_name,
				  iio_shutdown->attr_val,
				  strlen(iio_shutdown->attr_val)) < 0)
		warn("Failed to disable dev:%s\n", iio_shutdown->name);
}

static void monitor_iio_dev_debug_shutdown(
		const struct monitor_iio_dev_shutdown *iio_shutdown)
{
	struct iio_device *dev;
	uint32_t val;

	if (!iio_shutdown->name || !iio_shutdown->attr_val ||
	    iio_shutdown->reg == DONT_CARE) {
		error("IIO device not properly initialized\n");
		return;
	}

	dev = iio_context_find_device(fan_monitor.ctx, iio_shutdown->name);
	if (!dev) {
		warn("Could not find iio dev %s\n", iio_shutdown->name);
		return;
	}

	val = atoi(iio_shutdown->attr_val);
	if (iio_device_reg_write(dev, iio_shutdown->reg, val) < 0)
		warn("Failed to disable dev:%s\n", iio_shutdown->name);
}

static void monitor_handle_fan_fault(void)
{
	char *fault;
	uint32_t fan_fault = 0;
	uint32_t dev;
	static uint32_t cnt = 0;

	fault = sysfs_read_attr(fan_monitor.path, "fan1_fault");
	if (!fault)
		return;

	fan_fault = atoi(fault);
	free(fault);

	if (!fan_fault) {
		cnt = 0;
		return;
	} else if (cnt++ < fault_hyst) {
		return;
	}

	emerg("FAN is faulty. System is going to poweroff!!!\n");
	/* disable all known devices */
	for (dev = 0; dev < ARRAY_SIZE(monitor_shutdown_list); dev++) {
		if (monitor_shutdown_list[dev].iio_shutdown)
			monitor_shutdown_list[dev].iio_shutdown(
					&monitor_shutdown_list[dev].iio_dev);
	}

	/* if we reach this point the fan is faulty, let's poweroff! */
	reboot(RB_POWER_OFF);
	clean_up();
	exit(EXIT_SUCCESS);
}

static void run(const time_t sleep)
{
	int ret;
	struct timeval refresh;

	while (TRUE) {
		uint32_t cnt;
		long temp, max_temp = 0;
		uint8_t new_pwm = 0;
		/* refresh timeout */
		refresh.tv_sec = sleep;
		refresh.tv_usec = 0;

		ret = TEMP_FAILURE_RETRY(select(0, NULL, NULL, NULL, &refresh));
		if (ret < 0) {
			error("select: %s", strerror(errno));
			break;
		}

		if (!running)
			break;

		monitor_handle_fan_fault();

		/* do monitoring... */
		for (cnt = 0; cnt < fan_monitor.numb_devs; cnt++) {
			temp = monitor_iio_dev_attr_get(
					&fan_monitor.iio_devs[cnt]);
			if (temp > max_temp)
				max_temp = temp;
		}

		debug("Monitor devices (temp=%ld)...\n", max_temp);

		new_pwm = monitor_get_pwm(max_temp);
		if (new_pwm != fan_monitor.pwm) {
			info("Set new pwm %d\n", new_pwm);
			sysfs_write_attr(fan_monitor.path, "pwm1", new_pwm);
			fan_monitor.pwm = new_pwm;
		}
	}
}

static void sig_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		info("Exit....\n");
		running = 0;
	}
}

static int hwmon_find(const char *dev_name)
{
	const char *name = (dev_name == NULL) ? DEFAULT_DEVNAME : dev_name;
	DIR *hwmon;
	struct dirent *dir;
	uint8_t found = FALSE;
	int ret = -ENODEV;

	hwmon = opendir(HWMON_PATH);
	if (!hwmon) {
		error("opendir: %s\n", strerror(errno));
		return -1;
	}

	dir = readdir(hwmon);
	while (dir != NULL && found == FALSE) {
		char path[BUFLEN_512];
		char *chip_name;

		if (dir->d_type != DT_LNK ||
		    strncmp(dir->d_name, "hwmon", strlen("hwmon")))
			goto next_dir;

		snprintf(path, sizeof(path), "%s/%s", HWMON_PATH, dir->d_name);
		chip_name = sysfs_read_attr(path, "name");
		if (chip_name && !strcmp(chip_name, name)) {
			char *pwm;

			found = TRUE;
			/* copy chip path */
			fan_monitor.path = calloc(1, strlen(path) + 1);
			if (!fan_monitor.path) {
				error("calloc: %s\n", strerror(errno));
				ret = -1;
				goto free_chip;
			}
			/* get current pwm */
			pwm = sysfs_read_attr(path, "pwm1");
			if (!pwm) {
				ret = -1;
				goto free_chip;
			}

			fan_monitor.pwm = atoi(pwm);
			free(pwm);
			strcpy(fan_monitor.path, path);
			info("Found (%s) in %s with p:%u\n", name,
							fan_monitor.path,
							fan_monitor.pwm);
			ret = 0;
		}
free_chip:
		free(chip_name);
next_dir:
		dir = readdir(hwmon);
	}

	closedir(hwmon);
	return ret;
}

static int register_signals(void)
{
	struct sigaction sa;
	sigset_t mask;

	memset(&sa, 0, sizeof(sa));

	sa.sa_handler = sig_handler;
	sigemptyset(&sa.sa_mask);
	sigemptyset(&mask);

	if (sigaction(SIGTERM, &sa, NULL) < 0) {
		error("sigaction: %s\n", strerror(errno));
		return -1;
	}

	if (sigaction(SIGINT, &sa, NULL) < 0) {
		error("sigaction: %s\n", strerror(errno));
		return -1;
	}

	sigaddset(&mask, SIGINT);
	sigaddset(&mask, SIGTERM);
	/* make sure these signals are unblocked */
	if (sigprocmask(SIG_UNBLOCK, &mask, NULL)) {
		error("sigprocmask: %s", strerror(errno));
		return -1;
	}

	return 0;
}

static int iio_devices_scan(void)
{
	uint32_t cnt;
	int ret = 0;

	fan_monitor.ctx = iio_create_default_context();
	if (!fan_monitor.ctx) {
		error("Failed to create iio context\n");
		return -1;
	}

	for (cnt = 0; cnt < ARRAY_SIZE(iio_list); cnt++) {

		struct  iio_device *dev = iio_context_find_device(
				fan_monitor.ctx, iio_list[cnt].name);
		struct iio_channel *channel;
		char in_temp[BUFLEN_256];
		struct monitor_iio_dev *saved_iio;

		if (!dev) {
			warn("Could not find (%s)\n", iio_list[cnt].name);
			continue;
		}

		/* make sure that the desired tempX channel exists */
		sprintf(in_temp, "temp%d", iio_list[cnt].channel_idx);
		channel = iio_device_find_channel(dev, in_temp, FALSE);
		if (!channel) {
			warn("Dev (%s) does not contain chann (%s)\n",
				iio_list[cnt].name, in_temp);
			continue;
		}

		/* look for the desired attr */
		if (!iio_channel_find_attr(channel, iio_list[cnt].attr)) {
			warn("Attr (%s) not found in chann (%s)\n",
			     iio_list[cnt].attr, in_temp);
			continue;
		}

		saved_iio = realloc(fan_monitor.iio_devs,
				    sizeof(struct monitor_iio_dev) *
				    (fan_monitor.numb_devs + 1));
		if (!saved_iio) {
			error("Failed to allocate memory %s\n",
						strerror(errno));
			ret = -1;
			break;
		}

		debug("Adding device (%s)\n", iio_list[cnt].name);
		/* re-set the pointer */
		fan_monitor.iio_devs = saved_iio;
		fan_monitor.iio_devs[fan_monitor.numb_devs].ch = channel;
		fan_monitor.iio_devs[fan_monitor.numb_devs].attr =
							iio_list[cnt].attr;
		fan_monitor.numb_devs++;
	}

	return (ret == -1) || (!fan_monitor.numb_devs) ? -1 : 0;
}

static void clean_up(void)
{
	free(fan_monitor.iio_devs);
	free(fan_monitor.path);

	if (fan_monitor.ctx)
		iio_context_destroy(fan_monitor.ctx);
}

int main(int argc, char *argv[])
{
	int c, ret = 0;
	time_t sleep = DEFAULT_SLEEP_TIME;
	const char *dev_name = NULL;

	const struct option long_opts[] =  {
		{ "help", no_argument, NULL, 'h' },
		{ "verbose", no_argument, NULL, 'v'},
		{ "sleep", required_argument, NULL, 's' },
		{ "fault-cnt", required_argument, NULL, 'f' },
		{ 0, 0, 0, 0 }
	};

	memset(&fan_monitor, 0, sizeof(fan_monitor));

	c = getopt_long(argc, argv, "s:f:vh", long_opts, NULL);
	while (c != -1) {
		switch (c) {
		case 's':
			sleep = atoi(optarg);
			if (!sleep) {
				error("Invalid argument for '-s, --sleep'\n");
				usage(argv);
			}
			break;
		case 'f':
			fault_hyst = atoi(optarg);
			if (!fault_hyst) {
				error("Invalid argument for '-f, --fault-cnt'\n");
				usage(argv);
			}
			break;
		case 'v':
			verbose = TRUE;
			break;
		case 'h':
			usage(argv);
			break;
		default:
			usage(argv);
		}

		c = getopt_long(argc, argv, "s:f:vh", long_opts, NULL);
	}

	/* check if we got a chip name */
	if (optind + 1 < argc) {
		error("Only one chip name can be given\n");
		usage(argv);
	} else if (optind + 1 == argc)
		dev_name = argv[optind];

	openlog("adrv9009-fan-control", 0, LOG_DAEMON);

	/* register signals */
	if (register_signals() < 0)
		exit(EXIT_FAILURE);

	/* lookup for the hwmon device */
	ret = hwmon_find(dev_name);
	if (ret < 0) {
		/*
		 * don't treat -ENODEV as error. Just treat it like "no device,
		 * no need for monitoring"
		 */
		if (ret == -ENODEV)
			ret = 0;

		goto err;
	}

	ret = iio_devices_scan();
	if (ret < 0)
		goto err;

	run(sleep);
err:
	clean_up();

	return ret;
}
