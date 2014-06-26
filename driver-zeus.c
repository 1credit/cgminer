/*
 * Copyright 2013-2014 Con Kolivas <kernel@kolivas.org>
 * Copyright 2014 Zeus Integrated Systems Limited
 * Copyright 2014 Dominik Lehner
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#ifndef WIN32
  #include <sys/select.h>
  #include <termios.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #ifndef O_CLOEXEC
    #define O_CLOEXEC 0
  #endif
#else
  #include "compat.h"
  #include <windows.h>
  #include <io.h>
#endif

#include "miner.h"
#include "usbutils.h"
#include "elist.h"
#include "util.h"
#include "driver-zeus.h"

// Configuration options
extern bool opt_zeus_debug;
extern int opt_zeus_chips_count;		// number of Zeus chips chained together
extern int opt_zeus_chip_clk;			// frequency to run chips with
extern bool opt_zeus_nocheck_golden;		// bypass hashrate check

//static int opt_zeus_chips_count_max = 1;	// smallest power of 2 >= opt_zeus_chips_count
						// is currently auto-calculated

// Index for device-specific options
//static int option_offset = -1;

static struct name_chip_map {
	char	*model_name;
	int	chips_count;
} zeus_models[] = {
	{ "Blizzard",		6  },
	//{ "Cyclone",		96 },	// model renamed??
	{ "Hurricane X2",	48 },
	{ "Hurricane X3",	64 },
	{ "Thunder X2",		96 },
	{ "Thunder X3",		128 },
	{ NULL, 0 }
};

static const char golden_ob[] =
		"55aa0001"
		"00038000063b0b1b028f32535e900609c15dc49a42b1d8492a6dd4f8f15295c989a1decf584a6aa93be26066d3185f55ef635b5865a7a79b7fa74121a6bb819da416328a9bd2f8cef72794bf02000000";

static const char golden_ob2[] =
		"55aa00ff"
		"c00278894532091be6f16a5381ad33619dacb9e6a4a6e79956aac97b51112bfb93dc450b8fc765181a344b6244d42d78625f5c39463bbfdc10405ff711dc1222dd065b015ac9c2c66e28da7202000000";

/************************************************************
 * Utility Functions
 ************************************************************/

static void rev(unsigned char *s, size_t l)
{
	size_t i, j;
	unsigned char t;

	for (i = 0, j = l - 1; i < j; i++, j--) {
		t = s[i];
		s[i] = s[j];
		s[j] = t;
	}
}

static int log_2(int value)
{
	int x = 0;
	while (value > 1) {
		value >>= 1;
		x++;
	}
	return x;
}

static uint32_t __maybe_unused chip_index(uint32_t value, int bit_num)
{
	uint32_t newvalue = 0;
	int i;

	// isolate bits 19-28, then shift right to get the
	// highest bits that distinguish multiple chips
	value = (value & 0x1ff80000) >> (29 - bit_num);

	for (i = 0; i < bit_num; i++) {
		newvalue = newvalue << 1;
		newvalue += value & 0x01;
		value = value >> 1;
	}

	return newvalue;
}

static int lowest_pow2(int min)
{
	int i;
	for (i = 1; i < 1024; i = i * 2) {
		if (min <= i){
			return i;
		}
	}
	return 1024;
}

/************************************************************
 * Detection and setup
 ************************************************************/

static unsigned char zeus_clk_to_freqcode(int clkfreq)
{
	if (clkfreq > ZEUS_CLK_MAX) {
		applog(LOG_WARNING, "Clock frequency %d too high, resetting to %d",
								clkfreq, ZEUS_CLK_MAX);
		clkfreq = ZEUS_CLK_MAX;
	}

	if (clkfreq < ZEUS_CLK_MIN) {
		applog(LOG_WARNING, "Clock frequency %d too low, resetting to %d",
								clkfreq, ZEUS_CLK_MIN);
		clkfreq = ZEUS_CLK_MIN;
	}

	return (unsigned char)((double)clkfreq * 2. / 3.);
}

static void zeus_get_device_options(const char *devid, int *chips_count, int *chip_clk, const char *options)
{
	char *p, *all, *found = NULL;
	long lval;
	int index = 0;

	// set global default options
	*chips_count = (opt_zeus_chips_count) ? opt_zeus_chips_count : ZEUS_MIN_CHIPS;
	*chip_clk = (opt_zeus_chip_clk) ? opt_zeus_chip_clk : ZEUS_CLK_MIN;

	if (options == NULL)
		return;

	all = strdup(options);

	for (p = strtok(all, ";"); p != NULL; p = strtok(NULL, ";")) {
		if (strncmp(p, devid, strlen(devid)) == 0) {
			found = p;
			break;
		}
	}

	if (found == NULL) {
		free(all);
		return;
	}

	for (p = strtok(found, ","); p != NULL; p = strtok(NULL, ",")) {
		lval = strtol(p, NULL, 10);

		switch (index++) {
			case 1:			// chip count
				if (lval < ZEUS_MIN_CHIPS || lval > ZEUS_MAX_CHIPS) {
					applog(LOG_ERR, "Invalid chip count %ld for Zeus device %s",
					       lval, devid);
					break;
				}
				*chips_count = (int)lval;
				break;
			case 2:			// clock
				if (lval < ZEUS_CLK_MIN || lval > ZEUS_CLK_MAX) {
					applog(LOG_ERR, "Invalid clock speed %ld for Zeus device %s",
					       lval, devid);
					break;
				}
				*chip_clk = (int)lval;
				break;
			default:
				break;
		}
	}

	free(all);
	return;
}

static char *zeus_device_name(int chips_count)
{
	struct name_chip_map *p;

	for (p = zeus_models; p->model_name != NULL; ++p) {
		if (p->chips_count == chips_count)
			return p->model_name;
	}

	return NULL;
}

static int zeus_usb_control_transfer(struct cgpu_info *zeus, uint8_t request_type, uint8_t bRequest,
		uint16_t wValue, uint16_t wIndex, uint32_t *data, int siz, enum usb_cmds cmd)
{
	int err = usb_transfer_data(zeus, request_type, bRequest, wValue, wIndex, data, siz, cmd);
	if (err)
		applog(LOG_DEBUG, "%s%d: error %d on USB control transfer %s",
			zeus->drv->name, zeus->cgminer_id, err, usb_cmdname(cmd));
	return err;
}

static bool zeus_initialize_usbuart(struct cgpu_info *zeus)
{
	int interface = usb_interface(zeus);
	//uint32_t baudrate = CP210X_DATA_BAUD;

	// Enable the UART
	if (zeus_usb_control_transfer(zeus, CP210X_TYPE_OUT, CP210X_REQUEST_IFC_ENABLE,
			CP210X_VALUE_UART_ENABLE, interface, NULL, 0, C_ENABLE_UART))
		return false;

	// Set data control
	if (zeus_usb_control_transfer(zeus, CP210X_TYPE_OUT, CP210X_REQUEST_DATA,
			CP210X_VALUE_DATA, interface, NULL, 0, C_SETDATA))
		return false;

	// Zeusminers have baud hardcoded to 115200, and reject baud commands, even to same value
	// Set the baud
	//if (zeus_usb_control_transfer(zeus, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD,
	//		0, interface, &baudrate, sizeof(baudrate), C_SETBAUD))
	//	return false;

	return true;
}

static struct cgpu_info *zeus_detect_one_usb(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *zeus;
	struct ZEUS_INFO *info;
	unsigned char freqcode_init;
	unsigned char ob_bin[ZEUS_COMMAND_PKT_LEN];
	int ret, i;

	zeus = usb_alloc_cgpu(&zeus_drv, 1);
	if (!usb_init(zeus, dev, found))
		goto usbdealloc;
	info = calloc(1, sizeof(struct ZEUS_INFO));
	if (unlikely(!info))
		goto usbdealloc;

	zeus->device_data = info;
	zeus->deven = DEV_ENABLED;
	zeus->threads = 1;

	zeus->unique_id = zeus->device_path;
	strncpy(info->device_name, zeus->unique_id, sizeof(info->device_name) - 1);
	info->device_name[sizeof(info->device_name) - 1] = '\0';

	zeus_get_device_options(zeus->device_path, &info->chips_count, &info->chip_clk, opt_zeus_options);
	zeus->name = zeus_device_name(info->chips_count);
	info->freqcode = zeus_clk_to_freqcode(info->chip_clk);
	info->baud = ZEUS_IO_SPEED;
	info->cores_per_chip = ZEUS_CHIP_CORES;
	info->chips_count_max = lowest_pow2(info->chips_count);
	info->chips_bit_num = log_2(info->chips_count_max);
	info->next_chip_clk = -1;

	libusb_reset_device(zeus->usbdev->handle);
	update_usb_stats(zeus);

	zeus->usbdev->usb_type = USB_TYPE_STD;
	if (!zeus_initialize_usbuart(zeus)) {
		applog(LOG_ERR, "Failed to initialize Zeus USB-UART interface");
		goto alldealloc;
	}

	// from 150M step to the high or low speed. we need to add delay and resend to init chip
	if (info->chip_clk > 150)
		freqcode_init = zeus_clk_to_freqcode(165);
	else
		freqcode_init = zeus_clk_to_freqcode(139);

	hex2bin(ob_bin, golden_ob2, sizeof(ob_bin));
	ob_bin[0] = freqcode_init;
	ob_bin[1] = ~freqcode_init;
	ob_bin[2] = 0x00;
	ob_bin[3] = 0x01;
	for (i = 0; i < 2; ++i) {
		if (usb_write(zeus, (char *)ob_bin, sizeof(ob_bin), &ret, C_SENDWORK) != LIBUSB_SUCCESS ||
			ret != sizeof(ob_bin))
			goto alldealloc;
		cgsleep_ms(500);	// what is the minimum the miners need/will accept?
	}

	hex2bin(ob_bin, golden_ob2, sizeof(ob_bin));
	ob_bin[0] = info->freqcode;
	ob_bin[1] = ~(info->freqcode);
	ob_bin[2] = 0x00;
	ob_bin[3] = 0x01;
	for (i = 0; i < 2; ++i) {
		if (usb_write(zeus, (char *)ob_bin, sizeof(ob_bin), &ret, C_SENDWORK) != LIBUSB_SUCCESS ||
			ret != sizeof(ob_bin))
			goto alldealloc;
		cgsleep_ms(500);
	}

	info->golden_speed_per_core = (((info->chip_clk * 2.) / 3.) * 1024.) / 8.;
	info->work_timeout.tv_sec = 4294967296LL / (info->golden_speed_per_core * info->cores_per_chip * info->chips_count_max) * 0.9;
	info->work_timeout.tv_usec = 0;
	info->read_count = (uint32_t)((4294967296LL*10)/(info->cores_per_chip*info->chips_count_max*info->golden_speed_per_core*2));
	info->read_count = info->read_count*3/4;

	if (!add_cgpu(zeus))
		goto alldealloc;

	return zeus;

alldealloc:
	usb_uninit(zeus);
	free(zeus->device_data);
	zeus->device_data = NULL;

usbdealloc:
	zeus = usb_free_cgpu(zeus);
	return NULL;
}

/************************************************************
 * Host <-> ASIC protocol implementation
 ************************************************************/

static void zeus_purge_work(struct cgpu_info *zeus)
{
	struct ZEUS_INFO *info = zeus->device_data;
	if (info->current_work != NULL) {
		free_work(info->current_work);
		info->current_work = NULL;
	}
}

#define nonce_range_start(cperc, cmax, core, chip) \
	(((0xffffffff / cperc + 1) * core) + ((0x1fffffff / cmax + 1) * chip))
static bool zeus_read_response(struct cgpu_info *zeus, const unsigned char *evtpkt)
{
	struct ZEUS_INFO *info = zeus->device_data;
	int duration_ms;
	uint32_t nonce, chip, core;
	bool valid;

	memcpy(&nonce, evtpkt, ZEUS_EVENT_PKT_LEN);
	nonce = be32toh(nonce);

	if (info->current_work == NULL) {	// work was flushed before we read response
		applog(LOG_DEBUG, "%s%d: Received nonce for flushed work",
			zeus->drv->name, zeus->device_id);
		return true;
	}

	valid = submit_nonce(info->thr, info->current_work, nonce);

	//info->hashes_per_ms = (nonce % (0xffffffff / info->cores_per_chip / info->chips_count)) * info->cores_per_chip * info->chips_count / ms_tdiff(&info->workend, &info->workstart);
	//applog(LOG_INFO, "hashes_per_ms: %d", info->hashes_per_ms);

	++info->workdone;

	//chip = chip_index(nonce, info->chips_bit_num);
	core = (nonce & 0xe0000000) >> 29;		// core indicated by 3 highest bits
	chip = (nonce & 0x1ff80000) >> (29 - info->chips_bit_num);
	duration_ms = ms_tdiff(&info->workend, &info->workstart);

	if (chip < ZEUS_MAX_CHIPS && core < ZEUS_CHIP_CORES) {
		++info->nonce_count[chip][core];
		if (!valid)
			++info->error_count[chip][core];

		if (valid && duration_ms > 0) {
			info->hashes_per_ms = (nonce - nonce_range_start(info->cores_per_chip, info->chips_count_max, core, chip)) / duration_ms * info->cores_per_chip * info->chips_count;
			info->last_nonce = nonce;
		}
	} else {
		applog(LOG_INFO, "%s%d: Corrupt nonce message received, cannot determine chip and core",
			zeus->drv->name, zeus->device_id);
	}

	return true;
}

static bool zeus_check_need_work(struct cgpu_info *zeus)
{
	struct ZEUS_INFO *info = zeus->device_data;
	struct thr_info *thr = info->thr;
	struct work *work;
	bool need_work;

	need_work = (info->current_work == NULL);

	if (need_work) {
		work = get_work(thr, thr->id);  // get_work can block, so done outside mutex_lock

		mutex_lock(&info->lock);
		if (info->current_work == NULL) {  // verify still NULL
			work->devflag = false;
			info->current_work = work;
		} else {
			need_work = false;
		}
		mutex_unlock(&info->lock);

		if (!need_work)
			discard_work(work);
	}

	return need_work;
}

static bool zeus_send_work(struct cgpu_info *zeus, struct work *work)
{
	struct ZEUS_INFO *info = zeus->device_data;
	unsigned char cmdpkt[ZEUS_COMMAND_PKT_LEN];
	int ret;
	uint32_t diff_code, diff;

	diff = work->work_difficulty;
	if (diff < 1)
		diff = 1;

	diff_code = 0xffff / diff;
	applog(LOG_DEBUG, "zeus_send_work: diff=%d diff_code=%04x", diff, diff_code);

	cmdpkt[0] = info->freqcode;
	cmdpkt[1] = ~(info->freqcode);
	cmdpkt[2] = (diff_code & 0xff00) >> 8;
	cmdpkt[3] = (diff_code & 0x00ff);

	memcpy(cmdpkt + 4, work->data, 80);
	rev(cmdpkt + 4, 80);

	if (usb_write(zeus, (char *)cmdpkt, sizeof(cmdpkt), &ret, C_SENDWORK) != LIBUSB_SUCCESS ||
		ret != sizeof(cmdpkt))
		return false;

	return true;
}

static void *zeus_io_thread(void *data)
{
	struct cgpu_info *zeus = (struct cgpu_info *)data;
	struct ZEUS_INFO *info = zeus->device_data;
	char threadname[24];
	struct timeval tv_now, tv_spent, tv_rem;
	int retval;

	snprintf(threadname, sizeof(threadname), "Zeus/%d", zeus->device_id);
	RenameThread(threadname);
	applog(LOG_INFO, "%s%d: work send thread running, %s",
			zeus->drv->name, zeus->device_id, threadname);

	while (likely(!zeus->shutdown)) {
		zeus_check_need_work(zeus);

		mutex_lock(&info->lock);
		if (info->current_work && !info->current_work->devflag) {
			/* send task to device */
			if (opt_zeus_debug)
				applog(LOG_INFO, "Sending work");

			if (zeus_send_work(zeus, info->current_work)) {
				info->current_work->devflag = true;
				cgtime(&info->workstart);
				if (info->next_chip_clk != -1) {
					info->chip_clk = info->next_chip_clk;
					info->next_chip_clk = -1;
				}
			} else {
				mutex_unlock(&info->lock);
				applog(LOG_NOTICE, "%s%d: I/O error while sending work, will retry",
					zeus->drv->name, zeus->device_id);
				continue;
			}
		}
		mutex_unlock(&info->lock);

		cgtime(&tv_now);
		timersub(&tv_now, &info->workstart, &tv_spent);
		timersub(&info->work_timeout, &tv_spent, &tv_rem);

		if (opt_zeus_debug) {
			applog(LOG_DEBUG, "Workstart: %d.%06d", (int)info->workstart.tv_sec, (int)info->workstart.tv_usec);
			applog(LOG_DEBUG, "Spent: %d.%06d", (int)tv_spent.tv_sec, (int)tv_spent.tv_usec);
			applog(LOG_DEBUG, "Remaining: %d.%06d", (int)tv_rem.tv_sec, (int)tv_rem.tv_usec);
		}

		retval = cgsem_mswait(&info->wusem, (tv_rem.tv_sec < 1) ? 5000 : tv_rem.tv_sec * 1000);
		if (retval < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (retval == ETIMEDOUT) {
			mutex_lock(&info->lock);
			zeus_purge_work(zeus);		// abandon current work
			mutex_unlock(&info->lock);
		}
	}

	zeus->shutdown = true;
	return NULL;
}

/************************************************************
 * CGMiner Interface functions
 ************************************************************/

static void zeus_detect(bool __maybe_unused hotplug)
{
	usb_detect(&zeus_drv, zeus_detect_one_usb);
}

static bool zeus_prepare(struct thr_info *thr)
{
	struct cgpu_info *zeus = thr->cgpu;
	struct ZEUS_INFO *info = zeus->device_data;

	applog(LOG_NOTICE, "%s%d opened on %s",
			zeus->drv->name, zeus->device_id, zeus->device_path);

	info->thr = thr;
	mutex_init(&info->lock);
	cgsem_init(&info->wusem);

	return true;
}

static bool zeus_thread_init(struct thr_info *thr)
{
	struct cgpu_info *zeus = thr->cgpu;
	struct ZEUS_INFO *info = zeus->device_data;

	if (pthread_create(&info->pth_io, NULL, zeus_io_thread, zeus)) {
		applog(LOG_ERR, "%s%d: Failed to create I/O thread",
				zeus->drv->name, zeus->device_id);
		return false;
	}

	return true;
}

#define ZEUS_LIVE_HASHRATE 1
static int64_t zeus_scanwork(struct thr_info *thr)
{
	struct cgpu_info *zeus = thr->cgpu;
	struct ZEUS_INFO *info = zeus->device_data;
	struct timeval old_scanwork_time;
	unsigned char evtpkt[ZEUS_EVENT_PKT_LEN];
	int err, ret;
	double elapsed_s;
	int64_t estimate_hashes;

	err = usb_read_timeout(zeus, (char *)evtpkt, sizeof(evtpkt), &ret, 250, C_GETRESULTS);
	if (err && err != LIBUSB_ERROR_TIMEOUT) {
		applog(LOG_ERR, "%s%d: USB read error: %s",
			zeus->drv->name, zeus->device_id, libusb_error_name(err));
		zeus->shutdown = true;
		cgsem_post(&info->wusem);
		return 0;
	} else if (err == LIBUSB_SUCCESS) {
		mutex_lock(&info->lock);
		zeus_read_response(zeus, evtpkt);
		mutex_unlock(&info->lock);
	}

	mutex_lock(&info->lock);
	old_scanwork_time = info->scanwork_time;
	cgtime(&info->scanwork_time);
	elapsed_s = tdiff(&info->scanwork_time, &old_scanwork_time);
#ifdef ZEUS_LIVE_HASHRATE
	estimate_hashes = elapsed_s * info->hashes_per_ms * 1000;
#else
	estimate_hashes = elapsed_s * info->golden_speed_per_core *
				info->cores_per_chip * info->chips_count;
#endif
	mutex_unlock(&info->lock);

	if (unlikely(estimate_hashes > 0xffffffff))
		estimate_hashes = 0xffffffff;

	return estimate_hashes;
}

#define zeus_update_work zeus_flush_work
static void zeus_flush_work(struct cgpu_info *zeus)
{
	struct ZEUS_INFO *info = zeus->device_data;
	mutex_lock(&info->lock);
	zeus_purge_work(zeus);
	cgsem_post(&info->wusem);
	mutex_unlock(&info->lock);
	if (opt_zeus_debug)
		applog(LOG_INFO, "zeus_flush_work: Post Semaphore");
}

static struct api_data *zeus_api_stats(struct cgpu_info *zeus)
{
	struct ZEUS_INFO *info = zeus->device_data;
	struct api_data *root = NULL;
	static struct timeval tv_now, tv_diff, tv_diff2;
	static double khs_core, khs_chip, khs_board;

	cgtime(&tv_now);
	timersub(&tv_now, &(info->workstart), &tv_diff);
	timersub(&(info->workend), &(info->workstart), &tv_diff2);

	root = api_add_string(root, "Device Name", zeus->unique_id, false);
	khs_core = (double)info->golden_speed_per_core / 1000.;
	khs_chip = (double)info->golden_speed_per_core * (double)info->cores_per_chip / 1000.;
	khs_board = (double)info->golden_speed_per_core * (double)info->cores_per_chip * (double)info->chips_count / 1000.;
	root = api_add_khs(root, "KHS/Core", &khs_core, false);
	root = api_add_khs(root, "KHS/Chip", &khs_chip, false);
	root = api_add_khs(root, "KHS/Board", &khs_board, false);
	root = api_add_int(root, "Frequency", &(info->chip_clk), false);
	root = api_add_int(root, "Cores/Chip", &(info->cores_per_chip), false);
	root = api_add_int(root, "Chips Count", &(info->chips_count), false);
	root = api_add_timeval(root, "Time Spent Current Work", &tv_diff, false);
	root = api_add_timeval(root, "Work Timeout", &(info->work_timeout), false);
	/* It would be nice to report per chip/core nonce and error counts,
	 * but with more powerful miners with > 100 chips each with 8 cores
	 * there is too much information and we'd overflow the api buffer.
	 * Perhaps another api command to query individual chips? */

	/* these values are more for diagnostic and debugging */
	if (opt_zeus_debug) {
		root = api_add_int(root, "chips_count_max", &(info->chips_count_max), false);
		root = api_add_int(root, "chips_bit_num", &(info->chips_bit_num), false);
		root = api_add_uint32(root, "read_count", &(info->read_count), false);

		root = api_add_uint32(root, "hashes_per_ms", &(info->hashes_per_ms), false);
		root = api_add_uint32(root, "last_nonce", &(info->last_nonce), false);
		root = api_add_timeval(root, "last_nonce_time", &tv_diff2, false);
	}

	return root;
}

static void zeus_get_statline_before(char *buf, size_t bufsiz, struct cgpu_info *zeus)
{
	struct ZEUS_INFO *info = zeus->device_data;
	if (zeus->name)
		tailsprintf(buf, bufsiz, "%-12s  %4d MHz  ", zeus->name, info->chip_clk);
	else
		tailsprintf(buf, bufsiz, "%4d chips  %4d MHz  ", info->chips_count, info->chip_clk);
}

static char *zeus_set_device(struct cgpu_info *zeus, char *option, char *setting, char *replybuf)
{
	struct ZEUS_INFO *info = zeus->device_data;
	int val;

	if (strcasecmp(option, "help") == 0) {
		sprintf(replybuf, "freq: range %d-%d, abortwork: true/false",
				ZEUS_CLK_MIN, ZEUS_CLK_MAX);
		return replybuf;
	}

	if (strcasecmp(option, "freq") == 0) {
		if (!setting || !*setting) {
			sprintf(replybuf, "missing freq setting");
			return replybuf;
		}

		val = atoi(setting);
		if (val < ZEUS_CLK_MIN || val > ZEUS_CLK_MAX) {
			sprintf(replybuf, "invalid freq: '%s' valid range %d-%d",
					setting, ZEUS_CLK_MIN, ZEUS_CLK_MAX);
			return replybuf;
		}

		mutex_lock(&info->lock);
		info->next_chip_clk = val;
		info->freqcode = zeus_clk_to_freqcode(val);
		mutex_unlock(&info->lock);
		return NULL;
	}

	if (strcasecmp(option, "abortwork") == 0) {
		if (!setting || !*setting) {
			sprintf(replybuf, "missing true/false");
			return replybuf;
		}

		if (strcasecmp(setting, "true") != 0) {
			sprintf(replybuf, "not aborting current work");
			return replybuf;
		}

		mutex_lock(&info->lock);
		zeus_purge_work(zeus);
		cgsem_post(&info->wusem);
		mutex_unlock(&info->lock);
		return NULL;
	}

	sprintf(replybuf, "Unknown option: %s", option);
	return replybuf;
}

static void zeus_shutdown(struct thr_info *thr)
{
	struct cgpu_info *zeus = thr->cgpu;
	struct ZEUS_INFO *info = zeus->device_data;

	applog(LOG_NOTICE, "%s%d: Shutting down", zeus->drv->name, zeus->device_id);

	pthread_join(info->pth_io, NULL);
	mutex_destroy(&info->lock);
	cgsem_destroy(&info->wusem);
}

struct device_drv zeus_drv = {
		.drv_id = DRIVER_zeus,
		.dname = "Zeus",
		.name = "ZUS",
		.max_diff = 32768,
		.drv_detect = zeus_detect,
		.thread_prepare = zeus_prepare,
		.thread_init = zeus_thread_init,
		.hash_work = hash_driver_work,
		.scanwork = zeus_scanwork,
		.flush_work = zeus_flush_work,
		//.update_work = zeus_update_work,	// redundant, always seems to be called together with flush_work ??
		.get_api_stats = zeus_api_stats,
		.get_statline_before = zeus_get_statline_before,
		.set_device = zeus_set_device,
		.thread_shutdown = zeus_shutdown,
};
