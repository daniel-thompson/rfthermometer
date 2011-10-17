/*
 * thermoman.c
 *
 * Main functions for thermoman, a driver for a USB thermometer
 *
 * Copyright (C) 2011 Daniel Thompson <daniel@redfelineninja.org.uk>
 * Copyright (C) 2008 OBJECTIVE DEVELOPMENT Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define _GNU_SOURCE

#include <assert.h>
#include <ctype.h>
#include <getopt.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "hiddata.h"
#include "../firmware/usbconfig.h"  /* for device VID, PID, vendor name and product name */

/* ------------------------------------------------------------------------- */

typedef enum {
	ID_INTERNAL,
	ID_EXTERNAL_1_1,
	ID_EXTERNAL_2_56,
	ID_EXTERNAL_3_3,
	ID_MAX,
	ID_INVALID,
} rfsense_id_t;

typedef struct {
	uint16_t minadc;
	uint16_t minunit;
	uint16_t maxadc;
	uint16_t maxunit;
} rfsense_calibration_t;

typedef struct {
	uint8_t osccal_boot;
	uint8_t osccal_actual;

	uint16_t current[ID_MAX];
	uint16_t min[ID_MAX];
	uint16_t max[ID_MAX];

	rfsense_calibration_t calib[ID_MAX];
} rfsense_state_t;

typedef struct {
	double current[ID_MAX];
	double min[ID_MAX];
	double max[ID_MAX];
} rfsense_units_t;

/* ------------------------------------------------------------------------- */

rfsense_calibration_t rfsense_volts[ID_MAX] = {
	{ 0, 0, 0x3ff, 110 },
	{ 0, 0, 0x3ff, 110 },
	{ 0, 0, 0x3ff, 256 },
	{ 0, 0, 0x3ff, 330 }
};

rfsense_calibration_t rfsense_temp[ID_MAX] = {
	{ 300, 2500, 370, 9500 }, /* ATTinyx5 datasheet */
	{ 372,    0, 934, 3100 }, /* MCP9701(A) */
	{ 160,    0, 401, 3100 }, /* MCP9701(A) */
	{ 124,    0, 311, 3100 }  /* MCP9701(A) */
};

double rfsense_get_unit(uint16_t adc, const rfsense_calibration_t *calib)
{
	double x = adc;

	double x1 = calib->minadc;
	double y1 = calib->minunit / 100;
	double x2 = calib->maxadc;
	double y2 = calib->maxunit / 100;
	
	double m = -(y2 - y1) / (x1 - x2);
	double c = -((m*x1) - y1);

	return m*x + c;
}

void rfsense_decode(const uint8_t *report, rfsense_state_t *state)
{
#define R8(offset) (report[offset])
#define R16(offset) ((R8(offset+1) << 8) | R8(offset))

	int i;

	state->osccal_boot = R8(0);
	state->osccal_actual = R8(1);

	for (i=0; i<4; i++) {
		state->current[i] = R16(2 + 2*i);

		state->min[i] = R16(10 + 2*i);
		state->max[i] = R16(18 + 2*i);

		state->calib[i].minadc = R16(26 + 8*i);
		state->calib[i].minunit = R16(28 + 8*i);
		state->calib[i].maxadc = R16(30 + 8*i);
		state->calib[i].maxunit = R16(32 + 8*i);
	}
#undef R8
#undef R16
}

void rfsense_state_to_units(const rfsense_state_t *state,
			    const rfsense_calibration_t *calib,
			    rfsense_units_t *units)
{
	int i;

	if (NULL == calib)
		calib = state->calib;

	for (i=0; i<ID_MAX; i++) {
		units->current[i] = rfsense_get_unit(state->current[i], &calib[i]);
		units->min[i] = rfsense_get_unit(state->min[i], &calib[i]);
		units->max[i] = rfsense_get_unit(state->max[i], &calib[i]);
	}		
}

void rfsense_encode(const rfsense_state_t *state, uint8_t *report)
{
#define W8(offset, val) (report[offset] = val)
#define W16(offset, val) (W8(offset+1, val >> 8), W8(offset, val & 0xff))
	int i;

	W8(0, state->osccal_boot);
	W8(1, state->osccal_actual);

	for (i=0; i<4; i++) {
		W16(2 + 2*i, state->current[i]);

		W16(10 + 2*i, state->min[i]);
		W16(18 + 2*i, state->max[i]);
		
		W16(26 + 8*i, state->calib[i].minadc);
		W16(28 + 8*i, state->calib[i].minunit);
		W16(30 + 8*i, state->calib[i].maxadc);
		W16(32 + 8*i, state->calib[i].maxunit);
	}	
#undef W8
#undef W16
}

/**
 * Perform single ended calibration using the current temperature as the ADC reference.
 */
void rfsense_set_temp(rfsense_state_t *state, int temp)
{
	temp *= 100;

	state->calib[ID_INTERNAL].minunit = temp;
	state->calib[ID_INTERNAL].minadc = state->current[ID_INTERNAL];
	state->calib[ID_INTERNAL].maxunit = temp + 2000;
	state->calib[ID_INTERNAL].maxadc = state->current[ID_INTERNAL] + 20;

	// this one can overflow easily so we go down rather than up
	state->calib[ID_EXTERNAL_1_1].minunit = temp - 800;
	state->calib[ID_EXTERNAL_1_1].minadc = state->current[ID_EXTERNAL_1_1] - 145;
	state->calib[ID_EXTERNAL_1_1].maxunit = temp;
	state->calib[ID_EXTERNAL_1_1].maxadc = state->current[ID_EXTERNAL_1_1];

	state->calib[ID_EXTERNAL_2_56].minunit = temp;
	state->calib[ID_EXTERNAL_2_56].minadc = state->current[ID_EXTERNAL_2_56];
	state->calib[ID_EXTERNAL_2_56].maxunit = temp + 2200;
	state->calib[ID_EXTERNAL_2_56].maxadc = state->current[ID_EXTERNAL_2_56] + 171;

	state->calib[ID_EXTERNAL_3_3].minunit = temp;
	state->calib[ID_EXTERNAL_3_3].minadc = state->current[ID_EXTERNAL_3_3];
	state->calib[ID_EXTERNAL_3_3].maxunit = temp + 3100;
	state->calib[ID_EXTERNAL_3_3].maxadc = state->current[ID_EXTERNAL_3_3] + 187;
}

void _rfsense_print_raw(uint16_t *adc, double *unit, FILE *fp)
{
	fprintf(fp, "%5d (%5.2f)  ", adc[ID_INTERNAL], unit[ID_INTERNAL]);
	fprintf(fp, "%5d (%5.2f)  ", adc[ID_EXTERNAL_1_1], unit[ID_EXTERNAL_1_1]);
	fprintf(fp, "%5d (%5.2f)  ", adc[ID_EXTERNAL_2_56], unit[ID_EXTERNAL_2_56]);
	fprintf(fp, "%5d (%5.2f)\n", adc[ID_EXTERNAL_3_3], unit[ID_EXTERNAL_3_3]);
}

void rfsense_print_raw(rfsense_state_t *state, FILE *fp)
{
	rfsense_units_t units;
	rfsense_state_to_units(state, NULL, &units);


	fprintf(fp, "OSCCAL:     Actual 0x%02x    Stored 0x%02x\n",
		state->osccal_actual, state->osccal_boot);

	fprintf(fp, "Current:  ");
	_rfsense_print_raw(state->current, units.current, fp);

        fprintf(fp, "Minimum:  ");
	_rfsense_print_raw(state->min, units.min, fp);

	fprintf(fp, "Maximum:  ");
	_rfsense_print_raw(state->max, units.max, fp);
}

/**
 * Display the results from a sensor.
 *
 * Note the, unlike the raw results, we only show one decimal place.
 * This is to avoid "tricking" the user about the precision offered
 * by the hardware.
 */
void rfsense_print(rfsense_state_t *state, rfsense_id_t id, FILE *fp)
{
	assert(id < ID_MAX);

	rfsense_units_t units;
	rfsense_state_to_units(state, NULL, &units);

	fprintf(fp, "%4.1f        Min %4.1f    Max %4.1f\n",
			units.current[id], units.min[id], units.max[id]);
}

/* ------------------------------------------------------------------------- */

static char *usbErrorMessage(int errCode)
{
static char buffer[80];

    switch(errCode){
        case USBOPEN_ERR_ACCESS:      return "Access to device denied";
        case USBOPEN_ERR_NOTFOUND:    return "The specified device was not found";
        case USBOPEN_ERR_IO:          return "Communication error with device";
        default:
            sprintf(buffer, "Unknown USB error %d", errCode);
            return buffer;
    }
    return NULL;    /* not reached */
}

static usbDevice_t  *openDevice(void)
{
usbDevice_t     *dev = NULL;
unsigned char   rawVid[2] = {USB_CFG_VENDOR_ID}, rawPid[2] = {USB_CFG_DEVICE_ID};
char            vendorName[] = {USB_CFG_VENDOR_NAME, 0}, productName[] = {USB_CFG_DEVICE_NAME, 0};
int             vid = rawVid[0] + 256 * rawVid[1];
int             pid = rawPid[0] + 256 * rawPid[1];
int             err;

    if((err = usbhidOpenDevice(&dev, vid, vendorName, pid, productName, 0)) != 0){
        fprintf(stderr, "error finding %s: %s\n", productName, usbErrorMessage(err));
        return NULL;
    }
    return dev;
}

/* ------------------------------------------------------------------------- */

static void hexdump(uint8_t *buffer, int len)
{
int     i;
FILE    *fp = stdout;

    for(i = 0; i < len; i++){
        if(i != 0){
            if(i % 16 == 0){
                fprintf(fp, "\n");
            }else{
                fprintf(fp, " ");
            }
        }
        fprintf(fp, "0x%02x", buffer[i] & 0xff);
    }
    if(i != 0)
        fprintf(fp, "\n");
}

#if 0
static int  hexread(uint8_t *buffer, char *string, int buflen)
{
char    *s;
int     pos = 0;

    while((s = strtok(string, ", ")) != NULL && pos < buflen){
        string = NULL;
        buffer[pos++] = (char)strtol(s, NULL, 0);
    }
    return pos;
}
#endif

/* ------------------------------------------------------------------------- */



static void issue_help(char *myName)
{
	printf(
"Usage: %s [OPTION]...\n"
"Driver for the RFThermometer. Shows the current temperature by\n"
"default but can also be used to calibrate the device using both\n"
"single and double ended methods\n"
"\n"
"Mandatory arguments to long options are mandatory for short options too.\n"
"  -c, --calibrate=SENSOR:MIN:MAX:MINADC:MAXADC\n"
"                             calibrate a specific sensor using the supplied\n"
"                             range (e.g. --calibrate=2v:20:52:440:508 ).\n"
"                             Sensor names are: int, 1v, 2v and 3v\n"
"      --factory-calibration  adopt factory calibration\n"
"  -h, --help                 display this help and exit\n"
"  -r, --reset                reset the min/max values\n"
"  -s, --show=SENSOR          show results measured with SENSOR (default: 3v)\n"
"  -H, --show-hex             show the raw hex packet exchanged with hardware\n"
"  -R, --show-raw             show the raw decimal values read from the h/ware\n"
"      --single-ended=TEMP    perform fast single ended calibration based on\n"
"                             the current temperature (leave time to warm up)\n"
"      --unsafe               enable operations that write to devices EEPROM\n"
"\n",
	       myName);
}

static rfsense_id_t unpack_id(const char *sid)
{
	if (0 == strcmp(sid, "int")) {
		return ID_INTERNAL;
	} else if (0 == strcmp(sid, "1v")) {
		return ID_EXTERNAL_1_1;
	} else if (0 == strcmp(sid, "2v")) {
		return ID_EXTERNAL_2_56;
	} else if (0 == strcmp(sid, "3v")) {
		return ID_EXTERNAL_3_3;
	}

	return ID_INVALID;
}

static int unpack_calibration(const char *s, rfsense_id_t *idp, rfsense_calibration_t *calibration)
{
	char buf[80]; // this is very generous
	char delim[] = ":, ";
	int len;
	char *sid, *smin, *smax, *sminadc, *smaxadc, *snull;
	rfsense_id_t id;

	len = strlen(s) + 1;
	if (len > sizeof(buf))
		return -1;

	memcpy(buf, s, len);

	sid = strtok(buf, delim);
	smin = strtok(NULL, delim);
	smax = strtok(NULL, delim);
	sminadc = strtok(NULL, delim);
	smaxadc = strtok(NULL, delim);
	snull = strtok(NULL, delim);

	if (smaxadc == NULL || snull != NULL)
		return -1;

	if (!isdigit(smin[0]) || !isdigit(smax[0]) || !isdigit(sminadc[0]) || !isdigit(smaxadc[0]))
		return -1;

	id = unpack_id(sid);
	if (id == ID_INVALID)
		return -1;

	// success guaranteed after this point so wecan start modifying the
	// out parameters

	*idp = id;
	calibration->minunit = atof(smin) * 100.0;
	calibration->minadc = atoi(sminadc);
	calibration->maxunit = atof(smax) * 100.0;
	calibration->maxadc = atoi(smaxadc);

#if 0
	printf("%s -> %d   %s -> %d   %s -> %d   %s -> %d   %s -> %d\n",
		sid, *id,
		smin, calibration->minunit, sminadc, calibration->minadc,
		smax, calibration->maxunit, smaxadc, calibration->maxadc);
#endif
	return 0;
}

int main(int argc, char **argv)
{
	enum {
		OPT_FC = 256,
		OPT_SINGLE,
		OPT_UNSAFE,
	};

	static struct option long_options[] = {
		{ "calibrate", 1, 0, 'c' },
		{ "factory-calibration", 0, 0, OPT_FC },
		{ "help", 0, 0, 'h' },
		{ "reset", 0, 0, 'r' },
		{ "single-ended", 1, 0, OPT_SINGLE },
		{ "show", 1, 0, 's' },
		{ "show-hex", 0, 0, 'H' },
		{ "show-raw", 0, 0, 'R' },
		{ "unsafe", 0, 0, OPT_UNSAFE },
	};

	int opt;

	/* arguments */
	rfsense_id_t calibrate = ID_INVALID;
	bool factory_calibration = false;
	bool reset = false;
	rfsense_id_t show = ID_INVALID;
	bool show_raw = false;
	bool show_hex = false;
	bool single_ended = false;
	bool unsafe = false;

	int  calibration_point = 20;
	rfsense_calibration_t calibration;
	
	/* derived arguments */
	bool bad_args = false;
	bool do_write = false;

	usbDevice_t *dev;
	uint8_t     buffer[129];    /* room for dummy report ID */
	int len = sizeof(buffer);
	int         err;
	rfsense_state_t state;

	while ((opt = getopt_long(argc, argv, "chrs:HR", long_options, NULL)) != -1) {
		switch (opt) {
		case 'c': // --calibrate
			if (0 != unpack_calibration(optarg, &calibrate, &calibration)) {
				fprintf(stderr, "Bad calibration set: '%s'\n", optarg);
				bad_args = true;
			}
			break;

		case OPT_FC: // --factory-calibration
			factory_calibration = true;
			break;

		case 'h': // --help
			issue_help(argv[0]);
			exit(0);
			break;

		case 'r': // --reset
			reset = true;
			break;

		case 's': // --show=SENSOR
			show = unpack_id(optarg);
			if (ID_INVALID == show) {
				fprintf(stderr, "Bad sensor name: '%s'\n", optarg);
				bad_args = true;
			}
			break;

		case 'H': // --show-hex
			show_hex = true;
			break;

		case 'R': // --show-raw
			show_raw = true;
			break;

		case OPT_SINGLE: // --single-ended
			single_ended = true;
			if (isdigit(optarg[0])) {
				calibration_point = atoi(optarg);
			} else {
				fprintf(stderr, "Bad calibration point: '%s'\n", optarg);
				bad_args = true;
			}
			break;
	
		case OPT_UNSAFE: // --unsafe
			unsafe = true;
			break;

		default:
			fprintf(stderr, "Try '%s --help' for more information.\n",
				argv[0]);
			return 1;
		}
	}

	if (optind < argc && bad_args) {
		if (!bad_args)
			fprintf(stderr, "%s: Too many arguments\n", argv[0]);
		fprintf(stderr, "Try '%s --help' for more information.\n", argv[0]);
		return 2;
	}

	if((dev = openDevice()) == NULL)
		return 3;

	if((err = usbhidGetReport(dev, 0, (char *) buffer, &len)) != 0) {
		fprintf(stderr, "error reading data: %s\n", usbErrorMessage(err));
		return 4;
	}

	rfsense_decode(buffer+1, &state);

	if (reset) {
		memset(state.min, 0xff, sizeof(state.min));
		memset(state.max, 0, sizeof(state.max));
	       
		do_write = true;
	}

	if (factory_calibration) {
		if (!unsafe) {
			fprintf(stderr, "Factory calibration will destroy data, requires --unsafe\n");
		} else {
			memcpy(state.calib, rfsense_temp, sizeof(state.calib));
			do_write = true;
		}
	}

	if (single_ended) {
		if (!unsafe) {
			fprintf(stderr, "Single ended calibration will destroy data, requires --unsafe\n");
		} else {
			rfsense_set_temp(&state, calibration_point);
			do_write = true;
		}
	}

	if (calibrate != ID_INVALID) {
		if (!unsafe) {
			fprintf(stderr, "Calibration destroys existing values, requires --unsafe\n");
		} else {
			state.calib[calibrate] = calibration;
			do_write = true;
		}
	}

	if (do_write) {
		rfsense_encode(&state, buffer+1);

		if((err = usbhidSetReport(dev, buffer, sizeof(buffer))) != 0)
			fprintf(stderr, "error writing data: %s\n",
				usbErrorMessage(err));
	}

	// we've now finished interacting with the USB device
	usbhidCloseDevice(dev);

	// select a default display mode if not other activity was requested
	if (!do_write && !reset && !show_raw && !show_hex && show == ID_INVALID)
		show = ID_EXTERNAL_3_3;

	if (show_raw)
		rfsense_print_raw(&state, stdout);

	if (show_hex)
		hexdump(buffer+1, sizeof(buffer)-1);

	if (show != ID_INVALID)
		rfsense_print(&state, show, stdout);

	return 0;
}

/* ------------------------------------------------------------------------- */
