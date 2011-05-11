/* Name: hidtool.c
 * Project: hid-data example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-11
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: hidtool.c 723 2009-03-16 19:04:32Z cs $
 */

#include <stdio.h>
#include <string.h>
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
	ID_MAX
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

/* ------------------------------------------------------------------------- */

rfsense_calibration_t rfsense_volts[ID_MAX] = {
	{ 0, 0, 0x3ff, 110 },
	{ 0, 0, 0x3ff, 110 },
	{ 0, 0, 0x3ff, 256 },
	{ 0, 0, 0x3ff, 330 }
};

rfsense_calibration_t rfsense_temp[ID_MAX] = {
	{ 300, 2500, 370, 8500 }, /* ATTinyx5 datasheet */
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

void rfsense_print(rfsense_state_t *state, FILE *fp)
{
	double internal = rfsense_get_unit(state->current[ID_INTERNAL],
					   &state->calib[ID_INTERNAL]);
	double ext_1_1 = rfsense_get_unit(state->current[ID_EXTERNAL_1_1],
					  &state->calib[ID_EXTERNAL_1_1]);
	double ext_2_56 = rfsense_get_unit(state->current[ID_EXTERNAL_2_56],
					  &state->calib[ID_EXTERNAL_2_56]);
	double ext_3_3 = rfsense_get_unit(state->current[ID_EXTERNAL_3_3],
					  &state->calib[ID_EXTERNAL_3_3]);

	fprintf(fp, "OSCCAL: Actual 0x%02x  Stored 0x%02x\n",
		state->osccal_actual, state->osccal_boot);

	fprintf(fp, "Internal:        %5d  (%3.2f)\n",
		state->current[ID_INTERNAL], internal);
	fprintf(fp, "External (1.1):  %5d  (%3.2f)\n",
		state->current[ID_EXTERNAL_1_1], ext_1_1);
	fprintf(fp, "External (2.56): %5d  (%3.2f)\n",
		state->current[ID_EXTERNAL_2_56], ext_2_56);
	fprintf(fp, "External (3.3):  %5d  (%3.2f)\n",
		state->current[ID_EXTERNAL_3_3], ext_3_3);
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

/* ------------------------------------------------------------------------- */



static void usage(char *myName)
{
    fprintf(stderr, "usage:\n");
    fprintf(stderr, "  %s read\n", myName);
    fprintf(stderr, "  %s write <listofbytes>\n", myName);
}

int main(int argc, char **argv)
{
usbDevice_t *dev;
uint8_t     buffer[129];    /* room for dummy report ID */
int len = sizeof(buffer);
int         err;

    if(argc < 2){
        usage(argv[0]);
        exit(1);
    }
    if((dev = openDevice()) == NULL)
        exit(1);
    if((err = usbhidGetReport(dev, 0, (char *) buffer, &len)) != 0) {
         fprintf(stderr, "error reading data: %s\n", usbErrorMessage(err));
	 exit(2);
    }

    if(strcasecmp(argv[1], "read") == 0){
	rfsense_state_t state;

	hexdump(buffer + 1, sizeof(buffer) - 1);

	rfsense_decode(buffer+1, &state);
	rfsense_print(&state, stdout);
		
    } else if(strcasecmp(argv[1], "write") == 0){
	rfsense_state_t state;

	rfsense_decode(buffer+1, &state);
	rfsense_encode(&state, buffer+1);
	hexdump(buffer+1, sizeof(buffer)-1);

#if 0
        int i, pos;
        memset(buffer, 0, sizeof(buffer));

        for(pos = 1, i = 2; i < argc && pos < sizeof(buffer); i++){
            pos += hexread(buffer + pos, argv[i], sizeof(buffer) - pos);
        }
#endif

        if((err = usbhidSetReport(dev, buffer, sizeof(buffer))) != 0)   /* add a dummy report ID */
            fprintf(stderr, "error writing data: %s\n", usbErrorMessage(err));

    }else{
        usage(argv[0]);
        exit(1);
    }
    usbhidCloseDevice(dev);
    return 0;
}

/* ------------------------------------------------------------------------- */
