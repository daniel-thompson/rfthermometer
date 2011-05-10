/*
 * main.c
 *
 * Main functions for RFThermometer, a firmware for a USB thermometer based
 * on vusb.
 *
 * Copyright (C) 2011 Daniel Thompson <daniel@redfelineninja.org.uk> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usbdrv.h"
#include "oddebug.h"

#include "osccal.h"

/* ------------------------------------------------------------------------- */

#define lengthof(x) (sizeof(x) / sizeof(x[0]))

#define BUTTON_PORT PORTB       /* PORTx - register for button output */
#define BUTTON_PIN PINB         /* PINx - register for button input */
#define BUTTON_BIT PB0          /* bit for button input/output */

#define LED_BIT PB1
#define LED_ON()   (PORTB &= ~_BV(LED_BIT))
#define LED_OFF()  (PORTB |= _BV(LED_BIT))
#define LED_TOGGLE() (PORTB ^= _BV(LED_BIT))
#define LED_INIT() (LED_OFF(), DDRB |= _BV(LED_BIT))

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

/* ------------------------------------------------------------------------- */

static volatile unsigned int latchedConversion[4];

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;

/* ------------------------------------------------------------------------- */

static uchar usbGetSensorReport(uchar offset) {
	if (offset == 1)
		return OSCCAL;

	if (offset >= 2 && offset < 10)
		return ((uchar *) (&latchedConversion))[offset -2 ];

	return eeprom_read_byte((uchar *)0 + offset);
}


/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionRead(uchar *data, uchar len)
{
    uchar i;
    LED_TOGGLE();

    if(len > bytesRemaining)
        len = bytesRemaining;

    for (i=0; i<len; i++, currentAddress++)
	    data[i] = usbGetSensorReport(currentAddress);
    bytesRemaining -= len;
	 
    return len;
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
	LED_TOGGLE();
    if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_write_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}

void hadUsbReset(void)
{
    const uchar threshold = 2;

    uchar oldcal, newcal;

    cli();
    calibrateOscillator();
    sei();

    oldcal = eeprom_read_byte(0);
    newcal = OSCCAL;

    /* store the calibrated value in EEPROM if it has changed by more
     * than the threshold
     */    
    if (((oldcal > newcal) && ((oldcal - newcal) > threshold)) ||
	((oldcal < newcal) && ((newcal - oldcal) > threshold)))
        eeprom_write_byte(0, OSCCAL);
}

/* ------------------------------------------------------------------------- */

#define ADC_START_CONVERSION() (ADCSRA |= _BV(ADSC))
#define ADC_IS_BUSY()  (ADCSRA & _BV(ADSC))

static void adcInit(void)
{
	/* enable ADC, not free running, interrupt disable, rate = 1/128 */	
	ADCSRA = UTIL_BIN8(1000, 0111);

	/* must match muxval[0] (from adcPoll) */
	ADMUX = UTIL_BIN8(1000, 1111);
	
	/* kick off the first conversion */
	ADC_START_CONVERSION();
}

static void adcPoll(void)
{
	static uchar state = 0;
	static uchar discard = 1;

	static const uchar muxval[] = {
		UTIL_BIN8(1000, 1111), /* Vref=1.1V, ADC4 (temp sensor) */
		UTIL_BIN8(1000, 0001), /* Vref=1.1V, ADC1 (PB2) */
		UTIL_BIN8(1001, 0001), /* Vref=2.56V, ADC1 (PB2) */
		UTIL_BIN8(0000, 0001), /* Vref=3.3V, ADC1 (PB2) */
	};
	

	if (!ADC_IS_BUSY()) {
		if (discard) {
			/* discard this result and start again */
			ADC_START_CONVERSION();
			discard = 0;
		} else {
			/* conversion is complete */
			/*cli();*/
			if (!bytesRemaining)
				latchedConversion[state] = ADC;
			/*sei();*/
			
			/* whether we did the write or not if the latched value
			 * matches we're done (note also that we've moved most
			 * of the work outside the interrupt lock)
			 */
			if (latchedConversion[state] == ADC) {
				/* switch to next state */
				state++;
				if (state >= lengthof(muxval))
					state = 0;
				
				/* reprogram the ADC (making sure we discard
				 * the next value)
				 */
				ADMUX = muxval[state];
				ADC_START_CONVERSION();
				discard = 1;
			}
		}
	}
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;
uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    
    usbInit();
    adcInit();

    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_1S);

    LED_INIT();
	/* turn on internal pull-up resistor for the switch */
    BUTTON_PORT |= _BV(BUTTON_BIT);

    sei();

    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
	adcPoll();
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
