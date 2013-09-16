/* Copyright (C) 2013 David Zanetti
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* XMEGA ADC interface */

#include <avr/io.h>
#include "global.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stddef.h>
#include "errors.h"
#include "serial.h"
#include "adc.h"

/** \file
 *  \brief XMEGA ADC implemenation
 */

#define MAX_CHANS 1

/** \brief Retrieve calibration words from production row
 *  \param offset offset into signatures
 */
uint16_t read_cal_word(uint8_t offset);

uint8_t adc_init(adc_mode_t mode, adc_vref_t vref, uint8_t bits,
	adc_clkpre_t clkpre) {

	/* power up the ADC */
	PR.PRPA &= ~PR_ADC_bm;

	/* set the conversion mode as described */
	switch (mode) {
		case adc_unsigned:
			ADCA.CTRLB = 0;
			break;
		case adc_signed:
			ADCA.CTRLB = ADC_CONMODE_bm;
			break;
		default:
			return -EINVAL;
	}

	/* resolution as requested */
	switch (bits) {
		case 8:
			ADCA.CTRLB |= ADC_RESOLUTION_8BIT_gc;
			break;
		case 12:
			ADCA.CTRLB |= ADC_RESOLUTION_12BIT_gc;
			break;
		default:
			return -EINVAL;
	}

	/* reference control */
	switch (vref) {
		case adc_vref_int1v:
			ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
			break;
		case adc_vref_intvcc16:
			ADCA.REFCTRL = ADC_REFSEL_VCC_gc; /* really VCC/1.6 */
			break;
		case adc_vref_arefa:
			ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;
			break;
		case adc_vref_arefb:
			ADCA.REFCTRL = ADC_REFSEL_AREFB_gc;
			break;
		default:
			return -EINVAL;
	}

	/* prescaler */
	/* as it happens, these are the same values as the headers, so just
	 * apply it :D */
	/* note: AVR1300 states the max ADC clock is 2MHz for an A-series chip,
	   and 1.4MHz for a D-series chip. Take note! */
	if (clkpre > 7) {
		return -EINVAL;
	}
	ADCA.PRESCALER = clkpre;

	/* apply production row calibration */
	ADCA.CAL = read_cal_word(PROD_SIGNATURES_START+offsetof(NVM_PROD_SIGNATURES_t,ADCACAL0));

#ifdef DEBUG_ADC
	printf_P(PSTR("adc: calibration value %#x\r\n"),ADCA.CAL);
#endif // DEBUG_ADC

	/* now enable the module, and we're done */
	ADCA.CTRLA |= ADC_ENABLE_bm;

	return 0;
}

/* read the referenced production signature row */
uint16_t read_cal_word(uint8_t offset) {
	uint16_t n;
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	n = pgm_read_word(offset);
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	return n;
}

/* set up conversion paramters for the given channel */
uint8_t adc_conf(uint8_t chan, adc_input_t input, uint8_t muxpos,
	uint8_t muxneg) {

	/* check if we have a sensible channel number */
	if (chan >= MAX_CHANS) {
		return -ENODEV;
	}

	switch (chan) {
		case 0:
			ADCA.CH0.CTRL = input; /* input is at the botom */
			ADCA.CH0.MUXCTRL = (muxpos << 3) | (muxneg);
			break;
	}
	return 0;
}

/* blocking ADC conversion. Note: testing only, with interrupts enabled this
 * blocking code doesn't work
 */
uint16_t adc_conv_blocking(uint8_t chan) {
	/* start the conversion, then poll interrupt bit until it's done */
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
	while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm));
	/* conversion is done, return the result */
	return ADCA.CH0.RES;
}
