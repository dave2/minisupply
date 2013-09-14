/* Serial driver */

/* Copyright (C) 2009-2012 David Zanetti
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>

#include "serial.h"

#include "global.h"
#include <util/delay.h>

#include "ringbuffer.h"

/* port features */
#define PORT_NONE 0x0 /**< Serial port has no features */
#define PORT_ECHO 0x1 /**< Serial port will echo-back inside driver */

/* hw access, buffers, and metadata about a serial port */

/** \struct serial_port_t
 *  \brief  Contains the abstraction of a hardware port
 */
typedef struct {
		USART_t *hw;            /**< USART IO registers */
		ringbuffer_t *txring;   /**< TX ringbuffer */
		ringbuffer_t *rxring;   /**< RX ringbuffer */
		uint8_t isr_level;     /**< Level to run/restore interrupts at */
		uint8_t rxflag;        /**< Flag to apply when this port has an RX event */
		uint8_t cmdflag;       /**< Flag to apply when we see a \n in the stream */
		uint8_t features;      /**< Capabilities of the port, see PORT_* macros */
} serial_port_t;

/* FIXME: ideally we should instead have an array of these which map to hardware ports */

serial_port_t *serial0;         /**< First serial port abstraction */
serial_port_t *serial1;         /**< Second serial port abstraction */

#define SERIAL0_RXBUF 128       /**< Serial0 RX ringbuffer size */
#define SERIAL0_TXBUF 256       /**< Serial0 TX ringbuffer size */
#define SERIAL1_RXBUF 256       /**< Serial1 RX ringbuffer size */
#define SERIAL1_TXBUF 128       /**< Serial1 TX ringbuffer size */

/* private function prototypes */

/* Low-level ISR handlers, generic on which port they apply to */

/** \brief Handle a TX interrupt for the given port
 * \param port Port abstraction this event applies to
 *
 */
void _usart_tx_isr(serial_port_t *port);

/** \brief Handle an RX interrupt for the given port
 *  \param port Port abstraction this event applies to
 */
void _usart_rx_isr(serial_port_t *port);

/** \brief Start TX processing on the given port
 *  \param port Port abstraction this event applies to
 */
void _usart_tx_run(serial_port_t *port);

/* high-level API handling */

/** \brief Send single character to port
 *  \param port Serial port to use
 *  \param s The character
 */
uint8_t _tx_cout(serial_port_t *port, char s);

/** \brief Send a string to port
 *  \param port Serial port to use
 *  \param str The string (does not need to be NUL terminated)
 *  \param len Length of the string
 */
uint8_t _tx(serial_port_t *port, const char *str, uint8_t len);

/** \brief Send a string from flash space to port
 *  \param port Serial port to use
 *  \param str String in flash space (must be NUL terminated)
 */
uint8_t _tx_PGM(serial_port_t *port, const char *str);

/** \brief Send a number reformated as hexadecimal to port
 *  \param port Serial port to use
 *  \param c 8-bit number to send
 */
uint8_t _tx_hex(serial_port_t *port, uint8_t c);

/** \brief Send a number reformatted as integer to port
 *  \param port Serial port to use
 *  \param s 32-bit int to send
 */
uint8_t _tx_dec(serial_port_t *port, uint32_t s);

/** \brief Send a newline to port
 *  \param port Serial port to use
 */
uint8_t _tx_cr(serial_port_t *port);

/** \brief Flush the given serial port buffer
 *  \param port Serial port to flush
 */
void _flush(serial_port_t *port);

/* Interrupt hooks and handlers */

/* handle a TX event */
/* since this fires on empty, it's safe to fire more than we actually need to */
void _usart_tx_isr(serial_port_t *port) {
	if (!port) {
		return; /* don't try to use uninitalised ports */
	}
	/* check to see if we have anything to send */
	if (!ring_readable_unsafe(port->txring)) {
		/* disable the interrupt and then exit, nothing more to do */
		port->hw->CTRLA = port->hw->CTRLA & ~(USART_DREINTLVL_gm);
		return;
	}
	/* TX the waiting packet */
	port->hw->DATA = ring_read_unsafe(port->txring);
}

/* handle an RX event */
void _usart_rx_isr(serial_port_t *port) {
	char s;

	if (!port) {
		return; /* don't try to use uninitalised ports */
	}

	s = port->hw->DATA; /* read the char from the port */
	ring_write_unsafe(port->rxring, s); /* if this fails we have nothing useful we can do anyway */
	if (s == '\r' && port->cmdflag) {
		flag_serial |= port->cmdflag;
	}
	flag_serial |= port->rxflag; /* indicate to main loops we had an RX for this port */
	if (port->features & PORT_ECHO) {
		/* fixme: does this introduce another source of ring corruption? */
		ring_write_unsafe(port->txring,s);
		_usart_tx_run(port);
	}
}

/* make the given port start TXing */
void _usart_tx_run(serial_port_t *port) {
	/* enable interrupts for the appropriate port */
	port->hw->CTRLA = port->hw->CTRLA | (port->isr_level & USART_DREINTLVL_gm);
	return;
}

/* we map the hardware port interrupts to the appropriate structure here */

/* interrupt handler for packets on TX USARTF0 */
ISR(USARTE0_DRE_vect) {
	_usart_tx_isr(serial0); /* USARTF0 is serial0 in our code */
	return;
}

/* interrupt handler for packets on RX USARTF0 */
ISR(USARTE0_RXC_vect) {
	_usart_rx_isr(serial0); /* USARTF0 is serial0 in our code */
	return;
}

/* interrupt handler for packets on TX USARTC0 */
ISR(USARTD0_DRE_vect) {
	_usart_tx_isr(serial1);
	return;
}

/* interrupt handler for packets on RX USARTC0 */
ISR(USARTD0_RXC_vect) {
	_usart_rx_isr(serial1);
	return;
}

/* initalise the structures and hardware */

uint8_t serial0_init(void) {
	//uint16_t bsel = 3317;
	//uint8_t bscale = -4;
	uint16_t bsel = 1047; // 115200
	uint8_t bscale = -6;

	/* power up serial port */
	PR.PRPE &= ~(PR_USART0_bm);

	/* create the metadata for the port */
	serial0 = malloc(sizeof(serial_port_t));
	if (!serial0) {
		return 0;
	}

	/* create two ringbuffers, one for TX and one for RX */

	serial0->rxring = ring_create(SERIAL0_RXBUF-1);
	if (!serial0->rxring) {
		free(serial0);
		serial0 = NULL;
		return 0; /* FIXME: flag serial IO no longer works */
	}

	serial0->txring = ring_create(SERIAL0_TXBUF-1);
	if (!serial0->txring) {
		ring_destroy(serial0->rxring); /* since the first one succeeded */
		free(serial0);
		serial0 = NULL;
		return 0; /* FIXME: flag serial IO no longer works */
	}

	/* other metadata for the port */

	serial0->hw = &USARTE0; /* hard wired to this port */
	serial0->isr_level = USART_DREINTLVL_MED_gc | USART_RXCINTLVL_MED_gc; /* low prio interrupt */
	serial0->rxflag = FLAG_SERIAL_S0_RX;
	serial0->cmdflag = FLAG_SERIAL_S0_CMD;
	serial0->features = PORT_ECHO;

	/* set the baud rate. Sadly, this is fixed at 32MHz F_CPU and based off a calculator */
	/* FIXME: compute baud rate settings properly, or at least a table of good ones */

	/* set pins appropriate direction (3 is TX->OUT, 2 is RX->IN, for USARTF0) */
	PORTE.DIRSET = PIN3_bm;
	PORTE.DIRCLR = PIN2_bm;

	/* 8-bit, no parity, 1 stop bit */
	serial0->hw->CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	/* set port baud rate */
	serial0->hw->BAUDCTRLA = (uint8_t) bsel;
	serial0->hw->BAUDCTRLB = (bscale << 4) | (bsel >> 8);

	/* enable TX, RX, RX Interrupt */
	serial0->hw->CTRLA = (serial0->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (serial0->isr_level & USART_RXCINTLVL_gm);
	serial0->hw->CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);

	/* make sure low-level interrupts are enabled. Note: you still need to enable global interrupts */
	PMIC.CTRL |= PMIC_MEDLVLEX_bm;

	return 1;
}

uint8_t serial1_init(void) {
	uint16_t bsel = 3269; // 38400 @ 32MHz
	uint8_t bscale = -6;

	//uint16_t bsel = 3317; // 9600 @ 32MHz
	//uint8_t bscale = -4;

	/* power up serial port */
	PR.PRPD &= ~(PR_USART0_bm);

	/* create the metadata for the port */
	serial1 = malloc(sizeof(serial_port_t));
	if (!serial1) {
		return 0;
	}

	/* create two ringbuffers, one for TX and one for RX */

	serial1->rxring = ring_create(SERIAL1_RXBUF-1);
	if (!serial1->rxring) {
		free(serial1);
		serial1 = NULL;
		return 0; /* FIXME: flag serial IO no longer works */
	}

	serial1->txring = ring_create(SERIAL1_TXBUF-1);
	if (!serial1->txring) {
		ring_destroy(serial1->rxring); /* since the first one succeeded */
		free(serial1);
		serial1 = NULL;
		return 0; /* FIXME: flag serial IO no longer works */
	}

	/* other metadata for the port */

	serial1->hw = &USARTD0; /* hard wired to this port */
	serial1->isr_level = USART_DREINTLVL_LO_gc | USART_RXCINTLVL_LO_gc; /* low prio interrupt */
	serial1->rxflag = FLAG_SERIAL_S1_RX;
	serial1->cmdflag = 0;
	serial1->features = PORT_NONE;

	/* set the baud rate. Sadly, this is fixed at 32MHz F_CPU and based off a calculator */
	/* FIXME: compute baud rate settings properly, or at least a table of good ones */

	/* set pins appropriate direction (3 is TX->OUT, 2 is RX->IN, for USARTD0) */
	PORTD.DIRSET = PIN3_bm;
	PORTD.DIRCLR = PIN2_bm;

	/* 8-bit, no parity, 1 stop bit */
	serial1->hw->CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	/* set port baud rate */
	serial1->hw->BAUDCTRLA = (uint8_t) bsel;
	serial1->hw->BAUDCTRLB = (bscale << 4) | (bsel >> 8);

	/* enable interrupts, then TX, RX */
	serial1->hw->CTRLA = (serial1->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (serial1->isr_level & USART_RXCINTLVL_gm);
	serial1->hw->CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);

	/* make sure low-level interrupts are enabled. Note: you still need to enable global interrupts */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	return 1;
}

void serial0_flush(void) {
	_flush(serial0);
}

void serial1_flush(void) {
	_flush(serial1);
}

void _flush(serial_port_t *port) {
	/* protect this from interrupts */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_RXCINTLVL_gm | USART_DREINTLVL_gm));

	ring_reset(port->txring);
	ring_reset(port->rxring);
	if (port->cmdflag) {
		clear_flag(flag_serial,port->cmdflag);
	}
	if (port->rxflag) {
		clear_flag(flag_serial,port->rxflag);
	}

	/* re-enable RX interrupts */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (port->isr_level & USART_RXCINTLVL_gm);
	/* for TX, since we just wiped the ring buffer, it has nothing to TX, so don't enable DRE */

	return;
}

/* output a single char to a port */

uint8_t _tx_cout(serial_port_t *port, char s) {

	if (!port) { /* protect against uninitialised ports */
		return 0 ;
	}

	/* disable interrupt while we write */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_DREINTLVL_gm));

	ring_write(port->txring,s);

	/* this re-enables DRE */
	_usart_tx_run(port);
	return 1;
}

uint8_t serial0_tx_cout(char s) {
	return _tx_cout(serial0,s);
}

uint8_t serial1_tx_cout(char s) {
	return _tx_cout(serial1,s);
}

/* output a string of chars to a port (string is in RAM) */

uint8_t _tx(serial_port_t *port, const char *str, uint8_t len) {
	if (!port) { /* safety checking on port state */
		return 0;
	}

	/* disable interrupt while we write */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_DREINTLVL_gm));

	/* loop over each char in the buffer */
	while (len) {
		/* write char */
		ring_write(port->txring, *str++);
		/* move on */
		len--;
	}

	_usart_tx_run(port);

	return 1;
}

uint8_t serial0_tx(const char *str, uint8_t len) {
	return _tx(serial0, str, len);
}

uint8_t serial1_tx(const char *str, uint8_t len) {
	return _tx(serial1, str, len);
}

/* output a string of chars to a port (string is in FLASH) */

uint8_t _tx_PGM(serial_port_t *port, const char *str) {
	uint8_t n = 0;
	char c;

	/* disable interrupt while we write */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_DREINTLVL_gm));

	/* loop over each char in the buffer */
	while ((c = pgm_read_byte(&str[n]))) {
		/* write flash-read char into buffer */
		ring_write(port->txring,c);
		n++;
	}

	_usart_tx_run(port);

	return 1;
}

uint8_t serial0_tx_PGM(const char *str) {
	return _tx_PGM(serial0,str);
}

uint8_t serial1_tx_PGM(const char *str) {
	return _tx_PGM(serial1,str);
}

/* read a single char from a port */

uint8_t _rx(serial_port_t *port) {
	char c;

	if (!port) { /* safety check on the port */
		return 0; /* er, yeah, well.. *shrug* */
	}

	/* protect this from interrupts also writing into the buffer */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_RXCINTLVL_gm)); /* disable RX interrupt */

	/* grab next char from ring buffer */
	c = ring_read(port->rxring);

	/* if this is the last available char, clear the run flag */
	if (!ring_readable(port->rxring)) {
		clear_flag(flag_serial,port->rxflag);
	}

	/* re-enable RX interrupt */
	port->hw->CTRLA = (port->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (port->isr_level & USART_RXCINTLVL_gm);

	/* return read character */
	return c;
}

uint8_t serial0_rx(void) {
	return _rx(serial0);
}

uint8_t serial1_rx(void) {
	return _rx(serial1);
}

/* output a converted single-byte value as a hex string */

uint8_t _tx_hex(serial_port_t *port, uint8_t c) {
	char hex[16] = "0123456789abcdef";

	if (!port) { /* port init safety check */
		return 0;
	}

	_tx_cout(port,hex[c >> 4]);
	_tx_cout(port,hex[c & 0xf]);

	return 0;
}

uint8_t serial0_tx_hex(uint8_t c) {
	return _tx_hex(serial0,c);
}

uint8_t serial1_tx_hex(uint8_t c) {
	return _tx_hex(serial1,c);
}

/* output a converted 32-bit unsigned int as a decimal string */

uint8_t _tx_dec(serial_port_t *port, uint32_t s) {
	char a[11] = ""; /* 4294967296 */
	uint8_t n = 0;

	if (!port) { /* init safety check */
		return 0;
	}

	ultoa(s, a, 10);

	while (a[n]) {
		_tx_cout(port,a[n]);
		n++;
	}

	return 0;

}

uint8_t serial0_tx_dec(uint32_t s) {
	return _tx_dec(serial0,s);
}

uint8_t serial1_tx_dec(uint32_t s) {
	return _tx_dec(serial1,s);
}

/* output a "normal" CR, which is actually CR-LF */

uint8_t _tx_cr(serial_port_t *port) {
	if (!port) { /* init check */
		return 0;
	}
	return _tx_PGM(port,PSTR("\r\n"));
}

uint8_t serial0_tx_cr(void) {
	return _tx_cr(serial0);
}

uint8_t serial1_cr(void) {
	return _tx_cr(serial1);
}

void serial0_echo(uint8_t mode) {
	if (mode) {
		serial0->features |= PORT_ECHO;
	} else {
		serial0->features &= ~(PORT_ECHO);
	}
	return;
}
