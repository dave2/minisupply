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

/* internal flags */
#define RX_READY 1

/* hw access, buffers, and metadata about a serial port */

/** \struct serial_port_t
 *  \brief  Contains the abstraction of a hardware port
 */
typedef struct {
    USART_t *hw; /**< USART hardware IO registers */
	ringbuffer_t *txring; /**< TX ringbuffer */
	ringbuffer_t *rxring; /**< RX ringbuffer */
	register8_t *pr; /**< power reduction register to use */
	uint8_t pr_num; /**< what to fiddle in PR register */
	uint8_t isr_level; /**< Level to run/restore interrupts at */
	uint8_t flags; /**< Current state of flags */
	uint8_t features; /**< Capabilities of the port, see S_FEAT_ */
	void (*rx_fn)(uint8_t); /**< Callback function for RX */
} serial_port_t;

#define MAX_PORTS 2 /**< Maximum number of serial ports supported */

serial_port_t *ports[MAX_PORTS] = {0,0}; /**< Serial port abstractions */

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
	port->flags |= RX_READY; /* indicates to available we have RX */
	if (port->features & S_FEAT_ECHO) {
		/* fixme: does this introduce another source of ring corruption? */
		ring_write_unsafe(port->txring,s);
		_usart_tx_run(port);
	}
	/* invoke the callback if one exists */
	if (port->rx_fn) {
        (*port->rx_fn)(s);
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
	_usart_tx_isr(ports[0]); /* USARTE0 is serial0 in our code */
	return;
}

/* interrupt handler for packets on RX USARTF0 */
ISR(USARTE0_RXC_vect) {
	_usart_rx_isr(ports[0]); /* USARTE0 is serial0 in our code */
	return;
}

/* interrupt handler for packets on TX USARTD0 */
ISR(USARTD0_DRE_vect) {
	_usart_tx_isr(ports[1]);
	return;
}

/* interrupt handler for packets on RX USARTD0 */
ISR(USARTD0_RXC_vect) {
	_usart_rx_isr(ports[1]);
	return;
}

/* initalise the structures and hardware */
uint8_t serial_init(uint8_t portnum, uint8_t rx_size, uint8_t tx_size) {

    if (ports[portnum] || portnum >= MAX_PORTS) {
        /* refuse to re-initalise a port or one not allocatable */
        return 0;
    }

	/* create the metadata for the port */
	ports[portnum] = malloc(sizeof(serial_port_t));
	if (!ports[portnum]) {
		return 0;
	}

	/* create two ringbuffers, one for TX and one for RX */

	ports[portnum]->rxring = ring_create(rx_size-1);
    if (!ports[portnum]->rxring) {
		free(ports[portnum]);
		ports[portnum] = NULL;
		return 0; /* FIXME: flag serial IO no longer works */
	}

	ports[portnum]->txring = ring_create(tx_size-1);
	if (!ports[portnum]->txring) {
		ring_destroy(ports[portnum]->rxring); /* since the first one succeeded */
		free(ports[portnum]);
		ports[portnum] = NULL;
		return 0; /* FIXME: flag serial IO no longer works */
	}

	/* connect the hardware */
    switch (portnum) {
        case 0:
            //ports[portnum]->pr = &PR_PRPE;
            //ports[portnum]->pr_num = PR_USART0_bm;
            PR.PRPE &= ~(PR_USART0_bm);
            //*(ports[portnum]->pr) &= ~(ports[portnum]->pr_num);
            PORTE.DIRSET = PIN3_bm;
            PORTE.DIRCLR = PIN2_bm;
            ports[portnum]->hw = &USARTE0;
            break;
        case 1:
            ports[portnum]->pr = &PR_PRPD;
            ports[portnum]->pr_num = PR_USART0_bm;
            PORTD.DIRSET = PIN3_bm;
            PORTD.DIRCLR = PIN2_bm;
            ports[portnum]->hw = &USARTD0;
            break;
    }

    /* fixme: allow seperate interrupt levels for different ports */
	ports[portnum]->isr_level = USART_DREINTLVL_LO_gc | USART_RXCINTLVL_LO_gc; /* low prio interrupt */

	/* enable rx interrupts */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (ports[portnum]->isr_level & USART_RXCINTLVL_gm);
	ports[portnum]->hw->CTRLB |= USART_CLK2X_bm;

	/* make sure low-level interrupts are enabled. Note: you still need to enable global interrupts */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	return 1;
}

uint8_t serial_mode(uint8_t portnum, uint32_t baud, uint8_t bits,
                    uint8_t stop, uint8_t parity, uint8_t features) {
    uint8_t mode = 0;
    uint32_t div1k;
    uint8_t bscale = 0;
    uint16_t bsel;

    if (portnum > MAX_PORTS || !ports[portnum]) {
        return 0;
    }
    switch (bits) {
        case 5:
        case 6:
        case 7:
        case 8:
            mode = (bits - 5); /* cheating! */
            break;
        case 9:
            /* technically unsupported but here is the code to set it */
            mode = 7; /* cheating! */
            break;
        default:
            return 0;
    }
    switch (stop) {
        case 0:
        case 1:
            mode |= (stop & 1) << 3; /* more cheating */
            break;
        default:
            return 0;
    }
    switch (parity) {
        case S_PARITY_NONE:
            break;
        case S_PARITY_EVEN:
            mode |= (2 << 4);
            break;
        case S_PARITY_ODD:
            mode |= (3 << 4);
            break;
        default:
            return 0;
    }
    /* apply the cheating way we worked out the modes for the port */
    ports[portnum]->hw->CTRLC = mode;

    /* apply features */
    ports[portnum]->features = features;

    /* default callback is NULL */
    ports[portnum]->rx_fn = NULL;

    /* the following code comes from:
     * http://blog.omegacs.net/2010/08/18/xmega-fractional-baud-rate-source-code/
     */
    if (baud > (F_CPU/16)) {
        return 0;
    }

    div1k = ((F_CPU*128) / baud) - 1024;
    while ((div1k < 2096640) && (bscale < 7)) {
        bscale++;
        div1k <<= 1;
    }

    bsel = div1k >> 10;

    ports[portnum]->hw->BAUDCTRLA = bsel&0xff;
    ports[portnum]->hw->BAUDCTRLB = (bsel>>8) | ((16-bscale) << 4); // was 16-bscale

    /* end nicely borrowed code! */

    /* all good! */
    return 1;
}

void serial_run(uint8_t portnum, uint8_t state) {
    if (portnum > MAX_PORTS || !ports[portnum]) {
        /* do nothing */
        return;
    }
    switch (state) {
        case 0:
            /* disable the TX/RX sides, so we come out disabled */
            ports[portnum]->hw->CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
            /* don't worry about the interrupts, since we have disabled tx/rx */
            /* shutdown the HW */
            //*(ports[portnum]->pr) |= ports[portnum]->pr_num;
            break;
        case 1:
            /* re-enable the hardware */
            //*(ports[portnum]->pr) &= ~(ports[portnum]->pr_num);
            /* now re-enable the modules */
            ports[portnum]->hw->CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);
            break;
    }
    return;
}

/* install a new callback */
uint8_t serial_rx_hook(uint8_t portnum, void (*fn)(uint8_t)) {
    if (portnum > MAX_PORTS || !ports[portnum]) {
        return 0;
    }
    ports[portnum]->rx_fn = fn;
    return 1;
}

void serial_flush(uint8_t portnum) {

    if (portnum > MAX_PORTS || !ports[portnum]) {
        return;
    }

    /* disable the TX/RX engines */
    ports[portnum]->hw->CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);

	/* protect this from interrupts */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_RXCINTLVL_gm | USART_DREINTLVL_gm));

	ring_reset(ports[portnum]->txring);
	ring_reset(ports[portnum]->rxring);
	ports[portnum]->flags = 0;

	/* re-enable RX interrupts */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (ports[portnum]->isr_level & USART_RXCINTLVL_gm);
	/* for TX, since we just wiped the ring buffer, it has nothing to TX, so don't enable DRE */

    /* re-enable the actual ports */
    ports[portnum]->hw->CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);

	return;
}

/* output a single char to a port */

uint8_t serial_tx_cout(uint8_t portnum, char s) {

	if (portnum > MAX_PORTS || !ports[portnum]) {
		return 0 ;
	}

	/* disable interrupt while we write */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_DREINTLVL_gm));

	ring_write(ports[portnum]->txring,s);

	/* this re-enables DRE */
	_usart_tx_run(ports[portnum]);
	return 1;
}

/* output a string of chars to a port (string is in RAM) */

uint8_t serial_tx(uint8_t portnum, const char *str, uint8_t len) {
	if (portnum > MAX_PORTS || !ports[portnum]) { /* safety checking on port state */
		return 0;
	}

	/* disable interrupt while we write */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_DREINTLVL_gm));

	/* loop over each char in the buffer */
	while (len) {
		/* write char */
		ring_write(ports[portnum]->txring, *str++);
		/* move on */
		len--;
	}

	_usart_tx_run(ports[portnum]);

	return 1;
}

/* output a string of chars to a port (string is in FLASH) */

uint8_t serial_tx_PGM(uint8_t portnum, const char *str) {
	uint8_t n = 0;
	char c;

	/* disable interrupt while we write */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_DREINTLVL_gm));

	/* loop over each char in the buffer */
	while ((c = pgm_read_byte(&str[n]))) {
		/* write flash-read char into buffer */
		ring_write(ports[portnum]->txring,c);
		n++;
	}

	_usart_tx_run(ports[portnum]);

	return 1;
}


/* read a single char from a port */

uint8_t serial_rx(uint8_t portnum) {
	char c;

	if (portnum > MAX_PORTS || !ports[portnum]) { /* safety check on the port */
		return 0; /* er, yeah, well.. *shrug* */
	}

	/* protect this from interrupts also writing into the buffer */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_RXCINTLVL_gm)); /* disable RX interrupt */

	/* grab next char from ring buffer */
	c = ring_read(ports[portnum]->rxring);

	/* if this is the last available char, clear the run flag */
	if (!ring_readable(ports[portnum]->rxring)) {
        ports[portnum]->flags &= RX_READY;
	}

	/* re-enable RX interrupt */
	ports[portnum]->hw->CTRLA = (ports[portnum]->hw->CTRLA & ~(USART_RXCINTLVL_gm)) | (ports[portnum]->isr_level & USART_RXCINTLVL_gm);

	/* return read character */
	return c;
}

/* output a converted single-byte value as a hex string */

uint8_t serial_tx_hex(uint8_t portnum, uint8_t c) {
	char hex[16] = "0123456789abcdef";

    /* handle the port check inside the write code */
	serial_tx_cout(portnum,hex[c >> 4]);
	serial_tx_cout(portnum,hex[c & 0xf]);

	return 0;
}

/* output a converted 32-bit unsigned int as a decimal string */

uint8_t serial_tx_dec(uint8_t portnum, uint32_t s) {
	char a[11] = ""; /* 4294967296 */
	uint8_t n = 0;

	if (portnum > MAX_PORTS || !ports[portnum]) { /* init safety check */
		return 0;
	}

	ultoa(s, a, 10);

    /* fixme, output this using string function instead */
	while (a[n]) {
		serial_tx_cout(portnum,a[n]);
		n++;
	}

	return 0;

}


/* output a "normal" CR, which is actually CR-LF */

uint8_t serial_tx_cr(uint8_t portnum) {
	return serial_tx_PGM(portnum,PSTR("\r\n"));
}

uint8_t serial_rx_available(uint8_t portnum) {
    if (portnum > MAX_PORTS || !ports[portnum]) {
        return 0;
    }
    return (ports[portnum]->flags & RX_READY);
}
