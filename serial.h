/* Serial port definitions and public interface */

/* Copyright (C) 2009-2013 David Zanetti
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

#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

/* initalise serial port X */
/** \brief Initalise the first serial port
 */
uint8_t serial0_init(void);
/** \brief Iniitalise the second serial port
 */
uint8_t serial1_init(void);

/* flush the serial port, clearing any buffered packets */
void serial0_flush(void);
void serial1_flush(void);

/* single character send */
uint8_t serial0_tx_cout(char s);
uint8_t serial1_tx_cout(const char s);

/* send a string from SRAM */
uint8_t serial0_tx(const char *str, uint8_t len);
uint8_t serial1_tx(const char *str, uint8_t len);

/* send a string from PGM space */
uint8_t serial0_tx_PGM(const char *str);
uint8_t serial1_tx_PGM(const char *str);

/* send a simple number has hex */
uint8_t serial0_tx_hex(uint8_t s);
uint8_t serial1_tx_hex(uint8_t s);

/* decimal */
uint8_t serial0_tx_dec(uint32_t s);
uint8_t serial1_tx_dec(uint32_t s);

/* CRs */
uint8_t serial0_tx_cr(void);
uint8_t serial1_tx_cr(void);

/* Receive a character */
uint8_t serial0_rx(void);
uint8_t serial1_rx(void);

void serial0_echo(uint8_t mode);

#endif // SERIAL_H_INCLUDED
