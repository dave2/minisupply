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

/** \file
 *  \brief USART driver public API
 *
 *  Interrupt-driven USART driver with runtime baud rate calculation,
 *  flexible TX/RX buffering, and RX interrupt call hooks to faciliate
 *  integration in main loop processing.
 *
 *  Can be easily expanded to cover more ports than the 2 currently
 *  implemented, as all code is generic
 */

#define S_FEAT_NONE 0 /**< Serial port feature: None */
#define S_FEAT_ECHO 1 /**< Serial port feature: echoback inside driver */

#define S_PARITY_NONE 0 /**< No parity */
#define S_PARITY_EVEN 1 /**< Even parity */
#define S_PARITY_ODD 2 /**< Odd Parity */

/* initalise serial port X */
/** \brief Initalise the given serial port with the sized buffers
 *  \param portnum Number of the port
 *  \param rx_size Size of the RX buffer (must be power of two)
 *  \param tx_size Size of the TX buffer (must be power of two)
 *  \return 1 for success, 0 otherwise
 */
uint8_t serial_init(uint8_t portnum, uint8_t rx_size, uint8_t tx_size);

/** \brief Set parameters for the port, speed and such like
 *  \param portnum Number of the port
 *  \param baud Baudrate
 *  \param bits Bits per char (note: 9 is not supported)
 *  \param stop Stop bits
 *  \param parity Parity mode (see S_PARITY_*)
 *  \param features Features (see S_FEAT_*)
 *  \return 1 for success, 0 otherwise
 */
uint8_t serial_mode(uint8_t portnum, uint32_t baud, uint8_t bits,
                    uint8_t stop, uint8_t parity, uint8_t features);

/** \brief Provide an RX hook to the interrupt for a given port
 *
 *  The hook function will execute inside the interrupt for the RX, try
 *  not to do any heavy lifting. Provide NULL as the function hook to
 *  clear it.
 *
 *  Note: this MUST be called when the port is suspended or bad things
 *  may happen.
 *  \param portnum Number of the port
 *  \param fn Function to execute. It must return void and accept
 *         a single uint8_t argument containing most recent char.
 *  \return 1 if hook loaded successfully, 0 if not
 */
uint8_t serial_rx_hook(uint8_t portnum, void (*fn)(uint8_t));

/** \brief Start listening for events and characters, also allows
 *  TX to begin
 *  \param portnum Number of the port
 *  \param state 1 = run, 0 = suspend
 */
void serial_run(uint8_t portnum, uint8_t state);

/** \brief Flush the buffers for the serial port
 *  \param portnum Number of the port
 */
void serial_flush(uint8_t portnum);

/** \brief Send a single character to a serial port
 *  \param portnum Number of the port
 *  \param s Character to send
 */
uint8_t serial_tx_cout(uint8_t portnum, char s);

/** \brief Send a string to a serial port
 *  \param portnum Number of the port
 *  \param str String to send (does not need NUL termination)
 *  \param len Length of the string
 */
uint8_t serial_tx(uint8_t portnum, const char *str, uint8_t len);

/** \brief Send a string to a serial port from flash space
 *  \param portnum Number of the port
 *  \param str String to send (must be in flash, must be NUL terminated)
 */
uint8_t serial_tx_PGM(uint8_t portnum, const char *str);

/** \brief Send a number as a hexidecimal string to a serial port
 *  \param portnum Number of the port
 *  \param s Value to convert to string and send
 */
uint8_t serial_tx_hex(uint8_t portnum, uint8_t s);

/** \brief Send a number as a decimal string to a serial port
 *  \param portnum Number of the port
 *  \param s Value to convert to string and send
 */
uint8_t serial_tx_dec(uint8_t portnum, uint32_t s);

/** \brief Send a newline/return to a serial port
 *  \param portnum Number of the port
 */
uint8_t serial_tx_cr(uint8_t portnum);

/** \brief Test if there are unread characters in the buffer
 *  \param portnum Number of the port
 *  \return 1 if there are unread characters, 0 otherwise
 */
uint8_t serial_rx_available(uint8_t portnum);

/** \brief Return a single character from the port.
 *
 *  Since this has to be binary_safe, call serial_rx_available() before
 *  call this, since otherwise you'll get garbage
 *
 *  \param portnum Number of the port
 *  \return The character
 */
uint8_t serial_rx(uint8_t portnum);

#endif // SERIAL_H_INCLUDED
