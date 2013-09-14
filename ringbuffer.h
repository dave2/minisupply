/* Public interface to generic ringbuffer code */

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

#ifndef RINGBUFFER_H_INCLUDED
#define RINGBUFFER_H_INCLUDED

/* Ringbuffer storage. Allocate a singular ringbuffer of the given size */

typedef struct {
		char *buf;
		uint8_t head;
		uint8_t tail;
		uint8_t mask;
} ringbuffer_t;

/* create ring buffer as specified */
ringbuffer_t *ring_create(uint8_t mask);

/* wipe the contents of the ringbuffer and reset it */
void ring_reset(ringbuffer_t *ring);

/* destroy a ringbuffer, freeing all resources INCLUDING the passed metadata pointer */
void ring_destroy(ringbuffer_t *ring);

/* write to the ringbuffer, unsafe version assumes interrupts globally disabled. returns sucess/failure */

uint8_t ring_write(ringbuffer_t *ring, char value);
uint8_t ring_write_unsafe(ringbuffer_t *ring, char value);

/* read from the ringbuffer, you must check it's readable first or you'll get odd results (since this has to be binary safe) */
char ring_read(ringbuffer_t *ring);
char ring_read_unsafe(ringbuffer_t *ring);

/* do we have anything yet to be read? */
uint8_t ring_readable(ringbuffer_t *ring);
uint8_t ring_readable_unsafe(ringbuffer_t *ring);


#endif // RINGBUFFER_H_INCLUDED
