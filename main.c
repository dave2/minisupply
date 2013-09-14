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

#include "global.h"
#include <avr/io.h>
#include <avr/interrupt.h>


int main(void)
{
    /* ensure global interrupt flag is off while we change clocks */
    cli();

    /* set up the system clock so that we're running at 32MHz */
    OSC.CTRL = OSC_RC32MEN_bm; /* run the 32MHz OSC */
    while (!(OSC.STATUS & OSC_RC32MRDY_bm)); /* wait for it to stablise */
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc; /* select 32MHz clock as system clock */

    /* STALL */

    /* power down everything we don't need */
    /* this violates the datasheet by writing 1 into reserved bits */
    PR.PRGEN = 0xff;
    PR.PRPA = 0xff;

    PR.PRPC = 0xff;
    PR.PRPD = 0xff;
    PR.PRPE = 0xff;
    PR.PRPF = 0xff;

    /* never reached */
    return 0;
}
