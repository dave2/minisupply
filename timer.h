#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

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

/* XMEGA timer public interface */

/** \brief Various clock sources for the timer
 *
 *  Split between CLKper with various divisors, and events on an
 *  event channel. */
typedef enum {
	timer_off = 0, /**< Stop running the timer */
	timer_perdiv1, /**< CLKper/1 */
	timer_perdiv2, /**< CLKper/2 */
	timer_perdiv4, /**< CLKper/4 */
	timer_perdiv8, /**< CLKper/8 */
	timer_perdiv64, /**< CLKper/64 */
	timer_perdiv256, /**< CLKper/256 */
	timer_perdiv1024, /**< CLKper/1024 */
	timer_ev0, /**< Event Channel 0 */
	timer_ev1, /**< Event Channel 1 */
	timer_ev2, /**< Event Channel 2 */
	timer_ev3, /**< Event Channel 3 */
	timer_ev4, /**< Event Channel 4 */
	timer_ev5, /**< Event Channel 5 */
	timer_ev6, /**< EVent Channel 6 */
	timer_ev7, /**< Event Channel 7 */
} timer_clk_src_t;

/** \brief Timer PWM modes */
typedef enum {
	timer_norm, /**< Normal 0->PER, no outputs on compare */
	timer_freq, /**< Frequency generation */
	timer_reserved1, /**< Unused */
	timer_pwm, /**< PWM: Single Slope */
	timer_reserved2, /**< Unused */
	timer_pwm_dstop, /**< PWM: Dual Slope, OVF/Event at top */
	timer_pwm_dsboth, /**< PWM: Dual Slope, OVF/Event at both */
	timer_pwm_dsbot, /**< PWM: Dual Slope, OVF/Event at bottom */
} timer_pwm_t;

/** \brief Timer channels */
typedef enum {
	timer_ch_a = 0, /**< Channel A */
	timer_ch_b, /**< Channel B */
	timer_ch_c, /**< Channel C, Type 0 only */
	timer_ch_d, /**< Channel D, Type 0 only */
} timer_chan_t;

/** \brief Initialise the given timer slot
 *  \param timernum Number of the timer
 *  \param mode Timer mode (normal, pwm, etc)
 *  \param period Timer period (ie, TOP value)
 *  \return 0 on success, errors.h otherwise */
uint8_t timer_init(uint8_t timernum, timer_pwm_t mode, uint16_t period);

/** \brief Clock the timer from the given source
 *
 *  Note: this starts the timer running for any value other than
 *  timer_off.
 *
 *  \param timernum Number of the timer
 *  \param clk Clock source/divider
 *  \return 0 on success, errors.h otherwise.  */
uint8_t timer_clk(uint8_t timernum, timer_clk_src_t clk);

/** \brief Set the given channel compare
 *
 *  The cmp_hook function must return void, and accept
 *  a single uint8_t argument being the channel number of the compare
 *  event. They can be NULL if unused.
 *
 *  Note: In PWM modes, compares also toggle their associated pins
 *  automatically, if set as output.
 *
 *  Note: This uses the compare buffer, so is safe to do during timer
 *  running.
 *
 *  \param timernum Number of the timer
 *  \param ch Channel
 *  \param value Value to compare against
 *  \param cmp_hook Compare hook function.
 *  \param cmp_ev Event channel to strobe on compare. -1 is none.
 *  \return 0 for success, errors.h otherwise
 */
uint8_t timer_comp(uint8_t timernum, timer_chan_t ch, uint16_t value,
	void (*cmp_hook)(uint8_t), uint8_t cmp_ev);

/** \brief set the given channel overflow behaviour
 *
 *  The ovf_hook function must return void, and accept a single uint8_t
 *  of argument, being the channel number of the overflow event.
 *  They can be NULL if unused.
 *
 *  Note: In PWM modes, overflow also resets the state of the
 *  associated pins automatically, if set as output
 *
 *  \param timernum Number of the timer
 *  \param ovf_hook Overflow hook function
 *  \param ovf_ev Even channel to strobe on overflow, -1 is none.
 *  \return 0 for success, errors.h otherwise
 */
uint8_t timer_ovf(uint8_t timernum, void (*ovf_hook)(uint8_t),
	uint8_t ovf_ev);

/** \brief Set the given channel compare (value only)
 *
 *  This is a lightweight version of timer_comp() designed to just
 *  change the compare value for the channel.
 *
 *  \param timernum Number of the timer
 *  \param ch Channel
 *  \param value Compare value
 *  \return 0 for success, errors.h otherwise
 */
uint8_t timer_comp_val(uint8_t timernum, timer_chan_t ch, uint16_t value);

/** \brief Force a specific count value
 *
 *  Note: timer must be stopped for this to be safe.
 *  \param timernum Number of the timer
 *  \param value Count value
 *  \return 0 for success, errors.h otherwise
 */
uint8_t timer_count(uint8_t timernum, uint16_t value);

#endif // TIMER_H_INCLUDED
