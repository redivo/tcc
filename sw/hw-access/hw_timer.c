
#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif

#include "hw_general.h"

/******************************************************************************/
/**
 * \typedef timer_control_register_t
 * \brief This typedef is an union to describe the Timer Control Register (TxTCR). See item 23.6.2 from LPC23xx's datasheet.
 */
typedef union {
	struct {
		unsigned char counter_en:1;		//!< Enable counter.
		unsigned char counter_rst:1;	//!< Reset counter.
		unsigned char reserved:6;		//!< Reserved.
	} bits;
	unsigned char reg;
} timer_control_register_t;

/******************************************************************************/

int hw_timer_init()
{
#ifndef PC_COMPILATION
	timer_control_register_t tr;

	/* Disable timer */
	tr.reg = 0;
	T0TCR = tr.reg;

	/* Set counter to count in ms */
	T0PR = BOARD_CLOCK_FREQ/1000 - 1;

	/* Reset timer counter */
	tr.bits.counter_rst = 1;
	T0TCR = tr.reg;

	/* Enable timer */
	tr.bits.counter_en = 1;
	tr.bits.counter_rst = 0;
	T0TCR = tr.reg;
#endif

	return 0;
}

/******************************************************************************/

int hw_sleep(int t)
{
#ifndef PC_COMPILATION
	int expect_time;

	expect_time = T0TC + t;
	while (expect_time != T0TC);
#else
	sleep(t);
#endif

	return 0;
}

/******************************************************************************/

