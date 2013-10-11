#ifndef __HW_LEDS_H
#define __HW_LEDS_H

#include <stdbool.h>

/******************************************************************************/
/**
 * \brief	Initialize leds
 * \return	0 if OK, error code otherwise.
 */
int hw_leds_init(void);

/******************************************************************************/
/**
 * \brief	Turn on a LED
 * \param led		Led number.
 * \param enable	Inform if the led must be enabled or disabled.
 * \return	0 if OK, error code otherwise.
 */
int hw_enable_led(int led, bool enable);

/******************************************************************************/

#endif /* __HW_LEDS_H */

