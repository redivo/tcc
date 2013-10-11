#ifndef __HW_GENERAL_H
#define __HW_GENERAL_H

#include <stdbool.h>

/* Hardware access general defines */
#define INPUT	0
#define OUTPUT	1

/******************************************************************************/

/* Pin struct */
typedef struct {
	int port;
	int pin;
} hw_pin_t;

/******************************************************************************/
/**
 * \brief	Initialize pin signals, leds and displays
 * \return	0 if OK, error code otherwise.
 */
int hw_init(void);

/******************************************************************************/
/**
 * \brief	Set a value to a port pin
 * \param port		Port to be set.
 * \param pin		Pin (bit) to be set.
 * \param val		Value to be set.
 * \return	0 if OK, error code otherwise.
 */
int hw_set_value(int port, int pin, bool val);

/******************************************************************************/
/**
 * \brief	Set the direction of a uPC pin
 * \param port		Port to be set.
 * \param pin		Pin (bit) to be set.
 * \param dir		Direction to be set (0 to output and 1 to input).
 * \return	0 if OK, error code otherwise.
 */
int hw_pin_set_dir(int port, int pin, int dir);

/******************************************************************************/

#endif /* __HW_GENERAL_H */

