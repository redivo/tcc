#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif
#include "errors.h"
#include "hw_i2c.h"
#include "hw_leds.h"
#include "hw_control_pins.h"
#include "hw_timer.h"

/******************************************************************************/

int hw_init(void)
{
	CHK(hw_leds_init());
	CHK(hw_ctrl_pins_init());
	CHK(hw_i2c_init());
	CHK(hw_timer_init());

	return 0;
}

/******************************************************************************/

int hw_set_value(int port, int pin, bool val)
{
#ifndef PC_COMPILATION
	switch (port) {
		case 0:
			if (val == 0)
				FIO0CLR |= (1 << pin);
			else
				FIO0SET |= (1 << pin);
			break;

		case 1:
			if (val == 0)
				FIO1CLR |= (1 << pin);
			else
				FIO1SET |= (1 << pin);
			break;

		case 2:
			if (val == 0)
				FIO2CLR |= (1 << pin);
			else
				FIO2SET |= (1 << pin);
			break;

		case 3:
			if (val == 0)
				FIO3CLR |= (1 << pin);
			else
				FIO3SET |= (1 << pin);
			break;

		case 4:
			if (val == 0)
				FIO4CLR |= (1 << pin);
			else
				FIO4SET |= (1 << pin);
			break;

		default:
			return ERR_INVALID_UPC_PORT;
	}

#endif
	return 0;
}

/******************************************************************************/

int hw_pin_set_dir(int port, int pin, int dir)
{
#ifndef PC_COMPILATION
	switch (port) {
		case 0:
			if (dir == 0)
				FIO0DIR &= ~(1 << pin);
			else
				FIO0DIR |= (1 << pin);
			break;

		case 1:
			if (dir == 0)
				FIO1DIR &= ~(1 << pin);
			else
				FIO1DIR |= (1 << pin);
			break;

		case 2:
			if (dir == 0)
			FIO2DIR &= ~(1 << pin);
			else
				FIO2DIR |= (1 << pin);
			break;

		case 3:
			if (dir == 0)
				FIO3DIR &= ~(1 << pin);
			else
				FIO3DIR |= (1 << pin);
			break;

		case 4:
			if (dir == 0)
				FIO4DIR &= ~(1 << pin);
			else
				FIO4DIR |= (1 << pin);
			break;

		default:
			return ERR_INVALID_UPC_PORT;
	}

#endif
	return 0;
}

/******************************************************************************/

