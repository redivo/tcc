#include "hw_leds.h"
#include "hw_general.h"
#include "../errors.h"

/******************************************************************************/

/* Leds defines */
#define MAX_LEDS	2

/******************************************************************************/

/* Leds macros */
#define FOR_EACH_LED(_led) for (_led = 0; _led < MAX_LEDS; _led++)

#define IS_VALID_LED(_led) (_led > 0 && _led < MAX_LEDS)

/******************************************************************************/

/* Global Variables */
hw_pin_t Leds[] =
{
	{4, 12, },
	{4, 13, },
};

/******************************************************************************/

int hw_leds_init(void)
{
	int led;

	/* Initialize les. They starts turned off */
	FOR_EACH_LED(led) {
		CHK(hw_pin_set_dir(Leds[led].port, Leds[led].pin, OUTPUT));
		CHK(hw_enable_led(led, 0));
	}

	return 0;
}

/******************************************************************************/

int hw_enable_led(int led, bool enable)
{
	CHK(hw_set_value(Leds[led].port, Leds[led].pin, enable));

	return 0;
}

/******************************************************************************/

