#include "errors.h"
#include "hw_leds.h"
#include "hw_control_pins.h"
#include "cfg_general.h"

/******************************************************************************/

int cfg_insert_sfp(int sfp)
{
	int s;

	if (!IS_VALID_SFP(sfp))
		return ERR_INVALID_PARAM;

	/* Only one SFP can be inserted */
	FOR_EACH_SFP(s)
		if (Cfg.sfp[s].enable && s != sfp)
			return ERR_SFP_COLISION;

	/* Enable SFP LED, SFP select and "insert" pins */
	CHK(cfg_set_enable(sfp, 1));
	CHK(hw_enable_led(sfp, 1));
	CHK(hw_sfp_insert(sfp));

	return 0;
}

/******************************************************************************/

int cfg_remove_sfp(int sfp)
{
	/* Disable SFP LED and "remove" pins */
	CHK(cfg_set_enable(sfp, 0));
	CHK(hw_enable_led(sfp, 0));
	CHK(hw_sfp_remove(sfp));

	return 0;
}

/******************************************************************************/

