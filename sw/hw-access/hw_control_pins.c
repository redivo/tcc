#include "hw_control_pins.h"
#include "hw_general.h"
#include "hw_timer.h"
#include "errors.h"

/******************************************************************************/

/* Global Variables */
hw_pin_t Sfp_disable_mux = {4, 9, };

hw_pin_t Sfp_supply_disable[] =
{
	{4, 10, },
	{4, 11, },
};

hw_pin_t Sfp_select = {4, 8, };

/******************************************************************************/

int hw_ctrl_pins_init(void)
{
	/* Initialize SFP Select pin. Initialize on SFP 0 */
	CHK(hw_pin_set_dir(Sfp_select.port, Sfp_select.pin, OUTPUT));
	CHK(hw_set_value(Sfp_select.port, Sfp_select.pin, 0));

	/* Initialize with both SFPs' supply disabled */
	// TODO
	CHK(hw_pin_set_dir(Sfp_supply_disable[0].port, Sfp_supply_disable[0].pin, OUTPUT));
	CHK(hw_set_value(Sfp_supply_disable[0].port, Sfp_supply_disable[0].pin, 1));

	CHK(hw_pin_set_dir(Sfp_supply_disable[1].port, Sfp_supply_disable[1].pin, OUTPUT));
	CHK(hw_set_value(Sfp_supply_disable[1].port, Sfp_supply_disable[1].pin, 1));

	/* Initialize SFP Disable Sequencer pin. Initialize disabled */
	CHK(hw_pin_set_dir(Sfp_disable_mux.port, Sfp_disable_mux.pin, OUTPUT));
	CHK(hw_set_value(Sfp_disable_mux.port, Sfp_disable_mux.pin, 1));

	return 0;
}

/******************************************************************************/

int hw_sfp_insert(int sfp)
{
	CHK(hw_set_value(Sfp_select.port, Sfp_select.pin, !!sfp));

	/* Insert supple pins */
	CHK(hw_set_value(Sfp_supply_disable[sfp].port, Sfp_supply_disable[sfp].pin, 0));

	/* Wait a little time to emulate mechanical insertion */
	hw_sleep(200);

	/* Insert the other pins */
	CHK(hw_set_value(Sfp_disable_mux.port, Sfp_disable_mux.pin, 0));

	/* Update SFP memory data */
	hw_sleep(2000); // Wait a little time to SFP boot
//	CHK(hw_sfp_memory_update(sfp));

	return 0;
}

/******************************************************************************/

int hw_sfp_remove(int sfp)
{
	/* Remove data pins */
	CHK(hw_set_value(Sfp_disable_mux.port, Sfp_disable_mux.pin, 1));

	/* Wait a little time to emulate mechanical removing */
	hw_sleep(200);

	CHK(hw_set_value(Sfp_supply_disable[sfp].port, Sfp_supply_disable[sfp].pin, 1));

	/* Update SFP memory data */
//	CHK(hw_sfp_memory_update(sfp));

	return 0;
}

/******************************************************************************/

