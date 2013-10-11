#include "hw_control_pins.h"
#include "hw_general.h"
#include "errors.h"

/******************************************************************************/

/* Defines */
#define MAX_DIS_SEQ			3

/* Control pins macros */
#define FOR_EACH_SFP_DIS_SEQ(_seq) for (_seq = 0; _seq < MAX_DIS_SEQ; _seq++)

#define FOR_EACH_SFP_DIS_SEQ_INV(_seq) for (_seq = MAX_DIS_SEQ - 1; _seq >= 0; _seq--)

#define IS_VALID_SFP_DIS_SEQ(_seq) (_seq > 0 && _seq < MAX_DIS_SEQ)

/******************************************************************************/

/* Global Variables */
hw_pin_t Sfp_disable_seq[] =
{
	{4, 9, },
	{4, 10, },
	{4, 11, },
};

hw_pin_t Sfp_select = {4, 8, };

/******************************************************************************/

int hw_ctrl_pins_init(void)
{
	int seq;

	/* Initialize SFP Select pin. Initialize on SFP 0 */
	CHK(hw_pin_set_dir(Sfp_select.port, Sfp_select.pin, OUTPUT));
	CHK(hw_set_value(Sfp_select.port, Sfp_select.pin, 0));

	/* Initialize SFP Disable Sequencer pin. Initialize disabled */
	FOR_EACH_SFP_DIS_SEQ(seq) {
		CHK(hw_pin_set_dir(Sfp_disable_seq[seq].port, Sfp_disable_seq[seq].pin, OUTPUT));
		CHK(hw_set_value(Sfp_disable_seq[seq].port, Sfp_disable_seq[seq].pin, 1));
	}

	return 0;
}

/******************************************************************************/

int hw_sfp_insert(int sfp)
{
	int seq;

	CHK(hw_set_value(Sfp_select.port, Sfp_select.pin, !!sfp));

	/* "Insert" pins on order */
	FOR_EACH_SFP_DIS_SEQ(seq) {
		CHK(hw_set_value(Sfp_disable_seq[seq].port, Sfp_disable_seq[seq].pin, 0));
		// TODO insertion sleep
	}

	return 0;
}

/******************************************************************************/

int hw_sfp_remove(int sfp)
{
	int seq;

	/* "Remove" pins on order */
	FOR_EACH_SFP_DIS_SEQ_INV(seq) {
		CHK(hw_set_value(Sfp_disable_seq[seq].port, Sfp_disable_seq[seq].pin, 1));
		// TODO insertion sleep
	}

	return 0;
}

/******************************************************************************/

