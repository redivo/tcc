#include "cli.h"
#include "errors.h"
#include "cfg_debug.h"

/******************************************************************************/

int cli_debug_read(char *line, int num_of_args)
{
#ifdef DEBUG_MODE
	char reg[MAX_ARG_SIZE];

	/* Get the register name */
	CHK(cli_get_word(line, reg, sizeof(reg), 2));

	/* Call debug with write in 0 and dummy value */
	CHK(cfg_dbg_reg(0, reg, 0));

#endif /* DEBUG_MODE */
	return 0;
}

/******************************************************************************/

int cli_debug_write(char *line, int num_of_args)
{
#ifdef DEBUG_MODE
	char reg[MAX_ARG_SIZE];
	char value_str[MAX_ARG_SIZE];
	int value;

	/* Get the register name */
	CHK(cli_get_word(line, reg, sizeof(reg), 2));

	/* Get value to be written */
	CHK(cli_get_word(line, value_str, sizeof(value_str), 3));

	/* Convert strings to int */
	value = (int) strtol(value_str, &value_str, 16);

	/* Call debug with write in 1 */
	CHK(cfg_dbg_reg(1, reg, value));

#endif /* DEBUG_MODE */
	return 0;
}

/******************************************************************************/

