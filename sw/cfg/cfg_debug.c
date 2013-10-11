#include "hw_debug.h"
#include "hw_terminal.h"
#include "errors.h"
#include "debug.h"

int cfg_dbg_reg(bool write, char *reg, int value)
{
#ifdef DEBUG_MODE
	int reg_value;

	if ((reg_value = hw_dbg_get_reg(write, reg, value)) < 0)
		return reg_value;

	pprintf("%s = 0x%x\r\n", reg, reg_value);

#endif /* DEBUG_MODE */
	return 0;
}

