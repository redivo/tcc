#include "hw_terminal.h"
#include "errors.h"
#include "cfg_general.h"
#include "cfg_i2c.h"

/******************************************************************************/

int cfg_set_enable(int sfp, bool enable)
{
	if (!IS_VALID_SFP(sfp))
		return ERR_INVALID_PARAM;

	Cfg.sfp[sfp].enable = enable;

	return 0;
}

/******************************************************************************/

int cfg_init(void)
{
	int sfp, fi;

	/* The default is all I2C working and no one SFP enabled */
	FOR_EACH_SFP(sfp) {
		/* Disable SFP */
		CHK(cfg_set_enable(sfp, 0));
	}

	FOR_EACH_FAULT_INJECTION(fi) {
		int fault;

		/* Disable fault injection */
		CHK(cfg_set_fault_injection_enable(fi, 0));

		/* Set device address based on default summing 1 to each next index */
		CHK(cfg_set_fault_injection_dev_address(fi, DEFAULT_DEV_ADDR));
//		CHK(cfg_set_fault_injection_dev_address(fi, DEFAULT_DEV_ADDR + fi));

		/* Enable general read and write */
		CHK(cfg_set_fault_injection_write_enable(fi, 1));
		CHK(cfg_set_fault_injection_read_enable(fi, 1));

		/* Enable normal read and write on all reg addresses */
		FOR_EACH_CUSTOM_FAULT(fault) {
			CHK(cfg_set_fi_cust_fault_reg_addr(fi, fault, 0));
			CHK(cfg_set_fi_cust_fault_read_enable(fi, fault, 1));
			CHK(cfg_set_fi_cust_fault_write_enable(fi, fault, 1));
			CHK(cfg_set_fi_cust_fault_forced_response(fi, fault, NO_FORCE_RESPONSE));
		}
	}

	return 0;
}

/******************************************************************************/

void cfg_show(void)
{
	int sfp, fi;

	pprintf(" ***************************************************************\r\n");
	pprintf(" **                   Current Configuration                   **\r\n");
	pprintf(" ***************************************************************\r\n");
	pprintf("\r\n");

	/* The default is all I2C working and no one SFP enabled */
	FOR_EACH_SFP(sfp)
		pprintf("  SFP %d: %sinserted\r\n", sfp + 1, Cfg.sfp[sfp].enable ? "" : "not ");

	FOR_EACH_FAULT_INJECTION(fi) {
		int fault;

		pprintf("\r\n");
		pprintf("\r\n");
		pprintf("  Fault Injection %d\r\n", fi + 1);
		pprintf("    Status:         %s\r\n", Cfg.i2c_fi[fi].enable ? "enabled" : "disabled");
		pprintf("    Device Address: 0x%02x\r\n", Cfg.i2c_fi[fi].dev_address);
		pprintf("    General read:   %s\r\n", Cfg.i2c_fi[fi].general_read_enable ? "enabled" : "disabled");
		pprintf("    General write:  %s\r\n", Cfg.i2c_fi[fi].general_write_enable ? "enabled" : "disabled");
		pprintf("\r\n");
		pprintf("     Custom Fault # | Register Address | R/W | Forced Response \r\n");
		pprintf("    ----------------+------------------+-----+-----------------\r\n");

		FOR_EACH_CUSTOM_FAULT(fault) {
#define C_FAULT Cfg.i2c_fi[fi].i2c_cust_fault[fault]
			char forced_resp_str[9];

			/* Fill forced response string */
			if (C_FAULT.forced_response == NO_FORCE_RESPONSE)
				sprintf(forced_resp_str, "No force");
			else
				sprintf(forced_resp_str, "0x%02x", C_FAULT.forced_response);

			pprintf("           %2d       |       0x%02x       |  %s |        %s\r\n",
					fault + 1,
					C_FAULT.reg_addr,
					C_FAULT.read_enable && C_FAULT.write_enable ? "RW" :
						C_FAULT.read_enable ? "RO" :
						C_FAULT.write_enable ? "WO" : "--",
					forced_resp_str);
#undef C_FAULT
		}
	}

	pprintf("\r\n");
}

/******************************************************************************/

