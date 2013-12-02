#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif

#include "errors.h"
#include "cfg_control.h"
#include "hw_i2c_fault_injection.h"
#include "hw_i2c.h"

/******************************************************************************/

void hw_handle_irq_i2c_slv1(void)
{
#ifndef PC_COMPILATION
	static int c = 0;
	static int reg = 0;
	int cust_faults;
	int sfp = cfg_get_sfp_inserted();

	/* Check if there is some inserted SFP */
	if (sfp == ERR_NO_ONE_SFP_INSERTED)
		return;

	switch (I21STAT) {
		case 0x60:
			break;

		case 0x80:
			reg = I21DAT;
			break;

		case 0xA0:
			break;

		case 0xA8:
			/* Check for fault injection */
			if (Cfg.i2c_fi[0].enable) {
				if (!Cfg.i2c_fi[0].general_read_enable)
					break;

				/* Iterate over all custom faults (internal register specific faults) */
				for (cust_faults = 0; cust_faults < MAX_CUSTOM_FAULTS; cust_faults++) {
					if (Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].reg_addr != (reg + c))
						continue;

					/* Avoid to read when read is disabled */
					if (!Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].read_enable)
						break;

					/* Replay forced response, if it's the case */
					if (Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].forced_response != NO_FORCE_RESPONSE) {
						I21DAT = Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].forced_response;
						c++;
						break;
					}
				}
			}

			I21DAT = Sfp_mem_dev0[sfp][reg + c];
			c++;
			break;

		case 0xB8:
			/* Check for fault injection */
			if (Cfg.i2c_fi[0].enable) {
				if (!Cfg.i2c_fi[0].general_read_enable)
					break;

				/* Iterate over all custom faults (internal register specific faults) */
				for (cust_faults = 0; cust_faults < MAX_CUSTOM_FAULTS; cust_faults++) {
					if (Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].reg_addr != (reg + c))
						continue;

					/* Avoid to read when read is disabled */
					if (!Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].read_enable)
						break;

					/* Replay forced response, if it's the case */
					if (Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].forced_response != NO_FORCE_RESPONSE) {
						I21DAT = Cfg.i2c_fi[0].i2c_cust_fault[cust_faults].forced_response;
						c++;
						break;
					}
				}
			}

			I21DAT = Sfp_mem_dev0[sfp][reg + c];
			c++;
			break;

		case 0xC0:
			c = 0;
			reg = 0;
			break;

		default:
			break;
	}

	/* Set ACK and clear Interruption Address */
	I21CONSET = ACK;
	I21CONCLR = INT_FLAG;
	VICVectAddr = 0;
#endif
}

/******************************************************************************/

void hw_handle_irq_i2c_slv2(void)
{
#ifndef PC_COMPILATION
	static int c = 0;
	static int reg = 0;
	int cust_faults;
	int sfp = cfg_get_sfp_inserted();

	/* Check if there is some inserted SFP */
	if (sfp == ERR_NO_ONE_SFP_INSERTED)
		return;

	switch (I22STAT) {
		case 0x60:
			break;

		case 0x80:
			reg = I22DAT;
			break;

		case 0xA0:
			break;

		case 0xA8:
			/* Check for fault injection */
			if (Cfg.i2c_fi[1].enable) {
				if (!Cfg.i2c_fi[1].general_read_enable)
					break;

				/* Iterate over all custom faults (internal register specific faults) */
				for (cust_faults = 0; cust_faults < MAX_CUSTOM_FAULTS; cust_faults++) {
					if (Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].reg_addr != (reg + c))
						continue;

					/* Avoid to read when read is disabled */
					if (!Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].read_enable)
						break;

					/* Replay forced response, if it's the case */
					if (Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].forced_response != NO_FORCE_RESPONSE) {
						I22DAT = Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].forced_response;
						c++;
						break;
					}
				}
			}

			I22DAT = Sfp_mem_dev1[sfp][reg + c];
			c++;
			break;

		case 0xB8:
			/* Check for fault injection */
			if (Cfg.i2c_fi[1].enable) {
				if (!Cfg.i2c_fi[1].general_read_enable)
					break;

				/* Iterate over all custom faults (internal register specific faults) */
				for (cust_faults = 0; cust_faults < MAX_CUSTOM_FAULTS; cust_faults++) {
					if (Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].reg_addr != (reg + c))
						continue;

					/* Avoid to read when read is disabled */
					if (!Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].read_enable)
						break;

					/* Replay forced response, if it's the case */
					if (Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].forced_response != NO_FORCE_RESPONSE) {
						I22DAT = Cfg.i2c_fi[1].i2c_cust_fault[cust_faults].forced_response;
						c++;
						break;
					}
				}
			}

			I22DAT = Sfp_mem_dev1[sfp][reg + c];
			c++;
			break;

		case 0xC0:
			c = 0;
			reg = 0;
			break;

		default:
			break;
	}

	/* Set ACK and clear Interruption Address */
	I22CONSET = ACK;
	I22CONCLR = INT_FLAG;
	VICVectAddr = 0;
#endif
}

/******************************************************************************/

