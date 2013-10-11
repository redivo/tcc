#include "errors.h"
#include "cfg_general.h"
#include "hw_i2c.h"

/* Defines */
#define FI_INDEX_TO_I2C_NUMBER(_i) (_i + 1)

/******************************************************************************/

int cfg_set_fault_injection_enable(int index, int enable)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[index].enable = !!enable;

	return 0;
}

/******************************************************************************/

int cfg_set_fault_injection_dev_address(int index, unsigned char address)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(index))
		return ERR_INVALID_PARAM;

	if (!IS_VALID_DEV_ADDR(address))
		return ERR_INVALID_PARAM;

	CHK(hw_set_i2c_address(FI_INDEX_TO_I2C_NUMBER(index), address));

	Cfg.i2c_fi[index].dev_address = address;

	return 0;
}

/******************************************************************************/

int cfg_set_fault_injection_write_enable(int index, bool enable)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[index].general_write_enable = enable;

	return 0;
}

/******************************************************************************/

int cfg_set_fault_injection_read_enable(int index, bool enable)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[index].general_read_enable = enable;

	return 0;
}

/******************************************************************************/

int cfg_set_fi_cust_fault_reg_addr(int fi, int custom_fautl_index, unsigned char address)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(fi))
		return ERR_INVALID_PARAM;

	if (!IS_VALID_CUST_FAULT(custom_fautl_index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[fi].i2c_cust_fault[custom_fautl_index].reg_addr = address;

	return 0;
}
/******************************************************************************/

int cfg_set_fi_cust_fault_read_enable(int fi, int custom_fautl_index, bool enable)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(fi))
		return ERR_INVALID_PARAM;

	if (!IS_VALID_CUST_FAULT(custom_fautl_index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[fi].i2c_cust_fault[custom_fautl_index].read_enable = enable;

	return 0;
}

/******************************************************************************/

int cfg_set_fi_cust_fault_write_enable(int fi, int custom_fautl_index, bool enable)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(fi))
		return ERR_INVALID_PARAM;

	if (!IS_VALID_CUST_FAULT(custom_fautl_index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[fi].i2c_cust_fault[custom_fautl_index].write_enable = enable;

	return 0;
}

/******************************************************************************/

int cfg_set_fi_cust_fault_forced_response(int fi, int custom_fautl_index, int response)
{
	if (!IS_VALID_FAULT_INJECTION_INDEX(fi))
		return ERR_INVALID_PARAM;

	if (!IS_VALID_CUST_FAULT(custom_fautl_index))
		return ERR_INVALID_PARAM;

	Cfg.i2c_fi[fi].i2c_cust_fault[custom_fautl_index].forced_response = response;

	return 0;
}

/******************************************************************************/

