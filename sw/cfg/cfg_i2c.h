#ifndef __CFG_I2C_H
#define __CFG_I2C_H

/******************************************************************************/

/* Defines */
#define DEFAULT_DEV_ADDR	0x50

/******************************************************************************/
/**
 * \brief	Set fault injection enable
 * \param index		Fault injection index
 * \param enable	Inform if fault injection must be enabled
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fault_injection_enable(int index, int enable);

/******************************************************************************/
/**
 * \brief	Set the device address to be listened
 * \param index		Fault injection index
 * \param address	Device address to be configured
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fault_injection_dev_address(int index, unsigned char address);

/******************************************************************************/
/**
 * \brief	Set I²C device write enable
 * \param index		Fault injection index
 * \param enable	Inform if the I²C device is writable
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fault_injection_write_enable(int index, bool enable);

/******************************************************************************/
/**
 * \brief	Set I²C device read enable
 * \param index		Fault injection index
 * \param enable	Inform if the I²C device is readable
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fault_injection_read_enable(int index, bool enable);

/******************************************************************************/
/**
 * \brief	Set the register address for a custom fault injection
 * \param fi					Fault injection index
 * \param custom_fautl_index	Custom fault injection index
 * \param address				Register address to be set
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fi_cust_fault_reg_addr(int fi, int custom_fautl_index, unsigned char address);

/******************************************************************************/
/**
 * \brief	Set read enable in a custom fault injection
 * \param fi					Fault injection index
 * \param custom_fautl_index	Custom fault injection index
 * \param enable				Inform if read must be enabled
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fi_cust_fault_read_enable(int fi, int custom_fautl_index, bool enable);

/******************************************************************************/
/**
 * \brief	Set write enable in a custom fault injection
 * \param fi					Fault injection index
 * \param custom_fautl_index	Custom fault injection index
 * \param enable				Inform if write must be enabled
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fi_cust_fault_write_enable(int fi, int custom_fautl_index, bool enable);

/******************************************************************************/
/**
 * \brief	Set forced response in a I²C read
 * \param fi					Fault injection index
 * \param custom_fautl_index	Custom fault injection index
 * \param response				Response to be forced
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_fi_cust_fault_forced_response(int fi, int custom_fautl_index, int response);

/******************************************************************************/

#endif /* __CFG_I2C_H */

