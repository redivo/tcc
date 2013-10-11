#ifndef __HW_I2C_H
#define __HW_I2C_H

/******************************************************************************/
/**
 * \brief	Set I²C slave address.
 * \param i2c_number	I²C interface number.
 * \param address		I²C slave address.
 * \return	0 if OK, error code otherwise.
 */
int hw_set_i2c_address(int i2c_number, unsigned char address);

/******************************************************************************/
/**
 * \brief	Get I²C slave address.
 * \param i2c_number	I²C interface number.
 * \param address		Pointer to I²C slave address be stored.
 * \return	0 if OK, error code otherwise.
 */
int hw_get_i2c_address(int i2c_number, unsigned char *address);

/******************************************************************************/
/**
 * \brief	Initialize all I²C buses.
 * \return	0 if OK, error code otherwise.
 */
int hw_i2c_init(void);

/******************************************************************************/

#endif /* __HW_I2C_INIT */

