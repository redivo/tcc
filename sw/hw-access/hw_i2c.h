#ifndef __HW_I2C_H
#define __HW_I2C_H

/******************************************************************************/

/* Master Transmitter mode states */
#define MT_START_TXD			0x08
#define MT_REPEATED_START_TXD	0x10
#define MT_SLA_W_TXD_ACK_RXD	0x18
#define MT_SLA_W_TXD_NACK_RXD	0x20
#define MT_DATA_TXD_ACK_RXD		0x28
#define MT_DATA_TXD_NAK_RXD		0x20
#define MT_ARB_LOST				0x38

/* Master Receiver mode states */
#define MR_START_TXD			0x08
#define MR_REPEATED_START_TXD	0x10
#define MR_ARB_LOST				0x38
#define MR_SLA_R_TXD_ACK_RXD	0x40
#define MR_SLA_R_TXD_NAK_RXD	0x48
#define MR_DATA_RXD_ACK_TXD		0x50
#define MR_DATA_RXD_NACK_TXD	0x58

/* Slave Receiver mode states */
#define SR_OWN_SLA_W_RXD_ACK_TXD	0x60
#define SR_ARB_LOST_OWN_SLA_RXD		0x68
#define SR_GEN_CALL_RXD_ACK_TXD		0x70
#define SR_ARB_LOST_GEN_CALL_RXD	0x78
#define SR_DATA_RXD_OWN_ACK_TXD		0x80
#define SR_DATA_RXD_OWN_NACK_TXD	0x88
#define SR_DATA_RXD_GEN_ACK_TXD		0x90
#define SR_DATA_RXD_GEN_NACK_RXD	0x98
#define SR_STOP_OR_REP_START_RXD	0xA0

/* Slave Transmitter mode states */
#define ST_OWN_SLA_R_RXD_ACK_TXD	0xA8
#define ST_ARB_LOST					0xB0
#define ST_DATA_TXD_ACK_RXD			0xB8
#define ST_DATA_TXD_NACK_RXD		0xC0
#define ST_LAST_DATA_TXD_ACK_RXD	0xC8

/* General states */
#define G_ERROR		0x00
#define g_SLEEP		0xF8

#define START 0x20
#define STOP  0x10
#define ACK 0x04
#define NACK  0x04
#define START_FLAG 0x20
#define INT_FLAG 0x08
#define SMARTCARD_LEITURA 0xA1
#define SMARTCARD_ESCRITA 0xA0

#define SEND_START_BIT			0x08
#define END_OF_RECEIVE			0x58
#define ENVIOU_ENDERECO_ESCRITA 0x18
#define ENVIOU_ENDERECO_LEITURA	0x50
#define ENVIOU_ENDERECO_MEMORIA 0x28
#define RECEBEU_ACK				0x40
#define FIM_ESCRITA				0x28
#define FIM_LEITURA				0x58

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
/**
 * \brief	Read an address from an I2C device.
 * \param dev_addr	I²C device address.
 * \param reg_addr	Internal register address.
 * \return	Return the read value.
 */
int hw_i2c_read(int dev_addr, int reg_addr);

/******************************************************************************/
/**
 * \brief	Update memory of an SFP.
 * \param sfp	SFP number
 * \return	0 if OK, error code otherwise.
 */
int hw_sfp_memory_update(int sfp);

/******************************************************************************/

#endif /* __HW_I2C_INIT */

