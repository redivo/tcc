#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif
#include <string.h>
#include "errors.h"

#define START 0x20
#define STOP  0x10
#define ACK 0x04
#define NACK  0x04
#define START_FLAG 0x20
#define INT_FLAG 0x008
#define SMARTCARD_LEITURA 0xA1
#define SMARTCARD_ESCRITA 0xA0

#define ENVIOU_START 0x08
#define ENVIOU_ENDERECO_ESCRITA 0x18
#define ENVIOU_ENDERECO_LEITURA	0x50
#define ENVIOU_ENDERECO_MEMORIA 0x28
#define FIM_ESCRITA	0x28
#define FIM_LEITURA 0x58
#define RECEBEU_ACK 0x40

/* Boar clock frequency in kHz TODO */
#define BOARD_CLOCK_FREQ	12000

/* I²C interfaces number */
#define I2C_IF_TO_SFP			0 // Interface to be linked on SFP bus. It works like mater.
#define I2C_IF_TO_SWITCH_ADDR_0	1 // Interface to be linked on switch bus. It works like slave.
#define I2C_IF_TO_SWITCH_ADDR_1	2 // Interface to be linked on switch bus. It works like slave.

/* I²C interface power bit in Poer Control for Peripherals (PCONP) register */
#define PCONP_I2C_TO_SFP		(1 << 7)
#define PCONP_I2C_TO_SW_ADDR_0	(1 << 19)
#define PCONP_I2C_TO_SW_ADDR_1	(1 << 26)

/* I²C interface mode Pin Selection, offsets and values */
#define PINSEL_I2C_TO_SFP_SDA				PINSEL1
#define PINSEL_I2C_TO_SFP_SCL				PINSEL1
#define PINSEL_I2C_TO_SFP_SDA_OFFSET		22
#define PINSEL_I2C_TO_SFP_SCL_OFFSET		24
#define PINSEL_I2C_TO_SFP_SDA_VALUE			1
#define PINSEL_I2C_TO_SFP_SCL_VALUE			1
#define PINSEL_I2C_TO_SW_ADDR_0_SDA			PINSEL1
#define PINSEL_I2C_TO_SW_ADDR_0_SCL			PINSEL1
#define PINSEL_I2C_TO_SW_ADDR_0_SDA_OFFSET	6
#define PINSEL_I2C_TO_SW_ADDR_0_SCL_OFFSET	8
#define PINSEL_I2C_TO_SW_ADDR_0_SDA_VALUE	3
#define PINSEL_I2C_TO_SW_ADDR_0_SCL_VALUE	3
#define PINSEL_I2C_TO_SW_ADDR_1_SDA			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_1_SCL			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_1_SDA_OFFSET	20
#define PINSEL_I2C_TO_SW_ADDR_1_SCL_OFFSET	22
#define PINSEL_I2C_TO_SW_ADDR_1_SDA_VALUE	2
#define PINSEL_I2C_TO_SW_ADDR_1_SCL_VALUE	2

/******************************************************************************/
/**
 * \typedef i2c_control_t
 * \brief This typedef is an union to describe I²C Control register.
 */
typedef union {
	struct {
		unsigned char reserved1:2;	//!< Reserved
		unsigned char assert_ack:1;	//!< Assert ack flag.
		unsigned char interrupt:1;	//!< I²C interrupt flag.
		unsigned char stop:1;		//!< STOP flag.
		unsigned char start:1;		//!< START flag.
		unsigned char enable:1;		//!< I²C interface enable.
		unsigned char reserved2:1;	//!< Reserved
	} bits;
	unsigned char reg;
} i2c_control_t;

/******************************************************************************/
/**
 * \brief	Set the clock speed of an I²C port.
 * \param i2c_number	I²C interface number.
 * \param ferq			I²C clock frequency in kHz to be set.
 * \return	0 if OK, error code otherwise.
 */
static int hw_set_i2c_clock_speed(int i2c_number, int freq)
{
#ifndef PC_COMPILATION
	int period;

	/* The formula to calculate the period is defined in LPC2378 datasheet */
	period = BOARD_CLOCK_FREQ / freq;

	/* Set the period in the right register */
	switch (i2c_number) {
		case 0:
			I20SCLH = period / 2;
			I20SCLL = period / 2;
			break;

		case 1:
			I21SCLH = period / 2;
			I21SCLL = period / 2;
			break;

		case 2:
			I22SCLH = period / 2;
			I22SCLL = period / 2;
			break;

		default:
			return ERR_INVALID_I2C;
	}

#endif
	return 0;
}

/******************************************************************************/
/**
 * \brief	Set the I²C control.
 * \param i2c_number	I²C interface number.
 * \param i2c_control	I²C control register value.
 * \return	0 if OK, error code otherwise.
 */
static int hw_set_i2c_control(int i2c_number, unsigned char i2c_control)
{
#ifndef PC_COMPILATION
	switch (i2c_number) {
		case 0:
			I20CONCLR = ~i2c_control;
			I20CONSET = i2c_control;
			break;

		case 1:
			I21CONCLR = ~i2c_control;
			I21CONSET = i2c_control;
			break;

		case 2:
			I22CONCLR = ~i2c_control;
			I22CONSET = i2c_control;
			break;

		default:
			return ERR_INVALID_I2C;
	}

#endif
	return 0;
}

/******************************************************************************/

int hw_set_i2c_address(int i2c_number, unsigned char address)
{
#ifndef PC_COMPILATION
	switch (i2c_number) {
		case 0:
			I20ADR = address;
			break;

		case 1:
			I21ADR = address;
			break;

		case 2:
			I22ADR = address;
			break;

		default:
			return ERR_INVALID_I2C;
	}

#endif
	return 0;
}

/******************************************************************************/
int hw_get_i2c_address(int i2c_number, unsigned char *address)
{
#ifndef PC_COMPILATION
	switch (i2c_number) {
		case 0:
			*address = I20ADR;
			break;

		case 1:
			*address = I21ADR;
			break;

		case 2:
			*address = I22ADR;
			break;

		default:
			return ERR_INVALID_I2C;
	}

#endif
	return 0;
}

/******************************************************************************/

int hw_i2c_init(void)
{
#ifndef PC_COMPILATION
	i2c_control_t i2c_control;

	/************************************************************/
	/* Initialize I²C 0 as master, to be linked in SFP side bus */
	/************************************************************/
	memset(&i2c_control, 0, sizeof(i2c_control));

	/* Turn on power */
	PCONP |= PCONP_I2C_TO_SFP;

	/* Selec SCL and SDA mode to respective pins */
	PINSEL_I2C_TO_SFP_SDA |= (PINSEL_I2C_TO_SFP_SDA_VALUE << PINSEL_I2C_TO_SFP_SDA_OFFSET);
	PINSEL_I2C_TO_SFP_SCL |= (PINSEL_I2C_TO_SFP_SCL_VALUE << PINSEL_I2C_TO_SFP_SCL_OFFSET);

	/* Configure I²C interface */
	i2c_control.bits.enable = 1;
	CHK(hw_set_i2c_control(I2C_IF_TO_SFP, i2c_control.reg));
	CHK(hw_set_i2c_clock_speed(I2C_IF_TO_SFP, 60));


	/************************************************************************/
	/* Initialize I²C 1 as addressed slave, to be linked in switch side bus */
	/************************************************************************/
	memset(&i2c_control, 0, sizeof(i2c_control));

	/* Turn on power */
	PCONP |= PCONP_I2C_TO_SW_ADDR_0;

	/* Selec SCL and SDA mode to respective pins */
	PINSEL_I2C_TO_SW_ADDR_0_SDA |= (PINSEL_I2C_TO_SW_ADDR_0_SDA_VALUE << PINSEL_I2C_TO_SW_ADDR_0_SDA_OFFSET);
	PINSEL_I2C_TO_SW_ADDR_0_SCL |= (PINSEL_I2C_TO_SW_ADDR_0_SCL_VALUE << PINSEL_I2C_TO_SW_ADDR_0_SCL_OFFSET);

	/* Configure I²C interface */
	i2c_control.bits.enable = 1;
	i2c_control.bits.assert_ack = 1;
	CHK(hw_set_i2c_control(I2C_IF_TO_SWITCH_ADDR_0, i2c_control.reg));
	CHK(hw_set_i2c_address(I2C_IF_TO_SWITCH_ADDR_0, 0));


	/************************************************************************/
	/* Initialize I²C 2 as addressed slave, to be linked in switch side bus */
	/************************************************************************/
	memset(&i2c_control, 0, sizeof(i2c_control));

	/* Turn on power */
	PCONP |= PCONP_I2C_TO_SW_ADDR_1;

	/* Selec SCL and SDA mode to respective pins */
	PINSEL_I2C_TO_SW_ADDR_1_SDA |= (PINSEL_I2C_TO_SW_ADDR_1_SDA_VALUE << PINSEL_I2C_TO_SW_ADDR_1_SDA_OFFSET);
	PINSEL_I2C_TO_SW_ADDR_1_SCL |= (PINSEL_I2C_TO_SW_ADDR_1_SCL_VALUE << PINSEL_I2C_TO_SW_ADDR_1_SCL_OFFSET);

	/* Configure I²C interface */
	i2c_control.bits.assert_ack = 1;
	CHK(hw_set_i2c_control(I2C_IF_TO_SWITCH_ADDR_1, i2c_control.reg));
	CHK(hw_set_i2c_address(I2C_IF_TO_SWITCH_ADDR_1, 0));

	// TODO
//	I20SCLH = 100; /* Tempo alto do SCL */
//	I20SCLL = 100; /* Tempo baixo do SCL */

#endif
	return 0;
}

#if 0
void escreveI2C0(char dado, int endereco)
{
#ifndef PC_COMPILATION
	int fim = 0;

	I20CONSET = START;
	//lcd_dado('1');
	while(fim != 2)
	{
		switch(I20STAT)
		{
			case ENVIOU_START:
				//lcd_dado('2'); 
				I20DAT = SMARTCARD_ESCRITA;
				I20CONSET = ACK;
				I20CONCLR = START_FLAG;
				I20CONCLR = INT_FLAG; 
				break;
		
			case ENVIOU_ENDERECO_ESCRITA:
				//lcd_dado('3');
				I20DAT = endereco;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				break;
		
			case FIM_ESCRITA:
				if(fim == 0)
				{
					I20DAT = dado;
					I20CONSET = ACK;
					I20CONCLR = INT_FLAG;
					endereco++;
					fim = 1;
					//lcd_dado('4');
				}
				else{
					//lcd_dado('5');
					fim = 2;
				}
				break;
		}
	}
	I20CONSET = ACK;
	I20CONSET = STOP;
	I20CONCLR = INT_FLAG;
	//lcd_dado('6');
#endif
}




int leI2C0(int endereco)
{
#ifndef PC_COMPILATION
	int leitura = 0;
	int dado;

	I20CONSET = START;
	
	while(I20STAT != FIM_LEITURA)
	{		
		switch(I20STAT)
		{
			case ENVIOU_START: 
				if(leitura == 0) //se eh a primeira vez que envia start
				{
					I20DAT = SMARTCARD_ESCRITA;
					leitura = 1;
				}
				else
					I20DAT = SMARTCARD_LEITURA;
	
				I20CONSET = ACK;
				I20CONCLR = START_FLAG;
				I20CONCLR = INT_FLAG; 
				break;
		
			case ENVIOU_ENDERECO_ESCRITA:
				I20DAT = endereco;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				break;
		
			case ENVIOU_ENDERECO_MEMORIA:
				I20CONSET = STOP;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				I20CONSET = START;
				break;
	
			case RECEBEU_ACK:
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				break;
	
			case ENVIOU_ENDERECO_LEITURA:
				dado = I20DAT;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				I20CONCLR = NACK;
				break;
		}
	}
	I20CONSET = ACK;
	I20CONSET = STOP;
	I20CONCLR = INT_FLAG;
	return dado;
#endif
}

#endif
