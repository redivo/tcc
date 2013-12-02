#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif
#include <string.h>
#include "errors.h"
#include "hw_vic_cpsr.h"
#include "hw_general.h"
#include "hw_i2c.h"
#include "hw_i2c_fault_injection.h"
#include "cfg_general.h"

/* I²C interfaces number */
#define I2C_IF_TO_SFP			0 // Interface to be linked on SFP bus. It works like mater.
#define I2C_IF_TO_SWITCH_ADDR_0	1 // Interface to be linked on switch bus. It works like slave.
#define I2C_IF_TO_SWITCH_ADDR_1	2 // Interface to be linked on switch bus. It works like slave.

/* I²C interface power bit in Poer Control for Peripherals (PCONP) register (Table 56 from LPC23xx's datasheet) */
#define PCONP_I2C_TO_SFP		(1 << 7)
#define PCONP_I2C_TO_SW_ADDR_0	(1 << 19)
#define PCONP_I2C_TO_SW_ADDR_1	(1 << 26)

/* I²C interface mode Pin Selection, offsets and values */
#define PINSEL_I2C_TO_SFP_SDA				PINSEL1
#define PINSEL_I2C_TO_SFP_SCL				PINSEL1
#define PINSEL_I2C_TO_SFP_SDA_OFFSET		22		// See Table 108 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SFP_SCL_OFFSET		24		// See Table 108 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SFP_SDA_VALUE			1		// See Table 108 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SFP_SCL_VALUE			1		// See Table 108 from LPC23xx's datasheet

#define PINSEL_I2C_TO_SW_ADDR_0_SDA			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_0_SCL			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_0_SDA_OFFSET	0		// See Table 108 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SCL_OFFSET	2		// See Table 108 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SDA_VALUE	3		// See Table 108 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SCL_VALUE	3		// See Table 108 from LPC23xx's datasheet

//#define PINSEL_I2C_TO_SW_ADDR_0_SDA			PINSEL1
//#define PINSEL_I2C_TO_SW_ADDR_0_SCL			PINSEL1
//#define PINSEL_I2C_TO_SW_ADDR_0_SDA_OFFSET	6		// See Table 108 from LPC23xx's datasheet
//#define PINSEL_I2C_TO_SW_ADDR_0_SCL_OFFSET	8		// See Table 108 from LPC23xx's datasheet
//#define PINSEL_I2C_TO_SW_ADDR_0_SDA_VALUE	3		// See Table 108 from LPC23xx's datasheet
//#define PINSEL_I2C_TO_SW_ADDR_0_SCL_VALUE	3		// See Table 108 from LPC23xx's datasheet

#define PINSEL_I2C_TO_SW_ADDR_1_SDA			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_1_SCL			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_1_SDA_OFFSET	20		// See Table 106 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SCL_OFFSET	22		// See Table 106 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SDA_VALUE	2		// See Table 106 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SCL_VALUE	2		// See Table 106 from LPC23xx's datasheet

/* SFP data memory defines */
#define SFP_DD	92

/******************************************************************************/

int hw_i2c_read(int dev_addr, int reg_addr);

/******************************************************************************/
/**
 * \typedef i2c_control_t
 * \brief This typedef is an union to describe I²C Control register. See item 21.8.1 from LPC23xx's datasheet.
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
			I20ADR = address << 1;
			break;

		case 1:
			I21ADR = address << 1;
			break;

		case 2:
			I22ADR = address << 1;
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

	/* TODO IRQ */
	IRQdisable();
	VICIntSelect &= ~(1 << 19);		/* i2c2=bit 30 como IRQ	*/
	VICIntEnable |= (1 << 19);		/* Habilita int do i2c2 no VIC*/
	VICVectAddr19 = (int)hw_handle_irq_i2c_slv1;	/* Vetor para atendimento do I2C2 */


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
	i2c_control.bits.enable = 1;
	i2c_control.bits.assert_ack = 1;
	CHK(hw_set_i2c_control(I2C_IF_TO_SWITCH_ADDR_1, i2c_control.reg));
	CHK(hw_set_i2c_address(I2C_IF_TO_SWITCH_ADDR_1, 0));

	/* TODO IRQ */
	VICIntSelect &= ~(1 << 30);		/* i2c2=bit 30 como IRQ	*/
	VICIntEnable |= (1 << 30);		/* Habilita int do i2c2 no VIC*/
	VICVectAddr30 = (int)hw_handle_irq_i2c_slv2;	/* Vetor para atendimento do I2C2 */
	IRQenable();

	// TODO
	I20SCLH = 100; /* Tempo alto do SCL */
	I20SCLL = 100; /* Tempo baixo do SCL */
#endif
	return 0;
}

/******************************************************************************/

int hw_i2c_read(int dev_addr, int reg_addr)
{
#ifndef PC_COMPILATION
	int timeout;
	int leitura = 0;
	int dado;

	timeout = T0TC + 2000;

	I20CONSET = START;

	while (I20STAT != END_OF_RECEIVE)
	{
		switch (I20STAT)
		{
			case SEND_START_BIT:
				if (leitura == 0) //se eh a primeira vez que envia start
				{
					I20DAT = (dev_addr << 1);
					leitura = 1;
				}
				else {
					I20DAT = (dev_addr << 1) + 1;
				}

				I20CONSET = ACK;
				I20CONCLR = START_FLAG;
				I20CONCLR = INT_FLAG;
				break;

			case ENVIOU_ENDERECO_ESCRITA:
				I20DAT = reg_addr;
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

		/* Time out */
		if (T0TC == timeout) {
			pprintf("I2C ERROR!!!\r\n");
			return -1;
		}
	}

	I20CONSET = ACK;
	I20CONSET = STOP;
	I20CONCLR = INT_FLAG;
	return dado;
#endif
}

/******************************************************************************/

int hw_sfp_memory_update(int sfp)
{
#ifndef PC_COMPILATION
	int reg_addr;

	if (!IS_VALID_SFP(sfp))
		return ERR_INVALID_PARAM;

	/* If SFP is not inserted, clean the memory */
	if (!cfg_is_sfp_inserted(sfp)) {
		memset(Sfp_mem_dev0[sfp], 0, sizeof(Sfp_mem_dev0[sfp]));
		memset(Sfp_mem_dev1[sfp], 0, sizeof(Sfp_mem_dev1[sfp]));

		return 0;
	}

	/* Update first device */
	for (reg_addr = 0; reg_addr < MAX_SFP_DATA_BYTE; reg_addr++)
		Sfp_mem_dev0[sfp][reg_addr] = hw_i2c_read(Cfg.i2c_fi[0].dev_address, reg_addr);

	/* If the first device is the basic memory and there is DD in this SFP and the
	 * second device is the DD, update it */
	if (Cfg.i2c_fi[0].dev_address == 0x50
			&& Cfg.i2c_fi[1].dev_address == 0x51
			&& Sfp_mem_dev0[sfp][SFP_DD] != 0) {
		for (reg_addr = 0; reg_addr < MAX_SFP_DATA_BYTE; reg_addr++)
			Sfp_mem_dev1[sfp][reg_addr] = hw_i2c_read(Cfg.i2c_fi[1].dev_address, reg_addr);
	}

#endif

	return 0;
}

/******************************************************************************/

