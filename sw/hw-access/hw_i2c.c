#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif
#include <string.h>
#include "errors.h"
#include "hw_vic_cpsr.h"

#define START 0x20
#define STOP  0x10
#define ACK 0x04
#define NACK  0x04
#define START_FLAG 0x20
#define INT_FLAG 0x08
#define SMARTCARD_LEITURA 0xA1
#define SMARTCARD_ESCRITA 0xA0

/* Boar clock frequency in kHz TODO */
#define BOARD_CLOCK_FREQ	12000

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
#define PINSEL_I2C_TO_SFP_SDA_OFFSET		22		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SFP_SCL_OFFSET		24		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SFP_SDA_VALUE			1		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SFP_SCL_VALUE			1		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SDA			PINSEL1
#define PINSEL_I2C_TO_SW_ADDR_0_SCL			PINSEL1
#define PINSEL_I2C_TO_SW_ADDR_0_SDA_OFFSET	6		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SCL_OFFSET	8		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SDA_VALUE	3		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_0_SCL_VALUE	3		// See Table 107 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SDA			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_1_SCL			PINSEL0
#define PINSEL_I2C_TO_SW_ADDR_1_SDA_OFFSET	20		// See Table 106 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SCL_OFFSET	22		// See Table 106 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SDA_VALUE	2		// See Table 106 from LPC23xx's datasheet
#define PINSEL_I2C_TO_SW_ADDR_1_SCL_VALUE	2		// See Table 106 from LPC23xx's datasheet

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

void hw_handle_irq_i2c_slv1(void) __attribute__ ((interrupt("IRQ")));
void hw_handle_irq_i2c_slv2(void) __attribute__ ((interrupt("IRQ")));

void hw_handle_irq_i2c_slv1(void)
{
#ifndef PC_COMPILATION
	I22CONCLR = (1 << 3);
	pprintf("===== IRQ RECEIVED 1 =====\r\n");
	pprintf(" I21STAT      0x%x\r\n", I21STAT);
	pprintf(" I22STAT      0x%x\r\n", I22STAT);
	pprintf(" I21CONSET    0x%x\r\n", I21CONSET);
	pprintf(" I22CONSET    0x%x\r\n", I22CONSET);
	pprintf(" VICIRQStatus 0x%x\r\n", VICIRQStatus);
	pprintf(" VICIntSelect 0x%x\r\n", VICIntSelect);
	pprintf(" VICIntEnable 0x%x\r\n", VICIntEnable);
#endif
}

int temporario_inicializa_i2cs_slaves(void);

void hw_handle_irq_i2c_slv2(void)
{
#ifndef PC_COMPILATION
	VICVectAddr = 0;

//	pprintf("\r\n===== IRQ RECEIVED 2 =====\r\n");
//	pprintf(" VICIRQStatus 0x%x\r\n", VICIRQStatus);
//	pprintf(" VICIntSelect 0x%x\r\n", VICIntSelect);
//	pprintf(" VICIntEnable 0x%x\r\n", VICIntEnable);
//	pprintf(" I2C 2 status:  0x%x\r\n", I22STAT);
//	pprintf(" I2C 2 data:    0x%x\r\n", I22DAT);
//	pprintf(" I2C 2 control: 0x%x\r\n", I22CONSET);

	switch (I22STAT) {
		case 0x60:
//			pprintf("0x60, escrita requisitada no meu endereço.\r\n");
//			pprintf("1 ");
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
			break;

		case 0x80:
//			pprintf("0x80, received data\r\n");
//			pprintf("2 ");
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
			break;

		case 0xA0:
//			pprintf("3 ");
//			pprintf("0xA0, stop bit.\r\n");
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
			break;

		case 0xA8:
//			pprintf("0xA8, enviando dado.\r\n");
			I22DAT = 0x65;
//			pprintf("4 ");
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
			break;

		case 0xB8:
//			pprintf("5 ");
//			pprintf("0xB8, dado recebido pelo master.\r\n");
			I22DAT = 0x65;
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
			break;

		case 0xC0:
//			pprintf("0xC0, fim da transmição NACK reebido.\r\n");
//			pprintf("6 ");
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
			break;		

		default:
			pprintf("7 0x%x", I22STAT);
//			pprintf("I dont know\r\n");
			I22CONSET = ACK;
			I22CONCLR = INT_FLAG;
//			pprintf(" next I2C 2 status:  0x%x\r\n", I22STAT);
//			pprintf(" next I2C 2 data:    0x%x\r\n", I22DAT);
//			pprintf(" next I2C 2 control: 0x%x\r\n", I22CONSET);
			break;
	}
#endif
}

int temporario_inicializa_i2cs_slaves(void)
{
#ifndef PC_COMPILATION
	/* Turn on power */
	PCONP |= (1 << 19); // I2C1
	PCONP |= (1 << 26); // I2C2

	/* Selec SCL and SDA mode to respective pins */
	PINSEL1 |= (3 << 6);  // SDA I2C1
	PINSEL1 |= (3 << 8);  // SCL I2C1
	PINSEL0 |= (2 << 20); // SDA I2C2
	PINSEL0 |= (2 << 22); // SCL I2C2

	/* Configure I²C interface */
	I21CONCLR = 0xff;
	I22CONCLR = 0xff;
	I21CONSET = (1 << 2) | (1 << 6);
	I22CONSET = (1 << 2) | (1 << 6);
	I21ADR = 0x50 << 1;
	I22ADR = 0x51 << 1;


	IRQdisable();

	VICIntSelect &= ~(1 << 19);		/* i2c1=bit 19 como IRQ	*/
	VICIntSelect &= ~(1 << 30);		/* i2c2=bit 30 como IRQ	*/

	VICIntEnable |= (1 << 19);		/* Habilita int do i2c1 no VIC*/
	VICIntEnable |= (1 << 30);		/* Habilita int do i2c2 no VIC*/

	VICVectAddr19 = (int)hw_handle_irq_i2c_slv1;	/* Vetor para atendimento do I2C2 */
	VICVectAddr30 = (int)hw_handle_irq_i2c_slv2;	/* Vetor para atendimento do I2C2 */

	IRQenable();

	pprintf("===== IRQ STATUS 2 =====\r\n");
	pprintf(" I22STAT      0x%x\r\n", I22STAT);
	pprintf(" I22CONSET    0x%x\r\n", I22CONSET);
	pprintf(" VICIRQStatus 0x%x\r\n", VICIRQStatus);
	pprintf(" VICIntSelect 0x%x\r\n", VICIntSelect);
	pprintf(" VICIntEnable 0x%x\r\n", VICIntEnable);

#endif
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

#if 0
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
	VICVectAddr9 = (int)hw_handle_irq_i2c_slv2;	/* Vetor para atendimento do I2C2 */
	IRQenable();
#else
	temporario_inicializa_i2cs_slaves();
#endif

	// TODO
	I20SCLH = 100; /* Tempo alto do SCL */
	I20SCLL = 100; /* Tempo baixo do SCL */
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
			case SEND_START_BIT:
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


#endif

#define SEND_START_BIT			0x08
#define END_OF_RECEIVE			0x58
#define ENVIOU_ENDERECO_ESCRITA 0x18
#define ENVIOU_ENDERECO_LEITURA	0x50
#define ENVIOU_ENDERECO_MEMORIA 0x28
#define RECEBEU_ACK				0x40
#define FIM_ESCRITA				0x28
#define FIM_LEITURA				0x58


int hw_i2c_read(int dev_addr, int reg_addr)
{
#ifndef PC_COMPILATION
	int leitura = 0;
	int dado;
	int sts0 = 0, sts1 = 0, sts2 = 0;

	I20CONSET = START;

	while (I20STAT != END_OF_RECEIVE)
	{
		if (sts0 != I20STAT) {
			sts0 = I20STAT;
			pprintf("I2C 0 Status: 0x%x\r\n", sts0);
		}

		if (sts1 != I21STAT) {
			sts1 = I21STAT;
			pprintf("I2C 1 Status: 0x%x\r\n", sts1);
		}

		if (sts2 != I22STAT) {
			sts2 = I22STAT;
			pprintf("I2C 2 Status: 0x%x\r\n", sts2);
		}

		switch (I20STAT)
		{
			case SEND_START_BIT:
				pprintf("line %d\r\n", __LINE__);
				if (leitura == 0) //se eh a primeira vez que envia start
				{
					pprintf("line %d\r\n", __LINE__);
					I20DAT = (dev_addr << 1);
					leitura = 1;
				}
				else {
					pprintf("line %d\r\n", __LINE__);
					I20DAT = (dev_addr << 1) + 1;
				}

				pprintf("line %d\r\n", __LINE__);
				I20CONSET = ACK;
				I20CONCLR = START_FLAG;
				I20CONCLR = INT_FLAG;
				break;

			case ENVIOU_ENDERECO_ESCRITA:
				pprintf("line %d\r\n", __LINE__);
				I20DAT = reg_addr;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				break;

			case ENVIOU_ENDERECO_MEMORIA:
				pprintf("line %d\r\n", __LINE__);
				I20CONSET = STOP;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				I20CONSET = START;
				break;

			case RECEBEU_ACK:
				pprintf("line %d\r\n", __LINE__);
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				break;

			case ENVIOU_ENDERECO_LEITURA:
				pprintf("line %d\r\n", __LINE__);
				dado = I20DAT;
				I20CONSET = ACK;
				I20CONCLR = INT_FLAG;
				I20CONCLR = NACK;
				break;
		}
	}

	pprintf("line %d\r\n", __LINE__);
	I20CONSET = ACK;
	I20CONSET = STOP;
	I20CONCLR = INT_FLAG;
	return dado;
#endif
}

