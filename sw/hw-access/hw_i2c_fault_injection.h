#ifndef __HW_I2C_FAULT_INJECTION_H
#define __HW_I2C_FAULT_INJECTION_H

#include "cfg_general.h"

/******************************************************************************/

#define MAX_SFP_DATA_BYTE	256
/* Global variables*/
int Sfp_mem_dev0[MAX_SFPS][MAX_SFP_DATA_BYTE];
int Sfp_mem_dev1[MAX_SFPS][MAX_SFP_DATA_BYTE];

/******************************************************************************/
/**
 * \brief	IRQ handler for I²C slave 1.
 */
void hw_handle_irq_i2c_slv1(void) __attribute__ ((interrupt("IRQ")));

/******************************************************************************/
/**
 * \brief	IRQ handler for I²C slave 2.
 */
void hw_handle_irq_i2c_slv2(void) __attribute__ ((interrupt("IRQ")));

/******************************************************************************/

#endif /* __HW_I2C_FAULT_INJECTION_H */

