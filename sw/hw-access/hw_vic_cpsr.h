/************************************************************
Arquivo de cabecalho para habilitar interrupcoes no CPSR
(Current Processor Status Register)
************************************************************/
#ifndef __HW_VIC_CPSR_H
/* Declaracoes especiais para rotinas de atendimento a interrupcoes*/
void SWI_Routine(void) __attribute__ ((interrupt("SWI")));
void IRQ_Routine(void) __attribute__ ((interrupt("IRQ")));
void FIQ_Routine(void) __attribute__ ((interrupt("FIQ")));

/* Macros para habilitar/desabilitar IRQ/FIQ no CPSR */
#define FIQdisable() asm volatile(\
	"mrs r3, cpsr\n" \
	"bic r3, r3, # 0x40\n" \
	"msr cpsr, r3\n")

#define FIQdisable() asm volatile(\
	"mrs r3, cpsr\n" \
	"orr r3, r3, # 0x40\n" \
	"msr cpsr, r3\n")

#define IRQenable() asm volatile(\
	"mrs r3, cpsr\n" \
	"bic r3, r3, # 0x80\n" \
	"msr cpsr, r3\n")

#define IRQdisable() asm volatile(\
	"mrs r3, cpsr\n" \
	"orr r3, r3, # 0x80\n" \
	"msr cpsr, r3\n")

#endif /* __HW_VIC_CPSR_H */

