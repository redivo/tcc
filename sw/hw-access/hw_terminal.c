#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif

#include <stdarg.h>
#include <stdbool.h>
#include "hw_terminal.h"
#include "errors.h"
#include "hw_uart.h"
#include "hw_mprintf.h"

bool Terminal_init_done = 0;

/******************************************************************************/

#ifndef PC_COMPILATION
int pprintf(const char *formato, ... )
{
	va_list va;

	if (!term_init_is_done())
		return ERR_RESOURCE_UNAV;

	va_start(va,formato);

	return va_printf(U0putchar, formato, va);
}
#endif

/******************************************************************************/

#ifndef PC_COMPILATION
char ggetc(void)
{
	return U0getchar();
}
#endif

/******************************************************************************/

int term_init(void)
{
#ifndef PC_COMPILATION
	U0init();
	Terminal_init_done = 1;

#endif
	return 0;
}

/******************************************************************************/

bool term_init_is_done(void)
{
#ifndef PC_COMPILATION
	return Terminal_init_done;
#else
	return 1;
#endif
}

/******************************************************************************/

bool term_rx_data_ready(void)
{
#ifndef PC_COMPILATION
	uart_lsr_t lsr;

	lsr.reg = U0LSR;

	return lsr.bits.rcv_data_rdy;
#else
	return 0;
#endif
}

/******************************************************************************/

