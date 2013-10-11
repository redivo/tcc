#ifndef PC_COMPILATION
#include <arch/nxp/lpc23xx.h>
#endif
#include "hw_uart.h"

/* Configuracao da Porta Serial 0 (Parecido com o 16550) */
void U0init()
{
#ifndef PC_COMPILATION
PINSEL0 |= 0x50;	/* Seleciona pinos TxD0 e RxD0 */
U0FCR   = 0x7;		/* Habilita as FIFOs e reset */
U0LCR   = 0x83;		/* Habilita acesso ao divisor de baud-rate (DLAB) */
U0DLL   = ((CRYSTALFREQ/BAUDRATE + 8) >> 4) & 0xff;
U0DLM   = ((CRYSTALFREQ/BAUDRATE) >>12) & 0xff;
U0LCR    = 3;		/* Desabilita DLAB */
#endif
}

char U0getchar(void)
{
#ifndef PC_COMPILATION
while(!(U0LSR & 1));
return U0RBR;
#else
	return 0;
#endif
}

void U0putchar(int c)
{
#ifndef PC_COMPILATION
while(!(U0LSR & 0x20));
U0THR = c;
#endif
}

void U0puts(char *s)
{
#ifndef PC_COMPILATION
while(*s) U0putchar(*s++);
#endif
}

/* Entrada de texto */
char *U0gets(char *txt, int nmax)
{
#ifndef PC_COMPILATION
#define BACKSPACE	0x08
#define DELETE		0x7F

	int k;	/* Numero de caracteres armazenados */
	int c;	/* caractere lido */
	k=0;

	while(!(U0LSR & 1));

	do {
		c=U0getchar();

		/* Delete and Backspace work on the same way */
		/* Avoid to count backspace as a character */
		if(c == DELETE)
			c = BACKSPACE;
		else
			/* Avoid user to write more than the informed size */
			if(k >= nmax - 1)
				continue;
			else
				k++;

		/* Avoid to backspace erase things which it does not put in the screen */
		if (c != BACKSPACE || (c == BACKSPACE && k > 0))
			U0putchar(c);

		/* se for backspace retira um caractere do buffer */
		if(c == BACKSPACE && (k > 0)) {
			k--;
			U0putchar(' ');
			U0putchar(BACKSPACE);
		}
		else
			txt[k - 1] = c;
	} while(c!='\n' && c!='\r');

	txt[k]='\0';
	return txt;

#undef BACKSPACE
#undef DELETE
#else
	return 0;
#endif
}
