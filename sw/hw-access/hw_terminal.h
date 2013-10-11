#ifndef __TERMINAL_H
#define __TERMINAL_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/******************************************************************************/

/* Defines */
#define CMD_MAX_SIZE	100
#define TERMINAL_STR	"SFP-TESTER# "

/* Key defines */
#define KEY_BACKSPACE	0x7F
//#define KEY_DELETE		0x7F
#define KEY_RETURN		0x0D
#define KEY_NEW_LINE	0x0A
#define KEY_HYPHEN		0x2D
#define KEY_LEFT_ROW	0x11
#define KEY_RIGHT_ROW	0x12
#define KEY_BLANK_SPACE	0x20
#define KEY_TAB			0x09

/******************************************************************************/
/**
 * \typedef uart_lsr_t
 * \brief This typedef is an union that represents UART Line Status Register (LSR)
 */
typedef union {
	struct {
		char rcv_data_rdy : 1;	//!< Received data ready bit
		char overrun_err: 1;	//!< Overrun error bit
		char parity_err : 1;	//!< Parity error bit
		char frame_err : 1;		//!< Framing error bit
		char break_int : 1;		//!< Break interruption bit
		char thre : 1;			//!< Transmitter holding register empty bit
		char transm_empty : 1;	//!< Transmitter empty bit
		char rx_fifo_err : 1;	//!< Error in RX FIFO bit
	} bits;
	char reg;
} uart_lsr_t;

/******************************************************************************/
/**
 * \brief			Print a string in terminal.
 * \param formato	The string format.
 * \return 			0 if OK, error code otherwise.
 */
#ifndef PC_COMPILATION
int pprintf(const char *formato, ... );
#else
#define pprintf printf
#endif

/******************************************************************************/
/**
 * \brief			Get a charactere from serial terminal.
 * \return 			Written character.
 */
#ifndef PC_COMPILATION
char ggetc(void);
#else
#define ggetc getchar
#endif

/******************************************************************************/
/**
 * \brief	Initializer serial and terminal.
 * \return 	0 if OK, error code otherwise
 */
int term_init(void);

/******************************************************************************/
/**
 * \brief	Inform if terminal/serial initialization is done.
 * \return 	1 if it's initialized, 0 if it's not.
 */
bool term_init_is_done(void);

/******************************************************************************/
/**
 * \brief	Inform if there is some TX data ready
 * \return 	1 if data ready, 0 if it's not.
 */
bool term_rx_data_ready(void);

/******************************************************************************/

#endif /* __TERMINAL_H */

