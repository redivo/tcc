#ifndef __ERRORS_H
#define __ERRORS_H

#include "hw_terminal.h"

/* Defines */
#define ERR_GENERIC				-1
#define ERR_INVALID_PARAM		-2
#define ERR_RESOURCE_UNAV		-3
#define ERR_INVALID_LED			-4
#define ERR_INVALID_UPC_PORT	-5
#define ERR_SFP_COLISION		-6
#define ERR_INVALID_I2C			-7
#define ERR_INVALID_REG			-8

/* Macros */
#define PRINT_ERR(_r) \
	switch(_r) {\
		case ERR_INVALID_PARAM:\
			pprintf("\r\n ERROR [%d] Invalid parameter\r\n\n\n", _r);\
			break;\
		case ERR_RESOURCE_UNAV:\
			pprintf("\r\n ERROR [%d] Unavailable resource\r\n\n\n", _r);\
			break;\
		case ERR_INVALID_LED:\
			pprintf("\r\n ERROR [%d] Invalid led\r\n\n\n", _r);\
			break;\
		case ERR_INVALID_UPC_PORT:\
			pprintf("\r\n ERROR [%d] Invalid uPC port\r\n\n\n", _r);\
			break;\
		case ERR_SFP_COLISION:\
			pprintf("\r\n ERROR [%d] Only one SFP can be inserted at time\r\n\n\n", _r);\
			break;\
		case ERR_INVALID_I2C:\
			pprintf("\r\n ERROR [%d] Invalid I2C interface number\r\n\n\n", _r);\
			break;\
		case ERR_INVALID_REG:\
			pprintf("\r\n ERROR [%d] Invalid register\r\n\n\n", _r);\
			break;\
		case -999:\
			pprintf("\r\n COMMAND [%d] Resetting program\r\n\n\n", _r);\
			return -999;\
		case ERR_GENERIC:\
		default:\
			pprintf("\r\n ERROR [%d] Generic error\r\n\n\n", _r);\
			break;\
	}

#define CHK(_r) do {int _ret; _ret = _r; if (_ret < 0) { return _ret; }} while (0)
#define CHK_PRINT(_r) do {int _ret; _ret = _r; if (_ret < 0) { PRINT_ERR(_ret); return _ret; }} while (0)
#define CHK_PRINT_NO_RET(_r) do {int _ret; _ret = _r; if (_ret < 0) { PRINT_ERR(_ret); }} while (0)

#endif /* __ERRORS_H */

