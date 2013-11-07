#ifndef __CLI_H
#define __CLI_H

#include <stdio.h>

/******************************************************************************/

/* Defines */
#define MAX_CMD_SIZE		100
#define STR_SFP_RANGE		"[1-2]"
#define STR_CUST_I2C_RANGE	"[1-5]"
#define MAX_ARGS			5
#define MAX_ARG_SIZE		20

/******************************************************************************/

/**
 * \typedef command_t
 * \brief This typedef is a structure which saves the command trees
 */
//typedef struct {
//	char *cmd;			//!< Command string.
//	char *help;			//!< Help string.
//	void *next_menu;	//!< Next menu. NULL if there is no next menu.
//	void *callback;		//!< Callback function. NULL if it's not an end function.
//} command_t;

/******************************************************************************/
/**
 * \brief			Get a command line from serial terminal.
 * \return 			0 if OK, error code otherwise
 */
int cli_get_cmd(void);

/******************************************************************************/
/**
 * \brief			Handle the command line that are in global variable
 * \return 			0 if OK, error code otherwise
 */
int cli_handle_cmd(void);

/******************************************************************************/
/**
 * \brief	Get the word of the specified order
 * \param input		Input sentence to be analyzed
 * \param word		Pointer to store the found word
 * \param word_size	Number (in bytes) of word pointer
 * \param order		Order number of word to be got
 * \return 			0 if OK, error code otherwise
 */
int cli_get_word(char *input, char *word, int word_size, int order);


/******************************************************************************/
/**
 * \brief			Set I2C configuration
 * \return 			0 if OK, error code otherwise
 */
//int cli_i2c_set(void);

/******************************************************************************/
/**
 * \brief			Show configuration
 * \return 			0 if OK, error code otherwise
 */
//int cli_show_cfg(void);

/******************************************************************************/

//command_t Cmd_show[] = {
//	{"configuration", "Show current configuration", NULL, &cli_show_cfg},
//};

/******************************************************************************/

//command_t Cmd_sfp_number_en_dis[] = {
//	{"disable",	"Disable I2C access",	NULL, &cli_i2c_set},
//	{"enable",	"Enable I2C access",	NULL, &cli_i2c_set},
//};

/******************************************************************************/

//command_t Cmd_sfp_number_custom_i2c_number_force_response[] = {
//	{"[0x00 - 0xFF]", "Forced response of an I2C access", NULL, &cli_i2c_set},
//};

/******************************************************************************/

//command_t Cmd_sfp_number_custom_i2c_number_addr[] = {
//	{"[0x00 - 0xFF]", "I2C address", NULL, &cli_i2c_set},
//};

/******************************************************************************/

//command_t Cmd_sfp_number_custom_i2c[] = {
//	{STR_CUST_I2C_RANGE, "Number of customized I2C address", NULL, &cli_i2c_set},
//};

/******************************************************************************/

//command_t Cmd_sfp_number_read_write[] = {
//	{"read",	"Reading I2C access",	Cmd_sfp_number_en_dis, NULL},
//	{"write",	"Write I2C access",		Cmd_sfp_number_en_dis, NULL},
//};

/******************************************************************************/

//command_t Cmd_sfp_number[] = {
//	{"all-i2c",		"All I2Cs addresses",		Cmd_sfp_number_read_write, NULL},
//	{"custom-i2c",	"Customize I2C addresses",	Cmd_sfp_number_custom_i2c_number, NULL},
//	{"phy-i2c",		"Internal PHY I2C address",	Cmd_sfp_number_read_write, NULL},
//};

/******************************************************************************/

//command_t Cmd_sfp[] = {
//	{STR_SFP_RANGE, "Number of SFP to be configured", Cmd_sfp_number, NULL},
//};

/******************************************************************************/

//command_t Cmd_root[] = {
//	{"no",		"Remove configurations",	Cmd_root,	NULL},
//	{"sfp",		"SFPs configuration",		Cmd_sfp,	NULL},
//	{"show",	"Show informations",		Cmd_show,	NULL},
//};

/******************************************************************************/

#endif /* __CLI_H */

