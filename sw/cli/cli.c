#include <string.h>
#include <stdbool.h>
#include "cli.h"
#include "errors.h"
#include "hw_uart.h"
#include "cfg_general.h"
#include "cli_sfp.h"
#include "cli_fault_injection.h"
#include "cli_debug.h"
#include "debug.h"

/******************************************************************************/
/* Commands sizes defines */
#define COMMAND_MAX_SIZE 25
#define HELP_MAX_SIZE 150

/* Global variables */
char Cmd_line[MAX_CMD_SIZE];

/******************************************************************************/
/**
 * \typedef command_t
 * \brief This typedef is a structure which saves the commands details
 */
typedef struct {
	char command[COMMAND_MAX_SIZE];					//!< Command, in literal
	char help[HELP_MAX_SIZE];						//!< Help to be shown
	unsigned char argumment_number;					//!< Number of arguments of a command
	int (*callback)(char *line, int num_of_args);	//!< Callback to be called on command call
} command_t;

/******************************************************************************/
/**
 * \brief	Help CLI function
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
static int cli_print_help(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Show current configuration CLI function
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
static int cli_show_cfg(char *line, int num_of_args);

/******************************************************************************/

/* List of commands TODO */
command_t Commands[] = {
	{
		.command = "?",
		.help = "Show help.",
		.argumment_number = 0,
		.callback = cli_print_help,
	},
	{
		.command = "show-cfg",
		.help = "Show the current configuration.",
		.argumment_number = 0,
		.callback = cli_show_cfg,
	},
	{
		.command = "insert-sfp",
		.help = "{SFP#} Do the SFP inserion.",
		.argumment_number = 1,
		.callback = cli_insert_sfp,
	},
	{
		.command = "remove-sfp",
		.help = "{SFP#} Do the SFP remove.",
		.argumment_number = 1,
		.callback = cli_remove_sfp,
	},
	{
		.command = "fault-injection-en",
		.help = "{FAULT#} {EN-VALUE} Enable a fault injection.",
		.argumment_number = 2,
		.callback = cli_fault_injection_en,
	},
	{
		.command = "fault-injection-addr",
		.help = "{FAULT#} {HEX-DEV-ADDR} Set a I2C device address to be listened.",
		.argumment_number = 2,
		.callback = cli_fault_injection_dev_addr,
	},
	{
		.command = "fault-injection-read",
		.help = "{FAULT#} {READ-EN-VALUE} Set the general read to enable or disabled.",
		.argumment_number = 2,
		.callback = cli_fault_injection_gen_read_en,
	},
	{
		.command = "fault-injection-write",
		.help = "{FAULT#} {WRITE-EN-VALUE} Set the general write to enable or disabled.",
		.argumment_number = 2,
		.callback = cli_fault_injection_gen_write_en,
	},
	{
		.command = "custom-fault-regaddr",
		.help = "{FAULT#} {CUST-FAULT#} {HEX-REG-ADDR} Set the register address to be faulted.",
		.argumment_number = 3,
		.callback = cli_cust_fault_reg_addr,
	},
	{
		.command = "custom-fault-read",
		.help = "{FAULT#} {CUST-FAULT#} {READ-EN-VALUE} Set the custom fault read to enable or disabled.",
		.argumment_number = 3,
		.callback = cli_cust_fault_read_en,
	},
	{
		.command = "custom-fault-write",
		.help = "{FAULT#} {CUST-FAULT#} {WRITE-EN-VALUE} Set the custom fault WRITE to enable or disabled.",
		.argumment_number = 3,
		.callback = cli_cust_fault_write_en,
	},
	{
		.command = "custom-fault-force-resp",
		.help = "{FAULT#} {CUST-FAULT#} {HEX-RESPONSE} Set a forced response to this custom fault. 'no' to don't force it.",
		.argumment_number = 3,
		.callback = cli_cust_force_resp,
	},
#ifdef DEBUG_MODE
	{
		.command = "debug-r",
		.help = "{REGISTER} Print register value.",
		.argumment_number = 1,
		.callback = cli_debug_read,
	},
	{
		.command = "debug-w",
		.help = "{REGISTER} {HEX-VALUE} Write in a register and print register value.",
		.argumment_number = 2,
		.callback = cli_debug_write,
	},
#endif /* DEBUG_MODE */
};

/******************************************************************************/

static int cli_print_help(char *line, int num_of_args)
{
	int number_of_commands;
	int i;

	number_of_commands = sizeof(Commands) / sizeof(command_t);

	pprintf("\r\n");
	for (i = 0; i < number_of_commands; i++)
		pprintf(" %-25s %s\r\n", Commands[i].command, Commands[i].help);

	pprintf("\r\n");

	return 0;
}

/******************************************************************************/

static int cli_show_cfg(char *line, int num_of_args)
{
	cfg_show();

	return 0;
}

/******************************************************************************/

int cli_get_cmd(void)
{
	int char_index = 0;
	char cmd_line[MAX_CMD_SIZE];
	char c;

	if (!term_init_is_done())
		return ERR_RESOURCE_UNAV;

	pprintf(TERMINAL_STR);

	/* Erase command line */
	strcpy(cmd_line, "");

	/* Get string, character by character, and handle it based on commands */
	do {
		c = ggetc();

		if (c == KEY_RETURN)
			break;

		if (char_index >= MAX_CMD_SIZE - 1)
			continue;

		if (c != KEY_RETURN && c!= KEY_NEW_LINE) {
			cmd_line[char_index] = c;
			cmd_line[char_index + 1] = '\0';
			char_index++;
		}

		/* Terminal of PCs prints it automatically */
#ifndef PC_COMPILATION
		pprintf("%c", c);
#endif
	} while (c != KEY_RETURN && c!= KEY_NEW_LINE);

	//TODO fazer com \0

	/* Copy wrote command to global variable command */
	sprintf(Cmd_line, "%s", cmd_line);
#ifndef PC_COMPILATION
	pprintf("\r\n");
#endif

	return 0;
}

/******************************************************************************/
/**
 * \brief	Count the number of words blank space separated in a string.
 * \param line	Pointer to string to be analyzed.
 * \return		number of words or negative error code.
 */
static int cli_get_number_of_words(char *line)
{
	int i;
	int words = 0;
	int new_word = 1;

	/* Guarantee it's a valid string */
	if (line == NULL)
		return ERR_INVALID_PARAM;

	for (i = 0; line[i]; i++) {
		/* Blank spaces are not words */
		if (line[i] == ' ') {
			new_word = 1;
			continue;
		}

		/* If it's not a continue of word */
		if (new_word)
			words++;

		new_word = 0;
	}

	return words;
}

/******************************************************************************/
/**
 * \brief	Get the main command of a comman line
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
static int cli_get_command(char *input, char *command)
{
	if (input == NULL || command == NULL)
		return ERR_INVALID_PARAM;

	cli_get_word(input, command, COMMAND_MAX_SIZE, 1);

	return 0;
}

/******************************************************************************/

int cli_get_word(char *input, char *word, int word_size, int order)
{
	int i;
	int j = 0;
	int found_words = 0;
	int first_char = 1;

	if (input == NULL || word == NULL)
		return ERR_INVALID_PARAM;

	/* Clear string */
	sprintf(word, "");

	for (i = 0; input[i]; i++) {
		/* If j is more than mad word size get out */
		if (j >= word_size)
			break;

		/* Expect a word. Blank spaces separate words */
		if (input[i] == ' ') {
			first_char = 1;
			continue;
		}

		/* If end of string, get out for loop */
		if (input[i] == '\0')
			break;

		/* If it's not anyone of the before cases, it's a part of a word */

		/* If it's the first char of a word, increase the words counter */
		if (first_char) {
			found_words++;
			first_char = 0;
		}

		/* If it's not found word, check again */
		if (found_words != order)
			continue;

		/* If it's not anyone of the before cases, it's the found word, do copy it */
		word[j] = input[i];
		j++;
	}

	/* Sen end of word */
	word[j] = '\0';

	return 0;
}

/******************************************************************************/

int cli_handle_cmd(void)
{
	int number_of_commands;
	int number_of_argumments;
	char command[COMMAND_MAX_SIZE];
	int found_command = 0;
	int i;

	number_of_commands = sizeof(Commands) / sizeof(command_t);
	number_of_argumments = cli_get_number_of_words(Cmd_line) - 1;
	cli_get_command(Cmd_line, command);

	/* If there are no words, do nothing */
	if (cli_get_number_of_words(Cmd_line) == 0)
		return 0;

	for (i = 0; i < number_of_commands; i++) {
		/* If it's not the command */
		if (strcmp(command, Commands[i].command) != 0)
			continue;

		/* If the argument number does not match */
		if (number_of_argumments != Commands[i].argumment_number)
			continue;

		CHK(Commands[i].callback(Cmd_line, number_of_argumments));
		found_command = 1;
	}

	if (!found_command)
		pprintf(" Unknown command\r\n");

	return 0;
}

/******************************************************************************/

