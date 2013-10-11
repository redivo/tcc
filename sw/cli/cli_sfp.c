#include <stdlib.h>
#include "cli.h"
#include "errors.h"
#include "cfg_control.h"

/******************************************************************************/

int cli_insert_sfp(char *line, int num_of_args)
{
	int sfp;
	char sfp_str[2];

	/* Get the number of SFP (second word of command line) */
	CHK(cli_get_word(line, sfp_str, sizeof(sfp_str), 2));

	/* In software, SFP's index starts in 0, in CLI it stats in 1, so equalize it */
	sfp = atoi(sfp_str) - 1;

	CHK(cfg_insert_sfp(sfp));

	return 0;
}

/******************************************************************************/

int cli_remove_sfp(char *line, int num_of_args)
{
	int sfp;
	char sfp_str[2];

	/* Get the number of SFP (second word of command line) */
	CHK(cli_get_word(line, sfp_str, sizeof(sfp_str), 2));

	/* In software, SFP's index starts in 0, in CLI it stats in 1, so equalize it */
	sfp = atoi(sfp_str) - 1;

	CHK(cfg_remove_sfp(sfp));

	return 0;
}

/******************************************************************************/

