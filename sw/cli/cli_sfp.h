#ifndef __CLI_SFP_H
#define __CLI_SFP_H

/******************************************************************************/
/**
 * \brief	SFP insertion CLI function
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_insert_sfp(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	SFP removing CLI function
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_remove_sfp(char *line, int num_of_args);

/******************************************************************************/

#endif /* __CLI_SFP_H */

