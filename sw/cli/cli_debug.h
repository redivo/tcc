#ifndef __CLI_DEBUG_H
#define __CLI_DEBUG_H

/******************************************************************************/
/**
 * \brief	Print a register's value
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_debug_read(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Write a value in a register and print it afetr read
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_debug_write(char *line, int num_of_args);

/******************************************************************************/

#endif /* __CLI_DEBUG_H */

