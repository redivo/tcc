#ifndef __CLI_FAULT_INJECTION_H
#define __CLI_FAULT_INJECTION_H

/******************************************************************************/
/**
 * \brief	Enable fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_fault_injection_en(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Set I²C device address to a fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_fault_injection_dev_addr(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Enable or disable general read in a fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_fault_injection_gen_read_en(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Enable or disable general write in a fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_fault_injection_gen_write_en(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Set a register address to a custom fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_cust_fault_reg_addr(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Enable or disable read in a custom fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_cust_fault_read_en(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Enable or disable write in a custom fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_cust_fault_write_en(char *line, int num_of_args);

/******************************************************************************/
/**
 * \brief	Set a forced response for a custom fault injection
 * \param line			Pointer to written line
 * \param num_of_args	Number of typed arguments
 * \return	0 if OK, error code otherwise.
 */
int cli_cust_force_resp(char *line, int num_of_args);

/******************************************************************************/

#endif /* __CLI_FAULT_INJECTION_H */

