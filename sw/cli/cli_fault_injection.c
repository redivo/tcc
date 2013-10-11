#include <stdlib.h>
#include "cli.h"
#include "errors.h"
#include "cfg_i2c.h"
#include "cfg_general.h"

/******************************************************************************/

int cli_fault_injection_en(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char en_value_str[MAX_ARG_SIZE];
	int fault_num;
	int en_value;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get enable value */
	CHK(cli_get_word(line, en_value_str, sizeof(en_value_str), 3));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	en_value = atoi(en_value_str);

	CHK(cfg_set_fault_injection_enable(fault_num, en_value));

	return 0;
}

/******************************************************************************/

int cli_fault_injection_dev_addr(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char dev_addr_str[MAX_ARG_SIZE];
	int fault_num;
	int dev_addr;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get device address */
	CHK(cli_get_word(line, dev_addr_str, sizeof(dev_addr_str), 3));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	dev_addr = (int) strtol(dev_addr_str, (char **) &dev_addr_str, 16);


	CHK(cfg_set_fault_injection_dev_address(fault_num, (unsigned char) dev_addr));

	return 0;
}

/******************************************************************************/

int cli_fault_injection_gen_read_en(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char en_value_str[MAX_ARG_SIZE];
	int fault_num;
	int en_value;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get enable value */
	CHK(cli_get_word(line, en_value_str, sizeof(en_value_str), 3));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	en_value = atoi(en_value_str);

	CHK(cfg_set_fault_injection_read_enable(fault_num, en_value));

	return 0;

	return 0;
}

/******************************************************************************/

int cli_fault_injection_gen_write_en(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char en_value_str[MAX_ARG_SIZE];
	int fault_num;
	int en_value;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get enable value */
	CHK(cli_get_word(line, en_value_str, sizeof(en_value_str), 3));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	en_value = atoi(en_value_str);

	CHK(cfg_set_fault_injection_write_enable(fault_num, en_value));

	return 0;
}

/******************************************************************************/

int cli_cust_fault_reg_addr(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char cust_fault_num_str[MAX_ARG_SIZE];
	char reg_addr_str[MAX_ARG_SIZE];
	int fault_num;
	int cust_fault_num;
	int reg_addr;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get custom fault number */
	CHK(cli_get_word(line, cust_fault_num_str, sizeof(cust_fault_num_str), 3));

	/* Register address to be controlled */
	CHK(cli_get_word(line, reg_addr_str, sizeof(reg_addr_str), 4));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	cust_fault_num = atoi(cust_fault_num_str) - 1;
	reg_addr = (int) strtol(reg_addr_str, (char **) &reg_addr_str, 16);

	CHK(cfg_set_fi_cust_fault_reg_addr(fault_num, cust_fault_num, reg_addr));

	return 0;
}

/******************************************************************************/

int cli_cust_fault_read_en(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char cust_fault_num_str[MAX_ARG_SIZE];
	char en_value_str[MAX_ARG_SIZE];
	int fault_num;
	int cust_fault_num;
	int en_value;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get custom fault number */
	CHK(cli_get_word(line, cust_fault_num_str, sizeof(cust_fault_num_str), 3));

	/* Get enable value */
	CHK(cli_get_word(line, en_value_str, sizeof(en_value_str), 4));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	cust_fault_num = atoi(cust_fault_num_str) - 1;
	en_value = !!atoi(en_value_str);

	CHK(cfg_set_fi_cust_fault_read_enable(fault_num, cust_fault_num, en_value));

	return 0;
}

/******************************************************************************/

int cli_cust_fault_write_en(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char cust_fault_num_str[MAX_ARG_SIZE];
	char en_value_str[MAX_ARG_SIZE];
	int fault_num;
	int cust_fault_num;
	int en_value;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get custom fault number */
	CHK(cli_get_word(line, cust_fault_num_str, sizeof(cust_fault_num_str), 3));

	/* Get enable value */
	CHK(cli_get_word(line, en_value_str, sizeof(en_value_str), 4));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	cust_fault_num = atoi(cust_fault_num_str) - 1;
	en_value = !!atoi(en_value_str);

	CHK(cfg_set_fi_cust_fault_write_enable(fault_num, cust_fault_num, en_value));

	return 0;
}

/******************************************************************************/

int cli_cust_force_resp(char *line, int num_of_args)
{
	char fault_num_str[MAX_ARG_SIZE];
	char cust_fault_num_str[MAX_ARG_SIZE];
	char response_str[MAX_ARG_SIZE];
	int fault_num;
	int cust_fault_num;
	int response;

	/* Get number of fault injection */
	CHK(cli_get_word(line, fault_num_str, sizeof(fault_num_str), 2));

	/* Get enable value */
	CHK(cli_get_word(line, cust_fault_num_str, sizeof(cust_fault_num_str), 3));

	/* Register address to be controlled */
	CHK(cli_get_word(line, response_str, sizeof(response_str), 4));

	/* Convert strings to int */
	fault_num = atoi(fault_num_str) - 1;
	cust_fault_num = atoi(cust_fault_num_str) - 1;

	/* Check if the user set non forced response */
	if (strcmp(response_str, "no") == 0)
		response = NO_FORCE_RESPONSE;
	else
		response = (int) strtol(response_str, (char **) &response_str, 16);

	CHK(cfg_set_fi_cust_fault_forced_response(fault_num, cust_fault_num, response));

	return 0;
}
