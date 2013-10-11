#ifndef __CFG_GENERAL_H
#define __CFG_GENERAL_H

#include <stdbool.h>

/******************************************************************************/

/* Defines */
#define NO_FORCE_RESPONSE	-1
#define MAX_CUSTOM_FAULTS	5
#define MAX_LISTENED_ADDR	2
#define MAX_SFPS			2

/* Macros */
#define FOR_EACH_SFP(_s) for (_s = 0; _s < MAX_SFPS; _s++)
#define FOR_EACH_CUSTOM_FAULT(_f) for (_f = 0; _f < MAX_CUSTOM_FAULTS; _f++)
#define FOR_EACH_FAULT_INJECTION(_f) for (_f = 0; _f < MAX_LISTENED_ADDR; _f++)
#define IS_VALID_SFP(_s) (_s >= 0 && _s < MAX_SFPS)
#define IS_VALID_FAULT_INJECTION_INDEX(_f) (_f >= 0 && _f < MAX_LISTENED_ADDR)
#define IS_VALID_DEV_ADDR(_addr) ((_addr >> 7) == 0)
#define IS_VALID_CUST_FAULT(_f) (_f >= 0 && _f < MAX_CUSTOM_FAULTS)

/******************************************************************************/
/**
 * \typedef i2c_cutm_fi_cfg_t
 * \brief This typedef is a structure which saves custom I2C fails
 */
typedef struct {
	unsigned char reg_addr;	//!< Register address to be injected the faults.
	bool read_enable;		//!< Inform if the I2C read is disabled.
	bool write_enable;		//!< Inform if the I2C write is disabled.
	int forced_response;	//!< Forced response to be sent at reads. Use NO_FORCE_RESPONSE to a correct response for reads.
} i2c_cutm_fi_cfg_t;

/******************************************************************************/
/**
 * \typedef sfp_cfg_t
 * \brief This typedef is a structure which saves the SFP error configuration
 */
typedef struct {
	bool enable; //!< SFP is enabled
} sfp_cfg_t;

/******************************************************************************/
/**
 * \typedef i2c_fi_cfg_t
 * \brief This typedef is a structure which saves the I²C Fault Injection Module configuration.
 */
typedef struct {
	bool enable;											//!< SFP is enabled
	unsigned char dev_address;								//!< Device I²C address to be listened
	bool general_read_enable;								//!< Inform if the I2C read is enabled for all registers. Custom faults configuration have more priority than this configuration.
	bool general_write_enable;								//!< Inform if the I2X write is enabled for all registers. Custom faults configuration have more priority than this configuration.
	i2c_cutm_fi_cfg_t i2c_cust_fault[MAX_CUSTOM_FAULTS];	//!< Vector of custom I2C faults
} i2c_fi_cfg_t;

/******************************************************************************/
/**
 * \typedef cfg_t
 * \brief This typedef is a structure which saves the global configuration
 */
typedef struct {
	sfp_cfg_t sfp[MAX_SFPS];
	i2c_fi_cfg_t i2c_fi[MAX_LISTENED_ADDR];
} cfg_t;

/******************************************************************************/

/* Global configuration variable */
cfg_t Cfg;

/******************************************************************************/
/**
 * \brief	Initialize the global configuration with the default configuration
 * \return	0 if OK, error code otherwise.
 */
int cfg_init(void);

/******************************************************************************/
/**
 * \brief	Enable for all I2Cs from an SFP
 * \param sfp		SFP number
 * \param enable	Inform if I2C must be enabled
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_i2c_enable_all(int sfp, bool enable);

/******************************************************************************/
/**
 * \brief	Enable an SFP
 * \param sfp		SFP number
 * \param enable	Inform if I2C must be enabled
 * \return	0 if OK, error code otherwise.
 */
int cfg_set_enable(int sfp, bool enable);

/******************************************************************************/
/**
 * \brief	Show current configuration.
 */
void cfg_show(void);

/******************************************************************************/

#endif /* __CFG_GENERAL_H */

