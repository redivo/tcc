#ifndef __CONTROL_PINS_H
#define __CONTROL_PINS_H

/******************************************************************************/
/**
 * \brief	Initialize all control pins.
 * \return	0 if OK, error code otherwise.
 */
int hw_ctrl_pins_init(void);

/******************************************************************************/
/**
 * \brief	Remove an SFP.
 * \param sfp		SFP number.
 * \return	0 if OK, error code otherwise.
 */
int hw_sfp_remove(int sfp);

/******************************************************************************/
/**
 * \brief	Insert an SFP.
 * \param sfp		SFP number.
 * \return	0 if OK, error code otherwise.
 */
int hw_sfp_insert(int sfp);

/******************************************************************************/

#endif /* __CONTROL_PINS_H */

