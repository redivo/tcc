#ifndef __CFG_CONTROL_H
#define __CFG_CONTROL_H

/******************************************************************************/
/**
 * \brief		Insert an SFP.
 * \param sfp	SFP number
 * \return 		0 if OK, error code otherwise
 */
int cfg_insert_sfp(int sfp);

/******************************************************************************/
/**
 * \brief		Remove an SFP.
 * \param sfp	SFP number
 * \return 		0 if OK, error code otherwise
 */
int cfg_remove_sfp(int sfp);

/******************************************************************************/

#endif /* __CFG_CONTROL_H */

