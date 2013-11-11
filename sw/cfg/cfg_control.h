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
/**
 * \brief		Inform if a SFP is inserted.
 * \param sfp	SFP number
 * \return 		0 if the SFP is virtually inserted, 0 if it's not
 */
int cfg_is_sfp_inserted(int sfp);

/******************************************************************************/
/**
 * \brief	What SFP is inserted.
 * \return 	SFP number or error code if there is no SFP inserted.
 */
int cfg_get_sfp_inserted(void);

/******************************************************************************/


#endif /* __CFG_CONTROL_H */

