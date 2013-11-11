#ifndef __HW_TIMER_H
#define __HW_TIMER_H

/******************************************************************************/
/**
 * \brief	Initialize timer.
 * \return	0 if OK, error code otherwise.
 */
int hw_timer_init(void);

/******************************************************************************/
/**
 * \brief	Wait an specific time.
 * \param t	Time (in ms) to wait.
 * \return	0 if OK, error code otherwise.
 */
int hw_sleep(int t);

/******************************************************************************/

#endif

