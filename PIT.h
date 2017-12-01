/*
 * PIT.h
 *
 *  Created on: 16/08/2017
 *      Author: jlpe
 */

#ifndef PIT_H_
#define PIT_H_

#include "DataTypeDefinitions.h"
#include "NVIC.h"

/**/
#define PIT_CLOCK_GATING 0x00800000

/*! This enumerated constant are used to select the PIT to be used*/
typedef enum {PIT_0,PIT_1,PIT_2,PIT_3} PIT_Timer_t;

typedef struct{
	PIT_Timer_t	PIT_channel;
	PriorityLevelType PIT_priority;
} PIT_ConfigType;


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function configure the PIT to generate a delay base on the system clock.
 	 Internally it configures the clock gating and enables the PIT module.
 	 It is important to note that this strictly is not device driver since everything is
 	 contained in a single function, but in general you have to avoid this practices, this only
 	 for the propose of the homework

 	 \param[in] pitTimer timer to be configured
 	 \param[in] systemClock	the system clock frequency given in cycles
 	 \param[in] perior	the desired delay period
 	 \return void
 	 \todo	check if timer 0 overflow values in the comparison are correct
 */
void PIT_delay(PIT_Timer_t pitTimer,float systemClock ,float perior);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables the clock for the PIT module in order to make it work
 	 \param[in] void
 	 \return void
 */
void PIT_clockGating(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function is unimplemented because of instruction of the professor
 	 \param[in] void
 	 \return 0
 */
uint8 PIT_interruptFlagStatus(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns a PIT timer's interrupt flag current status
 	 \param[in] pitTimer selected timer to be analysed
 	 \return the desired PIT timer's interrupt flag status
 */
uint8 PIT_getIntrStutus(PIT_Timer_t pitTimer);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function clears the local variable "flag" in order to make clear the interrupt isn't active
 	 \param[in] void
 	 \return void
 */
void PIT_clear(uint8 PITtimer);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	This function stops the pit
 	\param[in]	pit timer
 	\return void
 */
void PIT_stop(PIT_Timer_t pitTimer);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	This function returns the current state of the desired PIT interrupt local variable flag
 	\param[in] uint8 PITtimer the selected PIT to know its flag
 	\return uint8 the
 */
uint8 PIT_getIntrFlags(uint8 PITtimer);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	This function erase the pit counter
 	\param[in]	void
 	\return void
 */
void Erase_counters();

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	This function initialize the pit counter
 	\param[in]	void
 	\return void
 */
void PIT_init(const PIT_ConfigType* pit);

#endif /* PIT_H_ */
