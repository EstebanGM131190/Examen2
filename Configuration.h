/**
	\file
	\brief
		This is the header configuration file
		This file contains all the configuration functions for the project.
	\author Esteban Gonzalez Moreno ie565892@iteso.mx, Gabriel Santamaria Garcia ie699356@iteso.mx
	\date	7/09/2014
 */
#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "NVIC.h"
#include "FlexTimer.h"
#include "UART.h"
#include "GPIO.h"
#include "PIT.h"

typedef struct{
	uint8 DASH_DOTS[6];	 /**field used to know which function to use in terms of the state*/
	uint8 next[27];			/**member which contains the next state in relation to the input*/
}MORSE_StatesMachine;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function generates all initial required configuration
 	 \param[in] void
 	 \return void
 	 \todo	Finish the required configurations
 */
void Initial_configuration();
#endif
