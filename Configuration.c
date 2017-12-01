/**
	\file
	\brief
		This is the configuration file
		This file contains all the configuration functions for the project.
	\author Esteban Gonzalez Moreno ie565892@iteso.mx, Gabriel Santamaria Garcia ie699356@iteso.mx
	\date	7/09/2014
 */

#include "Configuration.h"

PIT_ConfigType PIT0_Config = {PIT_0,PRIORITY_14};
UART_config_type UART_0_config = {UART_0,GPIO_B,BIT16,BIT17,60000000,BD_115200,PRIORITY_9};



void Initial_configuration(){	//function that does all the required configuration
	//Clock enable
	SIM->SCGC5 = SIM_SCGC5_PORTB_MASK + SIM_SCGC5_PORTE_MASK + SIM_SCGC5_PORTC_MASK + SIM_SCGC5_PORTA_MASK;	//encendido el reloj del puerto B, E, C y A

	//buttons
	PORTB->PCR[22] = PORT_PCR_MUX(1);	//LED
	PORTB->PCR[21] = PORT_PCR_MUX(1);   //LED
	PORTE->PCR[26] = PORT_PCR_MUX(1);   //LED
	PORTC->PCR[6]  = PORT_PCR_MUX(1) + PORT_PCR_PE_MASK + PORT_PCR_PS_MASK;	//modo GPIO + pull enable + pull up
	PORTA->PCR[4]  = PORT_PCR_MUX(1) + PORT_PCR_PE_MASK + PORT_PCR_PS_MASK; //modo GPIO + pull enable + pull up

	//PIT
	PIT_init(&PIT0_Config);	//pit 0 configuration


	//FLEXTIMER
	FlexTimer_Init_Overflow(FLEXTIMER1);
	//UART
	UART_Init(UART_0_config);

	EnableInterrupts;		//Enables all interrupts
}
