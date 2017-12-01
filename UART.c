/**
	\file
	\brief
		This is the source file for the UART Device Driver.
		Contains all the mechanism necessary to use Device driver
	\author Santamaria, G., Gonzalez, E.
	\date	28/10/2017
 */

#include "UART.h"

UART_MailBoxType UART_MailBox[6];

uint8 UART_getMailBoxValue(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		return UART_MailBox[UART_0].mailBox;
		break;
	case UART_1:
		return UART_MailBox[UART_1].mailBox;
		break;
	case UART_2:
		return UART_MailBox[UART_2].mailBox;
		break;
	case UART_3:
		return UART_MailBox[UART_3].mailBox;
		break;
	case UART_4:
		return UART_MailBox[UART_4].mailBox;
		break;
	case UART_5:
		return UART_MailBox[UART_5].mailBox;
		break;
	default:
		return 0;
		break;
	}
}

uint8 UART_getMailBoxFlag(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		return UART_MailBox[UART_0].flag;
		break;
	case UART_1:
		return UART_MailBox[UART_1].flag;
		break;
	case UART_2:
		return UART_MailBox[UART_2].flag;
		break;
	case UART_3:
		return UART_MailBox[UART_3].flag;
		break;
	case UART_4:
		return UART_MailBox[UART_4].flag;
		break;
	case UART_5:
		return UART_MailBox[UART_5].flag;
		break;
	default:
		return 0;
		break;
	}
}

void UART0_RX_TX_IRQHandler(){
	if(UART0->S1 & UART_S1_RDRF_MASK){	//if receiver interrupts,
		UART_MailBox[UART_0].flag = TRUE;			//a data has been received
			uint8 dummy1 = UART0->S1;					//procedure to clear the RX interrupt flag
			UART_MailBox[UART_0].mailBox = UART0->D; 	//the received data is stored in the mailbox
	}
}

void UART1_RX_TX_IRQHandler(){
	if(UART1->S1 & UART_S1_RDRF_MASK){	//if receiver interrupts,
		UART_MailBox[UART_1].flag = TRUE;			//a data has been received
			uint8 dummy1 = UART1->S1;					//procedure to clear the RX interrupt flag
			UART_MailBox[UART_1].mailBox = UART1->D; 	//the received data is stored in the mailbox
	}
}

void UART3_RX_TX_IRQHandler(){
	if(UART3->S1 & UART_S1_RDRF_MASK){	//if receiver interrupts,
		UART_MailBox[UART_3].flag = TRUE;			//a data has been received
			uint8 dummy1 = UART3->S1;					//procedure to clear the RX interrupt flag
			UART_MailBox[UART_3].mailBox = UART3->D; 	//the received data is stored in the mailbox
	}
}

void UART_Init(UART_config_type uart_conf){
	uint32 SBR = 0;		//declares the SBR variable
	uint8 brfa = 0;		//declares the brfa variable
	UART_clockGating(uart_conf.UART_channel);	//enables the clock gating for the selected UART
	GPIO_clockGating(uart_conf.UART_GPIOport);	//enables the selected pins port clock gating
	switch(uart_conf.UART_GPIOport){				//depending on the selected 
	case GPIO_A:
		PORTA->PCR[uart_conf.UART_TX_pin] = PORT_PCR_MUX(3);
		PORTA->PCR[uart_conf.UART_RX_pin] = PORT_PCR_MUX(3);
		break;
	case GPIO_B:
		PORTB->PCR[uart_conf.UART_TX_pin] = PORT_PCR_MUX(3);
		PORTB->PCR[uart_conf.UART_RX_pin] = PORT_PCR_MUX(3);
		break;
	case GPIO_C:
		PORTC->PCR[uart_conf.UART_TX_pin] = PORT_PCR_MUX(3);
		PORTC->PCR[uart_conf.UART_RX_pin] = PORT_PCR_MUX(3);
		break;
	case GPIO_D:
		PORTD->PCR[uart_conf.UART_TX_pin] = PORT_PCR_MUX(3);
		PORTD->PCR[uart_conf.UART_RX_pin] = PORT_PCR_MUX(3);
		break;
	case GPIO_E:
		PORTE->PCR[uart_conf.UART_TX_pin] = PORT_PCR_MUX(3);
		PORTE->PCR[uart_conf.UART_RX_pin] = PORT_PCR_MUX(3);
		break;
	}
	SBR = (uint16)((uart_conf.systemClk)/(uart_conf.baudRate * 16));
	brfa = (uint8)(2*((uart_conf.systemClk)/(uart_conf.baudRate)) - (SBR*32));
	brfa &= 0x1F;
	UART_disableTXandRX(uart_conf.UART_channel);
	UART_loadSBR(uart_conf.UART_channel,SBR);
	UART_loadBRFA(uart_conf.UART_channel,brfa);
	UART_enableTXandRX(uart_conf.UART_channel);
	UART_interruptEnable(uart_conf.UART_channel);
	switch(uart_conf.UART_channel){
	case UART_0:
		NVIC_enableInterruptAndPriotity(UART0_IRQ, uart_conf.interrupt_priority);
		break;
	case UART_1:
		NVIC_enableInterruptAndPriotity(UART1_IRQ, uart_conf.interrupt_priority);
		break;
	case UART_2:
		NVIC_enableInterruptAndPriotity(UART2_IRQ, uart_conf.interrupt_priority);
		break;
	case UART_3:
		NVIC_enableInterruptAndPriotity(UART3_IRQ, uart_conf.interrupt_priority);
		break;
	case UART_4:
		NVIC_enableInterruptAndPriotity(UART4_IRQ, uart_conf.interrupt_priority);
		break;
	case UART_5:
		NVIC_enableInterruptAndPriotity(UART5_IRQ, uart_conf.interrupt_priority);
		break;
	}
}

void UART_loadSBR(UART_ChannelType uartChannel, uint32 SBR){
	switch(uartChannel){
	case UART_0:
		UART0->BDH |=  (((SBR & 0x1F00) >> 8));
		UART0->BDL = (uint8)(SBR & 0x00FF);
		break;
	case UART_1:
		UART1->BDH |=  (((SBR & 0x1F00) >> 8));
		UART1->BDL = (uint8)(SBR & 0x00FF);
		break;
	case UART_2:
		UART2->BDH |=  (((SBR & 0x1F00) >> 8));
		UART2->BDL = (uint8)(SBR & 0x00FF);
		break;
	case UART_3:
		UART3->BDH |=  (((SBR & 0x1F00) >> 8));
		UART3->BDL = (uint8)(SBR & 0x00FF);
		break;
	case UART_4:
		UART4->BDH |=  (((SBR & 0x1F00) >> 8));
		UART4->BDL = (uint8)(SBR & 0x00FF);
		break;
	case UART_5:
		UART5->BDH |=  (((SBR & 0x1F00) >> 8));
		UART5->BDL = (uint8)(SBR & 0x00FF);
		break;
	}
}

void UART_loadBRFA(UART_ChannelType uartChannel, uint8 brfa){
	switch(uartChannel){
	case UART_0:
		UART0->C4 |= brfa;
		break;
	case UART_1:
		UART1->C4 |= brfa;
		break;
	case UART_2:
		UART2->C4 |= brfa;
		break;
	case UART_3:
		UART3->C4 |= brfa;
		break;
	case UART_4:
		UART4->C4 |= brfa;
		break;
	case UART_5:
		UART5->C4 |= brfa;
		break;
	}
}

void UART_disableTXandRX(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_1:
		UART1->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_2:
		UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_3:
		UART3->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_4:
		UART4->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_5:
		UART5->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	}
}

void UART_enableTXandRX(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		UART0->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_1:
		UART1->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_2:
		UART2->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_3:
		UART3->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_4:
		UART4->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	case UART_5:
		UART5->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
		break;
	}
}

void UART_interruptEnable(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		while(!(UART0->S1 & UART_S1_TC_MASK)); //checks if there isnt anything being transmitted
			UART0->C2 |= UART_C2_RIE_MASK;	//receiver interrupt enabled
		break;
	case UART_1:
		while(!(UART1->S1 & UART_S1_TC_MASK)); //checks if there isnt anything being transmitted
			UART1->C2 |= UART_C2_RIE_MASK;	//receiver interrupt enabled
		break;
	case UART_2:
		while(!(UART2->S1 & UART_S1_TC_MASK)); //checks if there isnt anything being transmitted
			UART2->C2 |= UART_C2_RIE_MASK;	//receiver interrupt enabled
		break;
	case UART_3:
		while(!(UART3->S1 & UART_S1_TC_MASK)); //checks if there isnt anything being transmitted
			UART3->C2 |= UART_C2_RIE_MASK;	//receiver interrupt enabled
		break;
	case UART_4:
		while(!(UART4->S1 & UART_S1_TC_MASK)); //checks if there isnt anything being transmitted
			UART4->C2 |= UART_C2_RIE_MASK;	//receiver interrupt enabled
		break;
	case UART_5:
		while(!(UART5->S1 & UART_S1_TC_MASK)); //checks if there isnt anything being transmitted
			UART5->C2 |= UART_C2_RIE_MASK;	//receiver interrupt enabled
		break;
	}
}

void UART_putChar (UART_ChannelType uartChannel, uint8 character){
	switch(uartChannel){
	case UART_0:
		while(!(UART0->S1 & UART_S1_TDRE_MASK));
			UART0->D = character;						//sends the character
		break;
	case UART_1:
		while(!(UART1->S1 & UART_S1_TDRE_MASK));	//if uart1 s1 register tdre field is false,
				UART1->D = character;						//sends the character
		break;
	case UART_2:
		while(!(UART2->S1 & UART_S1_TDRE_MASK));	//if uart2 s1 register tdre field is false,
				UART2->D = character;						//sends the character
		break;
	case UART_3:
		while(!(UART3->S1 & UART_S1_TDRE_MASK));	//if uart3 s1 register tdre field is false,
				UART3->D = character;						//sends the character
		break;
	case UART_4:
		while(!(UART4->S1 & UART_S1_TDRE_MASK));	//if uart4 s1 register tdre field is false,
				UART4->D = character;						//sends the character
		break;
	case UART_5:
		while(!(UART5->S1 & UART_S1_TDRE_MASK));	//if uart5 s1 register tdre field is false,
				UART5->D = character;						//sends the character
		break;
	}
}

void UART_putString(UART_ChannelType uartChannel, sint8* string){	//todo check if it works properly
	while (*string)		//while there is a char remaining to be transmitted
		  UART_putChar(uartChannel,*string++);	//calls the put char function
}

void UART_clockGating(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
		break;
	case UART_1:
		SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
		break;
	case UART_2:
		SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		break;
	case UART_3:
		SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
		break;
	case UART_4:
		SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
		break;
	case UART_5:
		SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;
		break;
	}
}

void UART_clearInterrupt(UART_ChannelType uartChannel){
	switch(uartChannel){
	case UART_0:
		UART_MailBox[UART_0].flag = FALSE;	//it clears the received data flag
		UART_MailBox[UART_0].mailBox = 0;	//it clears the mailbox
		break;
	case UART_1:
		UART_MailBox[UART_1].flag = FALSE;	//it clears the received data flag
		UART_MailBox[UART_1].mailBox = 0;	//it clears the mailbox
		break;
	case UART_2:
		UART_MailBox[UART_2].flag = FALSE;	//it clears the received data flag
		UART_MailBox[UART_2].mailBox = 0;	//it clears the mailbox
		break;
	case UART_3:
		UART_MailBox[UART_3].flag = FALSE;	//it clears the received data flag
		UART_MailBox[UART_3].mailBox = 0;	//it clears the mailbox
		break;
	case UART_4:
		UART_MailBox[UART_4].flag = FALSE;	//it clears the received data flag
		UART_MailBox[UART_4].mailBox = 0;	//it clears the mailbox
		break;
	case UART_5:
		UART_MailBox[UART_5].flag = FALSE;	//it clears the received data flag
		UART_MailBox[UART_5].mailBox = 0;	//it clears the mailbox
		break;
	}
}
