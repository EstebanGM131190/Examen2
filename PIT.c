#include "MK64F12.h"
#include "PIT.h"
#include "NVIC.h"

uint8 PIT0_flag = FALSE;	/**PIT 0 interrupt flag*/
uint8 PIT1_flag = FALSE;	/**PIT 1 interrupt flag*/
uint8 PIT2_flag = FALSE;	/**PIT 2 interrupt flag*/
uint8 PIT3_flag = FALSE;	/**PIT 3 interrupt flag*/

uint8 PIT_getIntrFlags(uint8 PITtimer){	/**This function returns the current state of the desired PIT interrupt flag*/
	switch(PITtimer){
	case 0:		/**PIT0*/
		return PIT0_flag;	/**Returns PIT0 interrupt flag*/
		break;
	case 1:		/**PIT1*/
		return PIT1_flag;	/**Returns PIT1 interrupt flag*/
		break;
	case 2:		/**PIT2*/
		return PIT2_flag;	/**Returns PIT2 interrupt flag*/
		break;
	case 3:		/**PIT3*/
		return PIT3_flag;	/**Returns PIT3 interrupt flag*/
		break;
	default:				/**invalid PIT selected*/
		return 0;			/**returns a 0 for code security*/
		break;
	}
}

void PIT_stop(PIT_Timer_t pitTimer){	/**this function disables the desired PIT and its interrupt*/
	PIT->CHANNEL[pitTimer].TCTRL &= ~(PIT_TCTRL_TEN_MASK);	/** stops the timer */
	PIT->CHANNEL[pitTimer].TCTRL &= ~(PIT_TCTRL_TIE_MASK);	/** disables PIT_0 interrupts */
}

void PIT0_IRQHandler(){		/** PIT_0 ISR */
	volatile uint8 dummy;	/** dummy variable used to force PIT_0 TCTRL read, so the bug can be solved */
	PIT->CHANNEL[PIT_0].TFLG |= PIT_TFLG_TIF_MASK;	/** clears the PIT 0 TimeOut flag */
	dummy = PIT->CHANNEL[PIT_0].TCTRL;	/** read control register for clear PIT flag, this is silicon bug */
	PIT0_flag = TRUE;	/**activates local variable flag in order to make clear an interrupt has ocurred*/
}

void PIT1_IRQHandler(){		/** PIT_1 ISR */
	volatile uint8 dummy;	/** dummy variable used to force PIT_1 TCTRL read, so the bug can be solved */
	PIT->CHANNEL[PIT_1].TFLG |= PIT_TFLG_TIF_MASK;	/** clears the PIT 1 TimeOut flag */
	dummy = PIT->CHANNEL[PIT_1].TCTRL;	/** read control register for clear PIT flag, this is silicon bug */
	PIT1_flag = TRUE;	/**activates local variable flag in order to make clear an interrupt has ocurred*/
}

void PIT2_IRQHandler(){		/** PIT_2 ISR */
	volatile uint8 dummy;	/** dummy variable used to force PIT_2 TCTRL read, so the bug can be solved */
	PIT->CHANNEL[PIT_2].TFLG |= PIT_TFLG_TIF_MASK;	/** clears the PIT 2 TimeOut flag */
	dummy = PIT->CHANNEL[PIT_2].TCTRL;	/** read control register for clear PIT flag, this is silicon bug */
	PIT2_flag = TRUE;	/**activates local variable flag in order to make clear an interrupt has ocurred*/
}

void PIT3_IRQHandler(){		/** PIT_3 ISR */
	volatile uint8 dummy;	/** dummy variable used to force PIT_3 TCTRL read, so the bug can be solved */
	PIT->CHANNEL[PIT_3].TFLG |= PIT_TFLG_TIF_MASK;	/** clears the PIT 3 TimeOut flag */
	dummy = PIT->CHANNEL[PIT_3].TCTRL;	/** read control register for clear PIT flag, this is silicon bug */
	PIT3_flag = TRUE;	/**activates local variable flag in order to make clear an interrupt has ocurred*/
}

void timerInterrupt_Enable(PIT_Timer_t pitTimer){	/** This function enables the desired PIT interrupt */
	switch(pitTimer){
	case PIT_0:
		PIT->CHANNEL[PIT_0].TCTRL |= PIT_TCTRL_TIE_MASK;	/** PIT_0 interrupt enabled */
		break;
	case PIT_1:
		PIT->CHANNEL[PIT_1].TCTRL |= PIT_TCTRL_TIE_MASK;	/** PIT_1 interrupt enabled */
		break;
	case PIT_2:
		PIT->CHANNEL[PIT_2].TCTRL |= PIT_TCTRL_TIE_MASK;	/** PIT_2 interrupt enabled */
		break;
	case PIT_3:
		PIT->CHANNEL[PIT_3].TCTRL |= PIT_TCTRL_TIE_MASK;	/** PIT_3 interrupt enabled */
		break;
	}
}

void Timer_Enable(PIT_Timer_t pitTimer){	/** This function enables the desired PIT */
	switch(pitTimer){
	case PIT_0:
		PIT->CHANNEL[PIT_0].TCTRL |= PIT_TCTRL_TEN_MASK;	/** PIT_0 enabled */
		break;
	case PIT_1:
		PIT->CHANNEL[PIT_1].TCTRL |= PIT_TCTRL_TEN_MASK;	/** PIT_1 enabled */
		break;
	case PIT_2:
		PIT->CHANNEL[PIT_2].TCTRL |= PIT_TCTRL_TEN_MASK;	/** PIT_2 enabled */
		break;
	case PIT_3:
		PIT->CHANNEL[PIT_3].TCTRL |= PIT_TCTRL_TEN_MASK;	/** PIT_3 enabled */
		break;
	}
}

void PIT_delay(PIT_Timer_t pitTimer,float systemClock ,float perior){
	PIT->MCR &= ~(PIT_MCR_MDIS_MASK);	/** clears the MDIS field of the PTR_MCR register, enabling PIT timers */
	float period_cycles = ((perior)*(systemClock))-1;	/** Calculates the number of cycles for the desired delay period */
	uint32 period_cycles_casted = (uint32)period_cycles;
	//if(period_cycles_casted>18446744065119617025){	/**if it exceeds timer 1 and 2 together capacity */
		/** pending to be implemented, timer 1, 2 and 3 */
	//}
	if(period_cycles_casted>4294967295){	/** if it exceeds timer 0 capacity */
		/** pending to be implemented, timer 1 and 2 */
	}else{	/** if timer 0 capacity is enough */
		PIT->CHANNEL[pitTimer].LDVAL = period_cycles_casted;	/** setup timer 0 for the calculated period cycles */
		timerInterrupt_Enable(pitTimer);	/** enable PIT 0 timer interrupts */
		Timer_Enable(pitTimer);	/** starts PIT 0 timer */
	}
}

void PIT_clockGating(void){		/**This function enables the clock gate for the PIT module*/
	SIM->SCGC6 |= PIT_CLOCK_GATING;		/** Bit 23 of SIM_SCGC6 is  set */
}

uint8 PIT_interruptFlagStatus(void){	/**omitida por instrucciones del profesor*/
	return 0;}

//uint8 PIT_getIntrStutus(PIT_Timer_t pitTimer){	/** This function returns the current status of the Interrupt flag(local variable) */
//	return flag;	/** returns current status: interrupt active/not active */
//}

void PIT_clear(uint8 PITtimer){	/** This function clears the local variable flag, as the interrupt is not active */
	switch(PITtimer){
	case 0:
		PIT0_flag = FALSE;	/** the interrupt flag (local variable) is not active */
		break;
	case 1:
		PIT1_flag = FALSE;	/** the interrupt flag (local variable) is not active */
		break;
	case 2:
		PIT2_flag = FALSE;	/** the interrupt flag (local variable) is not active */
		break;
	case 3:
		PIT3_flag = FALSE;	/** the interrupt flag (local variable) is not active */
		break;
	}
}


void PIT_init(const PIT_ConfigType* pit){
	PIT_clockGating();		//enables pit clock gating
	switch(pit->PIT_channel){	//depending on the pit channel selected,
	case PIT_0:
		NVIC_enableInterruptAndPriotity(PIT_CH0_IRQ, pit->PIT_priority);	//enables PIT 0 interrupt with the selected priority level
		break;
	case PIT_1:
		NVIC_enableInterruptAndPriotity(PIT_CH1_IRQ, pit->PIT_priority);	//enables PIT 1 interrupt with the selected priority level
		break;
	case PIT_2:
		NVIC_enableInterruptAndPriotity(PIT_CH2_IRQ, pit->PIT_priority);	//enables PIT 2 interrupt with the selected priority level
		break;
	case PIT_3:
		NVIC_enableInterruptAndPriotity(PIT_CH3_IRQ, pit->PIT_priority);	//enables PIT 3 interrupt with the selected priority level
		break;
	}
}
