/**
	\file
	\brief
		This is the starter file of FlexTimer.
		In this file the FlexTimer is configured in overflow mode.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Add configuration structures.
 */



#include "FlexTimer.h"
#include "MK64F12.h"
#include "NVIC.h"
#include "GPIO.h"

#define SYSTEMCLOCK 21000000
#define FTM_MOD_VALUE 0xFFFF
#define FLEX_TIMER_2_CG 0x04000000

uint8 FLAG_FTM1_OVERFLOW = FALSE;
uint32 Timer_value1 = 0;
uint8 FirstRead_Flag = FALSE;
uint32 Timer_delta = 0;
uint32 Real_time_delta = 0;
uint32 Frequency_read = 0;
uint8 FTM0_overflowFlag = FALSE;
uint8 FTM0_inputCaptureFlag = FALSE;
uint8 FTM1_overflowFlag = FALSE;
uint8 FTM1_inputCaptureFlag = FALSE;
uint8 FTM2_overflowFlag = FALSE;
uint8 FTM2_inputCaptureFlag = FALSE;
uint8 FTM3_overflowFlag = FALSE;
uint8 FTM3_inputCaptureFlag = FALSE;
uint8 OverflowCounter = 0;
uint8 FirstFlank = 0;
uint8 SecondFlank = 0;
uint8 FirstFlankCaptured = FALSE;
uint8 Delta_cycles = 0;
float DeltaTime = 0;
float Frequency = 0;
float CycleEquivalentTime = 0;

void FlexTimer_clockGating(sint16 channelValue){
	switch(channelValue){
	case FLEXTIMER0:
		SIM->SCGC6 |= FLEX_TIMER_0_CLOCK_GATING;/** Bit 24 of SIM_SCGC3 is set*/
	break;
	case FLEXTIMER1:
		SIM->SCGC6 |= FLEX_TIMER_1_CLOCK_GATING;/** Bit 25 of SIM_SCGC3 is set*/
	break;
	case FLEXTIMER2:
		SIM->SCGC6 |= FLEX_TIMER_2_CG;/** Bit 26 of SIM_SCGC3 is set*/
		break;
	case FLEXTIMER3:
		SIM->SCGC3 |= FLEX_TIMER_3_CLOCK_GATING;/** Bit 24 of SIM_SCGC3 is set*/
	break;
	}

}

void FTM0_IRQHandler(){
	if(128	== (FTM0->SC & FTM_SC_TOF_MASK)){	//if an overflow has occurred,
		FTM0_overflowFlag = TRUE;		//sets the overflow flag as TRUE
	}
	if(FTM0->STATUS & FTM_STATUS_CH0F_MASK){								//if a capture event has occurred,
		FTM0_inputCaptureFlag = TRUE;	//sets the input capture flag as TRUE
	}
	FTM_channelInterruptISRclear(FLEXTIMER0,CHANNEL0);		//clears the channel interrupt flag
	FTM_timerOverflowISRclear(FLEXTIMER0);					//clears the overflow flag

}
void FTM1_IRQHandler()
{
	FLAG_FTM1_OVERFLOW = TRUE;			//FTM1 overflow flag is set
	FTM1->SC &= ~FLEX_TIMER_TOF;		//Clears the FTM1 (TOF) flag
}
void FTM2_IRQHandler(){
	FTM2->SC &= ~FLEX_TIMER_TOF;		//Clears the FTM2 (TOF) flag
	GPIOD->PDOR ^= 0xFF;
}

void FTM_clearOverflowFlag(sint16 module){
	switch(module){
	case FLEXTIMER0:
		FTM0_overflowFlag = FALSE;  	//Clears the FTM0 overflow flag
		break;
	case FLEXTIMER3:
		FTM3_overflowFlag = FALSE;		//Clears the FTM3 overflow flag
		break;
	}
}

void FTM_clearInputCaptureFlag(sint16 module){
	switch(module){
	case FLEXTIMER0:
		FTM0_inputCaptureFlag = FALSE;		//Clears the FTM0 inputcapture flag
		break;
	case FLEXTIMER3:
		FTM3_inputCaptureFlag = FALSE;		//Clears the FTM3 input capture flag
		break;
	}
}
void FTM_Capture_compensateOverflow(uint8 input_flag, sint16 module){
	uint8 overflow_flag = input_flag;
	if(TRUE == overflow_flag){		//if an overflow occurred,
		OverflowCounter++;			//a lap has been completed
		FTM_clearOverflowFlag(module);	//clears the overflow flag
	}
}
void FlexTimer_InputCapture(){
	SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC;
	GPIO_pinControlRegisterType pinControlRegisterPORTC1 = GPIO_MUX4|GPIO_PE;//|GPIO_PS;

	GPIO_pinControlRegister(GPIO_C,BIT1, &pinControlRegisterPORTC1);

	/** Clock gating for the FlexTimer 0*/
	SIM->SCGC6 |= FLEX_TIMER_0_CLOCK_GATING;
		/**When write protection is enabled (WPDIS = 0), write protected bits cannot be written.
		* When write protection is disabled (WPDIS = 1), write protected bits can be written.*/
	FTM0->MODE |= FLEX_TIMER_WPDIS;
	FTM0->MODE |= FTM_MODE_FAULTM_MASK| FTM_MODE_FTMEN_MASK;
	FTM0->CONTROLS[0].CnSC |= FLEX_TIMER_ELSA;
	FTM0->CONTROLS[0].CnSC |= FLEX_TIMER_CHIE;

	FTM0->MOD = FTM_MOD_MOD_MASK;

	FTM0->SC = 0x00;
	FTM0->SC |= FLEX_TIMER_CLKS_1|FLEX_TIMER_PS_128|FTM_SC_TOIE_MASK;

	FTM0->CNT = 0x00;
	FTM0->CNTIN = 0x00;

	FTM0->COMBINE = 0x00;

	NVIC_enableInterruptAndPriotity(FTM0_IRQ, PRIORITY_5);
}

float FTM_converDeltaCyclesToFrequency(){
	CycleEquivalentTime = (float)1/((SYSTEMCLOCK)/128);
	DeltaTime = CycleEquivalentTime * Delta_cycles;
	return Frequency = (float)1/DeltaTime;
}

uint8 FTM_inputCapture_deltaCyclesGetter(sint16 module){
	uint8 returnState = FALSE;
	FTM_Capture_compensateOverflow(FTM0_overflowFlag, module);		//checks for an overflow event
	if(TRUE==FTM0_inputCaptureFlag && FALSE == FirstFlankCaptured){	//if the first flank showed,
		FirstFlank = FTM0->CONTROLS[0].CnV & (0x00FF);		//reads the first flank
		FirstFlankCaptured = TRUE;								//the first flank has already been captured
		FTM_clearInputCaptureFlag(module);
		returnState = FALSE;
	}else if(TRUE==FTM0_inputCaptureFlag && TRUE == FirstFlankCaptured){	//if the second flank showed,
		SecondFlank = (FTM_MOD_VALUE*OverflowCounter) + (FTM0->CONTROLS[0].CnV & (0x00FF));	//reads the second flank compensating the overflows ocurred
		OverflowCounter = 0;
		Delta_cycles = SecondFlank - FirstFlank;			//calculates the cycle difference between the two flanks captured
		FirstFlank = 0;
		SecondFlank = 0;
		FirstFlankCaptured = FALSE;								//the first flank can be recaptured now
		FTM_clearInputCaptureFlag(module);
		returnState =  TRUE;
	}
	return returnState;
}
void FTM_clearFlags(sint16 module){
	switch(module){
	case FLEXTIMER0:
		FTM0_overflowFlag = FALSE;
		FTM0_inputCaptureFlag = FALSE;
		break;
	case FLEXTIMER1:
		FTM1_overflowFlag = FALSE;
		FTM1_inputCaptureFlag = FALSE;
		break;
	case FLEXTIMER2:
		FTM2_overflowFlag = FALSE;
		FTM2_inputCaptureFlag = FALSE;
		break;
	case FLEXTIMER3:
		FTM3_overflowFlag = FALSE;
		FTM3_inputCaptureFlag = FALSE;
		break;
	}
}

void FTM_channelInterruptISRclear(sint16 module, sint16 channel){
	switch(module){
	case FLEXTIMER0:
		FTM0->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;		//Clears the FTM0 channel interrupt ISR
		break;
	case FLEXTIMER1:
		FTM1->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;		//Clears the FTM1 channel interrupt ISR
		break;
	case FLEXTIMER2:
		FTM2->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;		//Clears the FTM2 channel interrupt ISR
		break;
	case FLEXTIMER3:
		FTM3->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;		//Clears the FTM3 channel interrupt ISR
		break;
	}
}

void FTM_timerOverflowISRclear(sint16 channelValue){
	switch(channelValue){
	case FLEXTIMER0:
		FTM0->SC &= ~(FLEX_TIMER_TOF);		//Clears the FTM0 overflow interrupt ISR
		break;
	case FLEXTIMER1:
		FTM1->SC &= ~(FLEX_TIMER_TOF);		//Clears the FTM1 overflow interrupt ISR
		break;
	case FLEXTIMER2:
		FTM2->SC &= ~(FLEX_TIMER_TOF);		//Clears the FTM2 overflow interrupt ISR
		break;
	case FLEXTIMER3:
		FTM3->SC &= ~(FLEX_TIMER_TOF);		//Clears the FTM3 overflow interrupt ISR
		break;
	}
}



uint8 FTM_GET_FLAG_FTM1_OVERFLOW(){
	return FLAG_FTM1_OVERFLOW;		//Gets the FTM1 overflow flag
}
void FTM_CLEAR_FLAG_FTM1_OVERFLOW(){
	FLAG_FTM1_OVERFLOW = FALSE;		//Clear the FTM1 overflow flag
}


void FlexTimer_updateCHValue(sint16 channelValue)
{
	/**Assigns a new value for the duty cycle*/
	FTM2->CONTROLS[0].CnV = channelValue*2.56;	//updates the PWM value as it have a maximum of 255 to get 100% and channel value is a percentage it is multiplied by 2.56

}

void FlexTimer_Init_Overflow(sint16 channelValue)
{

	/** Clock gating for the FlexTimer 0*/
	FlexTimer_clockGating(channelValue);
	/**When write protection is enabled (WPDIS = 0), write protected bits cannot be written.
	 * When write protection is disabled (WPDIS = 1), write protected bits can be written.*/
	FTM1->MODE = FLEX_TIMER_WPDIS;
	/**Assigning a default value for modulo register*/
	FTM1->MOD = 0x5020;
	/**Enabling the interrupt,selecting the clock source and a pre-scaler of 128*/
	FTM1->SC = FLEX_TIMER_TOIE| FLEX_TIMER_CLKS_1|FLEX_TIMER_PS_128;
	NVIC_enableInterruptAndPriotity(FTM1_IRQ,PRIORITY_9);

}

void FlexTimer_Init(sint16 channelValue)
{
		/** Clock gating for the FlexTimer 0*/
		FlexTimer_clockGating(channelValue);
		/** FLEX PWM MODE*/ //checar esta parte
		PORTB->PCR[18]   = PORT_PCR_MUX(0x3);

		/**When write protection is enabled (WPDIS = 0), write protected bits cannot be written.
		* When write protection is disabled (WPDIS = 1), write protected bits can be written.*/
		FTM2->MODE |= FLEX_TIMER_WPDIS;
		/**Enables the writing over all registers*/
		FTM2->MODE &= ~FLEX_TIMER_FTMEN;
		/**Assigning a default value for modulo register*/
		FTM2->MOD = 0x00FF;
		/**Selects the Edge-Aligned PWM mode mode*/
		FTM2->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		/**Assign a duty cycle of 50%*/
		FTM2->CONTROLS[0].CnV = FTM0->MOD/2;
		/**Configure the times*/
		FTM2->SC = FLEX_TIMER_CLKS_1|FLEX_TIMER_PS_128;
}

