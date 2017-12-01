/**
	\file
	\brief
		This is the header file for the FlexTimer divice driver.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Add configuration structures.
 */

#ifndef FLEXTIMER_H_
#define FLEXTIMER_H_


#include "MK64F12.h"
#include "DataTypeDefinitions.h"

#define FLEX_TIMER_0_CLOCK_GATING 0x01000000
#define FLEX_TIMER_1_CLOCK_GATING 0x02000000
#define FLEX_TIMER_2_CLOCK_GATING 0x01000000
#define FLEX_TIMER_3_CLOCK_GATING 0x02000000

#define FLEX_TIMER_FAULTIE  0x80
#define FLEX_TIMER_FAULTM_0   0x00
#define FLEX_TIMER_FAULTM_1   0x20
#define FLEX_TIMER_FAULTM_2   0x40
#define FLEX_TIMER_FAULTM_3   0x60
#define FLEX_TIMER_CAPTEST  0x10
#define FLEX_TIMER_PWMSYNC  0x08
#define FLEX_TIMER_WPDIS    0x04
#define FLEX_TIMER_INIT     0x02
#define FLEX_TIMER_FTMEN    0x01

#define FLEX_TIMER_TOF     0x80
#define FLEX_TIMER_TOIE    0x40
#define FLEX_TIMER_CPWMS   0x20
#define FLEX_TIMER_CLKS_0  0x00
#define FLEX_TIMER_CLKS_1  0x08
#define FLEX_TIMER_CLKS_2  0x10
#define FLEX_TIMER_CLKS_3  0x18
#define FLEX_TIMER_PS_1    0x00
#define FLEX_TIMER_PS_2    0x01
#define FLEX_TIMER_PS_4    0x02
#define FLEX_TIMER_PS_8    0x03
#define FLEX_TIMER_PS_16    0x04
#define FLEX_TIMER_PS_32    0x05
#define FLEX_TIMER_PS_64    0x06
#define FLEX_TIMER_PS_128    0x07

#define FLEX_TIMER_PWMLOAD_CH0 0x01
#define FLEX_TIMER_PWMLOAD_CH1 0x02
#define FLEX_TIMER_PWMLOAD_CH2 0x04
#define FLEX_TIMER_PWMLOAD_CH3 0x08
#define FLEX_TIMER_PWMLOAD_CH4 0x10
#define FLEX_TIMER_PWMLOAD_CH5 0x20
#define FLEX_TIMER_PWMLOAD_CH6 0x40
#define FLEX_TIMER_PWMLOAD_CH7 0x80
#define FLEX_TIMER_LDOK        0x200


#define  FLEX_TIMER_DMA   0x01
#define  FLEX_TIMER_ELSA  0x04
#define  FLEX_TIMER_ELSB  0x08
#define  FLEX_TIMER_MSA   0x10
#define  FLEX_TIMER_MSB   0x20
#define  FLEX_TIMER_CHIE  0x40
#define  FLEX_TIMER_CHF   0x80

typedef enum{FLEXTIMER0, //FLEX TIMER CHANNEL 0
			 FLEXTIMER1, //FLEX TIMER CHANNEL 1
			 FLEXTIMER2, //FLEX TIMER CHANNEL 2
			 FLEXTIMER3, //FLEX TIMER CHANNEL 3
			 FLEXTIMER4, //FLEX TIMER CHANNEL 4
			 FLEXTIMER5, //FLEX TIMER CHANNEL 5
			 FLEXTIMER6, //FLEX TIMER CHANNEL 6
			 FLEXTIMER7, //FLEX TIMER CHANNEL 7
			 FLEXTIMER8, //FLEX TIMER CHANNEL 8
			 FLEXTIMER9, //FLEX TIMER CHANNEL 9
			 FLEXTIMER10, //FLEX TIMER CHANNEL 10
			 FLEXTIMER11, //FLEX TIMER CHANNEL 11
			 FLEXTIMER12, //FLEX TIMER CHANNEL 12
			 FLEXTIMER13, //FLEX TIMER CHANNEL 13
			 FLEXTIMER14, //FLEX TIMER CHANNEL 14
			 FLEXTIMER15, //FLEX TIMER CHANNEL 15
			 FLEXTIMER16, //FLEX TIMER CHANNEL 16
			} Flex_ModuleType;

typedef enum{CHANNEL0,
			 CHANNEL1,
			 CHANNEL2,
			 CHANNEL3,
			 CHANNEL4,
			 CHANNEL5
			}Flex_ChannelType;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function clears the FTM interrupts
	\param[in]	FTM module
	\param[in]	FTM channel
	\return void
 */
void FTM_channelInterruptISRclear(sint16 module, sint16 channel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function clears the timer overflow of the FTM->SC
	\param[in]	FTM channel
	\return void
 */
void FTM_timerOverflowISRclear(sint16 channelValue);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function clears the FTM input capture and overflow interrupt flags
	\param[in]	FTM module
	\return void
 */
void FTM_clearFlags(sint16 module);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function converts the cycles to frecuency
	\return frecuency (float)
 */
float FTM_converDeltaCyclesToFrequency();
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function gets the cycles of the input capture for the frequency reader
	\param[in]	FTM module
	\return void
 */
uint8 FTM_inputCapture_deltaCyclesGetter(sint16 module);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function clears the FTM overflow flag
	\param[in]	FTM module
	\return void
 */
void FTM_clearOverflowFlag(sint16 module);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function clears the FTM input capture flag
	\param[in]	FTM module
	\return void
 */
void FTM_clearInputCaptureFlag(sint16 module);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function compensate for the overflow flags on the input capture
	\param[in]	FTM module
	\param[in]	FTM overflow flag
	\return void
 */
void FTM_Capture_compensateOverflow(uint8 input_flag, sint16 module);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function updates the duty cycle value
	\param[in]	FTM channel value for the duty cycle
	\return void
 */
void FlexTimer_updateCHValue(sint16 channelValue);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function initializes the PWM
	\param[in]	FTM chanel value to initialize the PWM
	\return void
 */
void FlexTimer_Init(sint16 channelValue);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function initialize the FTM overflow mode to be used as a timer
	\param[in]	FTM channel value of the FTM
	\return void
 */
void FlexTimer_Init_Overflow(sint16 channelValue);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function GETS the FTM1 FLAG overflow
	\return FTM1 overflow flag
 */
uint8 FTM_GET_FLAG_FTM1_OVERFLOW();
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function clears the FTM1 overflow flag
	\return void
 */
void FTM_CLEAR_FLAG_FTM1_OVERFLOW();
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	\brief	This function initialize the input capture FTM mode
	\return void
 */
void FlexTimer_InputCapture();

#endif /* FLEXTIMER_H_ */
