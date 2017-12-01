/**
	\file
	\brief
		This is the header file for the UART device driver.
		It contains the macros and function definition.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	27/07/2015
 */
#ifndef UART_H_
#define UART_H_

#include "GPIO.h"
#include "NVIC.h"

/**
 * \brief A mail box type definition for serial port
 */
typedef struct{
	uint8 flag; /** Flag to indicate that there is new data*/
	uint8 mailBox; /** it contains the received data*/
} UART_MailBoxType;


/**
 * \brief This enum define the UART port to be used.
 */
typedef enum {UART_0,UART_1,UART_2,UART_3,UART_4,UART_5}UART_ChannelType;

/**
 * \brief It defines some common transmission baud rates
 */
typedef enum {BD_4800=4800,BD_9600=9600,BD_5600=5600, BD_115200=115200,BD_38400=38400}UART_BaudRateType;

/**
 * \brief  struct containing the UART configuration values
 */
typedef struct{
	UART_ChannelType UART_channel;
	GPIO_portNameType UART_GPIOport;
	BitsType UART_RX_pin;
	BitsType UART_TX_pin;
	uint32 systemClk;
	UART_BaudRateType baudRate;
	PriorityLevelType interrupt_priority;
}UART_config_type;



//
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It configures the UART to be used
 	 \param[in]  uartChannel indicates which UART will be used.
 	 \param[in]  systemClk indicates the MCU frequency.
 	 \param[in]  baudRate sets the baud rate to transmit.
 	 \return void
 */
void UART_init(UART_ChannelType uartChannel, uint32 systemClk, UART_BaudRateType baudRate);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 enables the RX UART interrupt). This function should include the next sentence:
 	 while (!(UART0_S1 & UART_S1_RDRF_MASK)). It is to guaranty that the incoming data is complete
 	 when reception register is read. For more details see chapter 52 in the kinetis reference manual.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \return void
 */
void UART_interruptEnable(UART_ChannelType uartChannel);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It sends one character through the serial port. This function should include the next sentence:
 	 while(!(UART0_S1 & UART_S1_TC_MASK)). It is to guaranty that before to try to transmit a byte, the previous
 	 one was transmitted. In other word, to avoid to transmit data while the UART is busy transmitting information.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \param[in]  character to be transmitted.
 	 \return void
 */

void UART_putChar (UART_ChannelType uartChannel, uint8 character);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It sends a string character through the serial port.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \param[in]  string pointer to the string to be transmitted.
 	 \return void
 */
void UART_putString(UART_ChannelType uartChannel, sint8* string);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It activates the UART clock gating.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \return void
 */

void UART_clockGating(UART_ChannelType uartChannel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It clears the mailbox flag
 	 \return void
 */
void UART_clearInterrupt(UART_ChannelType uartChannel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that configures the initialization of the selected UART
 	\param[in]	UART channel
 	\return Nothing
 */
void UART_Init(UART_config_type uart);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that configures the SBR of the selected UART
 	\param[in]	UART channel
 	\param[in]	UART SBR
 	\return Nothing
 */
void UART_loadSBR(UART_ChannelType uartChannel, uint32 sbr);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that configures the BRFA of the selected UART
 	\param[in]	UART channel
 	\param[in]	UART BRFA
 	\return Nothing
 */
void UART_loadBRFA(UART_ChannelType uartChannel, uint8 brfa);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that disable the Transmition and reception mode of the selected UART
 	\param[in]	UART channel
 	\return Nothing
 */
void UART_disableTXandRX(UART_ChannelType uartChannel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that enables the Transmition and reception mode of the selected UART
 	\param[in]	UART channel
 	\return Nothing
 */
void UART_enableTXandRX(UART_ChannelType uartChannel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that configures the Mailbox flag of the selected UART
 	\param[in]	UART channel
 	\return Nothing
 */
uint8 UART_getMailBoxFlag(UART_ChannelType uartChannel);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function  that configures the Mailbox value of the selected UART
 	\param[in]	UART channel
 	\return Nothing
 */
uint8 UART_getMailBoxValue(UART_ChannelType uartChannel);

#endif /* UART_H_ */

