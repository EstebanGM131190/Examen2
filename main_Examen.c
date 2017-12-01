/**
	\file
	\brief
		This is the Main file for the project 3.
	\author Esteban Gonzalez Moreno ie565892@iteso.mx, Gabriel Santamaria Garcia ie699356@iteso.mx
	\date	7/09/2014
 */
#include "Configuration.h"
#define LED_ROJO 	0x400000		//PUERTO B PIN 22
#define LED_AZUL 	0x200000		//PUERTO B PIN 21
#define LED_VERDE	0x4000000		//PUERTO E PIN 26

#define LED_ROJO 	0x400000		//PUERTO B PIN 22
#define SW2			0x40			//PUERTO C PIN 6
//#define SW3			0x10			//PUERTO A PIN 4
uint32 input_sw2 = 0;
uint32 input_sw3 = 0;




void lectura_PBs(){
	input_sw2 = GPIOC->PDIR;	//guarda el estado del sw2
	input_sw2 = input_sw2 & SW2;
//	input_sw3 = GPIOA->PDIR;	//guarda el estado del sw3
//	input_sw3 = input_sw3 & SW3;
}
void LEDS_APAGADOS(){
	GPIOB->PSOR = LED_ROJO;		//pin de LED rojo en 1, es decir, apagado
	GPIOB->PSOR = LED_AZUL;		//pin de LED azul en 1, es decir, apagado
	GPIOE->PSOR = LED_VERDE;	//pin de LED verde en 1, es decir, apagado
}
void LED_AZUL_ENCENDIDO(){
	GPIOB->PSOR = LED_ROJO;		//pin de LED rojo en 1, es decir, apagado
	GPIOB->PCOR = LED_AZUL;		//pin de LED azul en 0, es decir, encendido
	GPIOE->PSOR = LED_VERDE;	//pin de LED verde en 1, es decir, apagado
}

int main(void) {
	Initial_configuration();			//does all the required initial configuration
	GPIOB->PDDR = LED_ROJO + LED_AZUL;				//pines de led rojo y azul configurados como salida
		GPIOE->PDDR = LED_VERDE;						//pin de led verde configurado como salida
		GPIOC->PDDR &= ~(SW2);							//puerto C pin 6 (SW2) como entrada (bit en 0)
	LEDS_APAGADOS();
	while(1) {
		lectura_PBs();
		if(FALSE == input_sw2){
			//DO SOMETHING
			LED_AZUL_ENCENDIDO();
		}

	}
}
