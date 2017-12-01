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


static MORSE_StatesMachine MORSE[27] = {	//states machine for the terminals internal working; stores all the possible combinations from all states, and their respective function calling
/**A*/					{a,{1,2,3,0,0,0}},
/**B*/					{b,{2,1,1,1,0,0}},
/**C*/					{c,{2,1,2,1,3,0}},
/**D*/					{d,{2,1,1,3,0,0}},
/**E*/					{e,{1,3,0,0,0,0}},
/**F*/					{f,{1,1,2,1,3,0}},
/**G*/					{g,{2,2,1,3,0,0}},
/**H*/					{h,{1,1,1,1,3,0}},
/**I*/					{i,{1,1,3,0,0,0}},
/**J*/					{j,{1,2,2,2,3,0}},
/**K*/					{k,{2,1,2,3,0,0}},
/**L*/					{l,{1,2,1,1,3,0}},
/**M*/					{m,{2,2,3,0,0,0}},
/**N*/					{n,{2,1,3,0,0,0}},
/**O*/					{o,{2,2,2,3,0,0}},
/**P*/					{p,{1,2,2,1,3,0}},
/**Q*/					{q,{2,2,1,2,3,0}},
/**R*/					{r,{1,2,1,3,0,0}},
/**S*/					{s,{1,1,1,3,0,0}},
/**T*/					{t,{2,3,0,0,0,0}},
/**U*/					{u,{1,1,2,3,0,0}},
/**V*/					{v,{1,1,1,3,0,0}},
/**W*/					{w,{1,2,2,3,0,0}},
/**X*/					{x,{2,1,1,2,3,0}},
/**Y*/					{y,{2,1,2,2,3,0}},
/**Z*/					{z,{2,2,1,1,3,0}},
/**1*/					{1,{1,2,2,2,2,3}},
/**2*/					{2,{1,1,2,2,2,3}},
/**3*/					{3,{1,1,1,2,2,3}},
/**4*/					{4,{1,1,1,1,2,3}},
/**5*/					{5,{1,1,1,1,1,3}},
/**6*/					{6,{2,1,1,1,1,3}},
/**7*/					{7,{2,2,1,1,1,3}},
/**8*/					{8,{2,2,2,1,1,3}},
/**9*/					{9,{2,2,2,2,1,3}},
/**0*/					{0,{2,2,2,2,2,3}}
};

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
	uint8 flag_blink = FALSE;
	Initial_configuration();			//does all the required initial configuration
	GPIOB->PDDR = LED_ROJO + LED_AZUL;				//pines de led rojo y azul configurados como salida
		GPIOE->PDDR = LED_VERDE;						//pin de led verde configurado como salida
		GPIOC->PDDR &= ~(SW2);							//puerto C pin 6 (SW2) como entrada (bit en 0)
	LEDS_APAGADOS();
	uint8 DATA[20] = 0;
	uint8 counter = 0;
	while(1) {
		if(TRUE == UART_getMailBoxFlag(UART_0)){
			DATA[counter] = UART_getMailBoxValue(UART_0);
			UART_clearInterrupt(UART_0);	//clears the UART 0 reception interrupt
			//UART_putString(UART_0, "HolaMUNDO");
			UART_putChar(UART_0,'a');	//calls the put char function
			counter++;
			if(20 <= counter) counter = 0;

			uint8 Debugdummy = 0;

		}

		lectura_PBs();

		if(FALSE == input_sw2){
			PIT_delay(PIT_0, 21000000, 0.5);	//pit delay used to control the blinking transition
		}

		if(PIT_getIntrFlags(PIT_0) == TRUE){//DO SOMETHING
			if(flag_blink == FALSE){
				LED_AZUL_ENCENDIDO();
				flag_blink = TRUE;
			}
			else if(flag_blink == TRUE){
				LEDS_APAGADOS();
				flag_blink = FALSE;
			}
			PIT_clear(PIT_0);
		}
	}
}
