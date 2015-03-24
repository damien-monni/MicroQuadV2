/*
 * _02___Complementary_filter.c
 *
 * Created: 24/03/2015 20:44:00
 *  Author: Damien MONNI
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "monni_compFilter.h"

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
void initLed();

/************************************************************************/
/* Variables                                                            */
/************************************************************************/
volatile uint32_t t0OvfCount = 0; //Increased every 256us


/************************************************************************/
/* Timer 0 overflow. Every 256us.                                                                     */
/************************************************************************/
ISR(TIMER0_OVF_vect){
	t0OvfCount++;
}

int main(void)
{
	/************************************************************************/
	/* Variables                                                            */
	/************************************************************************/
	
	
	/************************************************************************/
	/* Initializations                                                      */
	/************************************************************************/
	initLed();
	mCompGyroInit();
	
	mCompInit();
	sei();
	
	int cpt = 0;
	
    while(1)
    {
		//Invert LED every 100 times gyro data are available (should be every second because of 100Hs ODR)
        if(mCompAhrsCompute()){
			cpt++;
		}
		if(cpt == 100){
			cpt = 0;
			PORTD ^= 1 << PORTD0;
		}
    }
}


void initLed(){
	DDRD |= 1<<DDD0; //PORTD0 as output
	PORTD |= 1<<PORTD0;
	
	PORTD |= 1<<PORTD0; //Turn on LED on PORTD0
	_delay_ms(1500); //Wait 1.5s
	PORTD &= ~(1<<PORTD0); //Turn off LED on PORTD0
	_delay_ms(1500); //Wait 1.5s
}