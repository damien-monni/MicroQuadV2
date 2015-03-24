/*
 * _02___Complementary_filter.c
 *
 * Created: 24/03/2015 20:44:00
 *  Author: Damien MONNI
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
void initLed();

int main(void)
{
    while(1)
    {
        /************************************************************************/
        /* LED INITIALISATION                                                   */
        /************************************************************************/
		initLed();
		
		
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