/*
 * monni_pmw.c
 *
 * Created: 04/04/2015 14:34:10
 *  Author: SESA212079
 */

#include "monni_pmw.h"

/************************************************************************/
/* Variables                                                            */
/************************************************************************/
int8_t channel = 1; //Controlled motor number : 1, 2, 3 or 4
uint16_t startPmwTcnt1 = 0; //TCNT1 value when the PMW cycle starts

volatile uint16_t servo[4] = {2300, 2300, 2300, 2300};


/************************************************************************/
/* Functions                                                            */
/************************************************************************/

//PMW Building ISR
ISR(TIMER1_COMPA_vect){
	
	uint16_t timerValue = TCNT1;
	
	if(channel < 0){ //Every motors was pulsed, waiting for the next period
		//TCNT1 = 0;
		channel = 1;
		PORTD |= 1<<channel;
		startPmwTcnt1 = timerValue;
		OCR1A = timerValue + servo[0];
		//timeFromStartMs += 20;
	}
	else{
		if(channel < 4){ //Last servo pin just goes high
			OCR1A = timerValue + servo[channel];
			PORTD &= ~(1<<channel); //Clear actual motor pin
			PORTD |= 1<<(channel + 1); //Set the next one
			channel++;
		}
		else{
			PORTD &= ~(1<<channel); //Clear the last motor pin
			OCR1A = startPmwTcnt1 + 20000;
			channel = -1; //Wait for the next period
		}
	}
}

void pmwInit(){
	TCCR1B |= 1<<CS11; //Prescaler of 8 because 8MHz clock source
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //LED and motors as output
	PORTD = 1<<channel; //Set first servo pin high
}