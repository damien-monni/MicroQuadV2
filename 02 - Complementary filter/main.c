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
uint16_t getLoopTimeUs();

/************************************************************************/
/* Global variables                                                            */
/************************************************************************/
volatile uint8_t t0OvfCount = 0; //Increased every 256us

/************************************************************************/
/* Timer 0 overflow. Every 256us.                                       */
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
	mCompAccelInit();
	mCompGyroInit();
	
	mCompInit();
	sei();
	
    while(1)
    {
		//If new gyro data has been read. Every 10ms - 100Hz.
        if(mCompReadAccel()){
			//Get loop time
			float loopTimeMs = getLoopTimeUs() / 1000.0f;
			float pitch = mCompCompute(loopTimeMs/1000.0f);
			
			if(((pitch > 10.0f) && (pitch < 20.0f)) || ((pitch > 30.0f) && (pitch < 40.0f)) || ((pitch > 50.0f) && (pitch < 60.0f))){
				PORTD |= 1<<PORTD0;
			}
			else{
				PORTD &= ~(1<<PORTD0);
			}
			
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

uint16_t getLoopTimeUs(){
	
	static uint8_t previousCount = 0; //Previous loop t0OvfCount. 8 bits => max 65,536ms
	uint8_t pastCount = 0;
	
	uint8_t actualCount = t0OvfCount;
	uint8_t t0 = TCNT0;
	
	if(actualCount > previousCount){
		pastCount = actualCount - previousCount;	
	}
	else{
		pastCount = (256 - previousCount) + actualCount;
	}
	
	previousCount = actualCount;
	
	return (pastCount*256) + t0;
	
}
