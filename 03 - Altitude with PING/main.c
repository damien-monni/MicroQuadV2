#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "monni_pmw.h"

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
uint16_t getLoopTimeUs();

/************************************************************************/
/* Global variables                                                            */
/************************************************************************/
volatile uint8_t t0OvfCount = 0; //Increased every 256us

//PMW
//Donne le temps d'execution du programme en ms (compte au max environ 1.5 mois soit environ 49 jours)
//MIS A JOUR SEULEMENT TOUTES LES 20MS VIA LA GENERATION PMW.
//volatile uint32_t timeFromStartMs = 0;

/************************************************************************/
/* PMW                                                                     */
/************************************************************************/

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
	float timeMs = 0; //While loop time in milisecond
	
	/************************************************************************/
	/* Initializations                                                      */
	/************************************************************************/
	pmwInit();
	
	sei();
	
    while(1)
    {
		float loopTimeMs = getLoopTimeUs() / 1000.0f;
		timeMs += loopTimeMs;
		
		if((timeMs > 2300) && (timeMs < 7000)){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
				
		if((timeMs > 7000) && (timeMs < 15000)){
				
		}
		
		
		if(timeMs > 15000){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		
    }
}

uint16_t getLoopTimeUs(){
	
	static uint8_t previousCount = 0; //Previous loop t0OvfCount. 8 bits => max 65,536ms (8*256us)
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