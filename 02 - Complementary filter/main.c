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
uint16_t getLoopTimeUs();
void pmwInit();

/************************************************************************/
/* Global variables                                                            */
/************************************************************************/
volatile uint8_t t0OvfCount = 0; //Increased every 256us

//PMW
//Donne le temps d'execution du programme en ms (compte au max environ 1.5 mois soit environ 49 jours)
//MIS A JOUR SEULEMENT TOUTES LES 20MS VIA LA GENERATION PMW.
//volatile uint32_t timeFromStartMs = 0;

volatile uint16_t servo[4] = {2300, 2300, 2300, 2300}; //Initial speed in microseconds
volatile int8_t channel = 1; //Controlled motor number : 1, 2, 3 or 4
volatile uint16_t startPmwTcnt1 = 0; //TCNT1 value when the PMW cycle starts

/************************************************************************/
/* Timer 0 overflow. Every 256us.                                       */
/************************************************************************/
ISR(TIMER0_OVF_vect){
	t0OvfCount++;
}

/************************************************************************/
/* PMW                                                                     */
/************************************************************************/
//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
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


int main(void)
{
	/************************************************************************/
	/* Variables                                                            */
	/************************************************************************/
	uint8_t readCount = 0; //Controle if ready to read new sensors data values
	float timeMs = 0; //While loop time in milisecond
	uint8_t pidLoopTime = 0;
	uint8_t isInitializing = 1; //Used to know if it is initializing
	
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
				
		if((timeMs > 7000) && (timeMs < 60000)){
		
			if(isInitializing){
				isInitializing = 0;
				mCompAccelInit();
				mCompGyroInit();			
				mCompInit();
			}
			else{
			
				readCount += mCompReadAccel();
				readCount += mCompReadGyro();
				
				//If new gyro and accelerometer data has been read. Every 10-20msms - 100-200Hz.
				if(readCount >= 2){
					
					readCount = 0;
					
					//PID
					float pitchSetpoint = 0.0f;
					float pitch = mCompCompute(loopTimeMs/1000.0f);
					float error = pitchSetpoint - pitch;
					
					//servo[0] = 850 + (error*20.0f); Proportionnal regulator
					uint16_t pitchServo = servo[0] + (error * 0.2f); //Integrator regulator - Miss the time variable
					if(pitchServo > 2300){
						pitchServo = 2300;
					}
					if(pitchServo < 700){
						pitchServo = 700;
					}
					servo[0] = pitchServo;
				
				}
				
			}
			
			
		}
		
		
		if(timeMs > 60000){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		
    }
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

void pmwInit(){
	//PMW
	TCCR1B |= 1<<CS11; //Prescaler of 8 because 8MHz clock source
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //LED and motors as output
	PORTD = 1<<channel; //Set first servo pin high
}