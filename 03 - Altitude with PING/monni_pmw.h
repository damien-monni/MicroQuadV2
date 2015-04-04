/*
 * monni_pmw.h
 *
 * Created: 04/04/2015 14:34:20
 *  Author: SESA212079
 */ 


#ifndef MONNI_PMW_H_
#define MONNI_PMW_H_

#include <avr/interrupt.h>

/************************************************************************/
/* Variables                                                            */
/************************************************************************/
extern volatile uint16_t servo[4]; //Initial speed in microseconds

/************************************************************************/
/* Functions                                                            */
/************************************************************************/
ISR(TIMER1_COMPA_vect);
void pmwInit();

#endif /* MONNI_PMW_H_ */