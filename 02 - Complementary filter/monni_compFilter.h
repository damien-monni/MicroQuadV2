/*
 * monni_compFilter.h
 *
 * Created: 24/03/2015 20:51:43
 *  Author: Damien MONNI
 */ 

#define F_CPU 8000000UL

#ifndef MONNI_COMPFILTER_H_
#define MONNI_COMPFILTER_H_

#include <avr/io.h>
#include <util/delay.h>
#include "monni_i2c.h"

/************************************************************************/
/* Variables                                                            */
/************************************************************************/



/************************************************************************/
/* Functions                                                            */
/************************************************************************/
void mCompInit();
void mCompAccelInit();
void mCompGyroInit();
uint8_t mCompReadAccel();
uint8_t mCompReadGyro();
float mCompCompute(float dt);


#endif /* MONNI_COMPFILTER_H_ */