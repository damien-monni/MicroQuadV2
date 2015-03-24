/*
 * monni_compFilter.h
 *
 * Created: 24/03/2015 20:51:43
 *  Author: Damien MONNI
 */ 


#ifndef MONNI_COMPFILTER_H_
#define MONNI_COMPFILTER_H_

#include <avr/io.h>
#include "monni_i2c.h"


/************************************************************************/
/* Functions                                                            */
/************************************************************************/
void mCompInit();
void mCompGyroInit();
uint8_t mCompAhrsCompute();


#endif /* MONNI_COMPFILTER_H_ */