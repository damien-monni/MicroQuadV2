/*
 * monni_compFilter.c
 *
 * Created: 24/03/2015 20:51:56
 *  Author: Damien MONNI
 */

#include "monni_compFilter.h"

/************************************************************************/
/* Private variables                                                     */
/************************************************************************/
static const uint8_t accelAdd = 0b0011101; //7 bits accelerometer's address
static const uint8_t gyroAdd = 0b1101011; //7 bits gyro's address
static const int16_t GRAVITY = 4096; //Raw value of gravity. 8g max on 16 signed bits => 1g = 4096.


/************************************************************************/
/* GYRO INIT                                                            */
/************************************************************************/
void gyroInit(){
	
	while(twiWriteOneByte(gyroAdd, 0x20, 0x0F) == 0); //CTRL1 => 100Hz - Low Pass Filter 25Hz
	while(twiWriteOneByte(gyroAdd, 0x21, 0x00) == 0); //CTRL2 => High pass filter 8Hz to remove some drift
	while(twiWriteOneByte(gyroAdd, 0x23, 0xA0) == 0); //CTRL4 => BDU - Full scale 2000dps
	while(twiWriteOneByte(gyroAdd, 0x24, 0x10) == 0); //CTRL5 => High pass filter enable

}