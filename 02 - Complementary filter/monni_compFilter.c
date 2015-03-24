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
/* Complementary filter init                                                                     */
/************************************************************************/
void mCompInit(){
	//8-bits Timer 0 configuration
	TCCR0A = 0; //Normal mode
	TCCR0B |= 1<<CS01; //Prescaling /8 => 1 tick every us
	TIMSK0 |= 1<<TOIE0; //Interrupt on overflow (every 256us)
}


/************************************************************************/
/* GYRO INIT                                                            */
/************************************************************************/
void mCompGyroInit(){
	
	while(twiWriteOneByte(gyroAdd, 0x20, 0x0F) == 0); //CTRL1 => 100Hz - Low Pass Filter 25Hz
	while(twiWriteOneByte(gyroAdd, 0x21, 0x00) == 0); //CTRL2 => High pass filter 8Hz to remove some drift
	while(twiWriteOneByte(gyroAdd, 0x23, 0xA0) == 0); //CTRL4 => BDU - Full scale 2000dps
	while(twiWriteOneByte(gyroAdd, 0x24, 0x10) == 0); //CTRL5 => High pass filter enable

}

uint8_t mCompReadGyro(){
	
	uint8_t gyroSplitedValues[6];
	int16_t gyroValues[3];
	
	//Check if X, Y and Z gyro data are available
	uint8_t gyroStatus = twiReadOneByte(gyroAdd, 0x27);
	if(gyroStatus & 0x08){
		//Read gyro values
		while(twiReadMultipleBytes(gyroAdd, 0x28, gyroSplitedValues, 6) == 0);
		gyroValues[0] = ((gyroSplitedValues[1] << 8) | (gyroSplitedValues[0] & 0xff));
		gyroValues[1] = ((gyroSplitedValues[3] << 8) | (gyroSplitedValues[2] & 0xff));
		gyroValues[2] = ((gyroSplitedValues[5] << 8) | (gyroSplitedValues[4] & 0xff));
		
		return 1;
	}
	
	return 0;
}