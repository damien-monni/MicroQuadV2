/*
 * monni_compFilter.c
 *
 * Created: 24/03/2015 20:51:56
 *  Author: Damien MONNI
 */

#include "monni_compFilter.h"
#include <math.h>

/************************************************************************/
/* Private variables                                                     */
/************************************************************************/
const uint8_t accelAdd = 0b0011101; //7 bits accelerometer's address
const uint8_t gyroAdd = 0b1101011; //7 bits gyro's address
const int16_t GRAVITY = 4096; //Raw value of gravity. 8g max on 16 signed bits => 1g = 4096.
const uint8_t gyroSensitivity = 70; //70mdps for +/2000dps full scale as write in gyro's documentation

float pitch = 0;

int16_t accelValues[3]; //Raw accel data
int16_t gyroValues[3]; //Raw gyro data

/************************************************************************/
/* Complementary filter init                                            */
/************************************************************************/
void mCompInit(){
	//8-bits Timer 0 configuration
	TCCR0A = 0; //Normal mode
	TCCR0B |= 1<<CS01; //Prescaling /8 => 1 tick every us
	TIMSK0 |= 1<<TOIE0; //Interrupt on overflow (every 256us)
}

/************************************************************************/
/* ACCEL INIT                                                           */
/************************************************************************/
void mCompAccelInit(){
	while(twiWriteOneByte(accelAdd, 0x20, 0x6F) == 0); //CTRL1 => 100Hz - BDU - XYZ Enable
	while(twiWriteOneByte(accelAdd, 0x21, 0xC8) == 0); //CTRL2 => Anti alias BW 50Hz - +-4g
	
	
}

/************************************************************************/
/* Read an accel value                                                  */
/************************************************************************/
uint8_t mCompReadAccel(){
	
	uint8_t accelSplitedValues[6];
	
	//Check if X, Y and Z gyro data are available
	uint8_t accelStatus = twiReadOneByte(accelAdd, 0x27);
	if(accelStatus & 0x08){
		//Read gyro values
		while(twiReadMultipleBytes(accelAdd, 0x28, accelSplitedValues, 6) == 0);
		accelValues[0] = ((accelSplitedValues[1] << 8) | (accelSplitedValues[0] & 0xff));
		accelValues[1] = ((accelSplitedValues[3] << 8) | (accelSplitedValues[2] & 0xff));
		accelValues[2] = ((accelSplitedValues[5] << 8) | (accelSplitedValues[4] & 0xff));
		
		return 1;
	}
	
	return 0;
}


/************************************************************************/
/* GYRO INIT                                                            */
/************************************************************************/
void mCompGyroInit(){
	
	while(twiWriteOneByte(gyroAdd, 0x20, 0x0F) == 0); //CTRL1 => 100Hz - Low Pass Filter 25Hz
	while(twiWriteOneByte(gyroAdd, 0x21, 0x00) == 0); //CTRL2 => High pass filter 8Hz to remove some drift
	while(twiWriteOneByte(gyroAdd, 0x23, 0xA0) == 0); //CTRL4 => BDU - Full scale 2000dps
	while(twiWriteOneByte(gyroAdd, 0x24, 0x10) == 0); //CTRL5 => High pass filter enable
	
	/*
	//Get gyro zero rate level
	
	uint8_t gyroSplitedValues[2];
	uint8_t loopCount = 0;
	int16_t gyroValue = 0;
	float totalValue = 0;
	PORTD = 1;
	while(1){//loopCount < 200){
		uint8_t gyroStatus = twiReadOneByte(gyroAdd, 0x27);
		if(gyroStatus & 0x08){
			//Read gyro values
			while(twiReadMultipleBytes(gyroAdd, 0x28, gyroSplitedValues, 2) == 0);
			gyroValue = ((gyroSplitedValues[1] << 8) | (gyroSplitedValues[0] & 0xff));
			totalValue += gyroValue;
			loopCount++;
			if(gyroValue < 3){
				PORTD = 1;
			}
			else{
				PORTD = 0;
			}
		}
	}
	//totalValue /= (float)loopCount;

	PORTD = 0;
	_delay_ms(200);
	
	if(totalValue > 10){
		PORTD = 1;
	}
	
	*/

}

uint8_t mCompReadGyro(){
	
	uint8_t gyroSplitedValues[6];
	
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

/************************************************************************/
/* Calculate angles
dt = loop time in s                                                     */
/************************************************************************/
float mCompCompute(float dt){
	//TODO : IF GYRO > 3 ?
	//X up increase accelX
	//1G => 8191 LSB
	
	float dps = (gyroValues[1] * gyroSensitivity) / 1000.f; //raw value in dps
	pitch += (dps * dt); //New angle
	
	
	float accelX = (float)accelValues[0] / 8191.0f;
	float accelY = (float)accelValues[1] / 8191.0f;
	float accelZ = (float)accelValues[2] / 8191.0f;
	//pitch = (asin(accelX) * 180) / M_PI;	//Simplier function than the next one
	float aPitch = atan(accelX / sqrt(pow(accelY, 2.0f) + pow(accelZ, 2.0f)));
	aPitch = (aPitch * 180) / M_PI;
	
	pitch = (0.8f * pitch) + (0.2f * aPitch);
	
	return pitch;
}