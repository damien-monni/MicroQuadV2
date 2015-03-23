/*
 * monni_ahrs.h
 *
 * Created: 22/03/2015 16:26:35
 *  Author: monni_000
 */ 


#ifndef MONNI_AHRS_H_
#define MONNI_AHRS_H_

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <math.h>

#include "monni_i2c.h"

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
/*#define M_X_MIN -2566
#define M_Y_MIN -1891
#define M_Z_MIN -2705
#define M_X_MAX 2646
#define M_Y_MAX 2835
#define M_Z_MAX 2177*/

#define M_X_MIN -3281
#define M_Y_MIN -2440
#define M_Z_MIN -4657
#define M_X_MAX 2154
#define M_Y_MAX 2904
#define M_Z_MAX 412

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//Extern values
extern int16_t accel_x;

//7 bits accelerometer's address
static const uint8_t accelAdd = 0b0011101;
//7 bits gyro's address
static const uint8_t gyroAdd = 0b1101011;

//Raw value of gravity. 8g max on 16 signed bits => 1g = 4096.
static const int16_t GRAVITY = 4096;

//**********************************//
//Global variables
//**********************************//

// Euler angles
extern float roll;
extern float pitch;
extern float yaw;

float constrain(float x, float a, float b);

//**********************************//
//MATRIX Calculations
//**********************************//

//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3]);

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);

//Multiply the vector by a scalar.
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!).
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);

void Normalize(void);

/**************************************************/
void Drift_correction(void);

/**************************************************/
/*
void Accel_adjust(void);
*/
/**************************************************/

void Matrix_update(void);

void Euler_angles(void);

//**********************************//
//Compute magnetometer's values to calculate the Heading
//**********************************//
void Compass_Heading();

//Timer 0 overflow. Every 256us.
ISR(TIMER0_OVF_vect);

void AhrsInit();

uint8_t AhrsCompute();



#endif /* MONNI_AHRS_H_ */