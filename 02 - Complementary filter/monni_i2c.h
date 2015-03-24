/*
 * monni_i2c.h
 *
 * Created: 22/03/2015 16:27:42
 *  Author: monni_000
 */ 


#ifndef MONNI_I2C_H_
#define MONNI_I2C_H_



#include <avr/io.h>

//Wait for the interrupt flag to be set
void twiWaitFlag();

//Get status code
uint8_t twiGetStatus();

//Write a byte
void twiWriteByte(uint8_t byte);


//*************************************
//USEFULL FUNCTIONS BELLOW
//*************************************

//Initialize a TWI communication sending a Start and SLA+R/W
//Return the status code
uint8_t twiInit(uint8_t slaveAddress, uint8_t isRead);

//Write only one byte
//Return 1 if OK, 0 if error
uint8_t twiWriteOneByte(uint8_t slaveAddress, uint8_t slaveRegister, uint8_t data);

//Read only one byte
//Return the read value
uint8_t twiReadOneByte(uint8_t slaveAddress, uint8_t slaveRegister);

//Read multiple bytes
//Return 1 if OK, 0 if error
uint8_t twiReadMultipleBytes(uint8_t slaveAddress, uint8_t slaveRegister, uint8_t result[], uint8_t nbBytes);




#endif /* MONNI_I2C_H_ */