#include "monni_ahrs.h"

//Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
float G_Dt=0.02;

//Temp values use to read gyro and accelerometer values.
uint8_t gyroSplitedValues[6];
uint8_t accelSplitedValues[6];

//Computed values
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t magnetom_x;
int16_t magnetom_y;
int16_t magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

// Euler angles
float roll;
float pitch;
float yaw;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};

float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int8_t SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int8_t SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

int16_t MAN[3];
int16_t AN[6]; //array that stores the gyro and accelerometer data
int32_t AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors gXYZ - aXYZ

int8_t compassCounter = 0;

//Timer related variables
volatile uint32_t t0OvfCount = 0;
uint8_t previousCount = 0;
uint8_t pastCount = 0;

float constrain(float x, float a, float b){
	if(x < a){
		return a;
	}
	else if(x > b){
		return b;
	}
	else{
		return x;
	}
}

//**********************************//
//MATRIX Calculations
//**********************************//

//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;
  
  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
	float op[3]; 
	for(int x=0; x<3; x++)
	{
		for(int y=0; y<3; y++)
		{
			for(int w=0; w<3; w++)
			{
			op[w]=a[x][w]*b[w][y];
			} 
			mat[x][y]=0;
			mat[x][y]=op[0]+op[1]+op[2];
  
			//float test=mat[x][y];
		}
	}
}

void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/

void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(gyro_x); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(gyro_y); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(gyro_z); //gyro Z yaw 
  
  Accel_Vector[0]=accel_x;
  Accel_Vector[1]=accel_y;
  Accel_Vector[2]=accel_z;
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;

 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
	
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}


//**********************************//
//Compute magnetometer's values to calculate the Heading
//**********************************//
void Compass_Heading(){

	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
	c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
	c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
	c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;

	// Tilt compensated Magnetic filed X:
	MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
	// Tilt compensated Magnetic filed Y:
	MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-MAG_Y,MAG_X);

}

//Timer 0 overflow. Every 256us.
ISR(TIMER0_OVF_vect){
	t0OvfCount++;
}

void AhrsInit(){

	//Accel initialisation
	while(twiWriteOneByte(accelAdd, 0x21, 0x18) == 0); //CTRL2 = 0x18 => full scale +/-8g
	while(twiWriteOneByte(accelAdd, 0x20, 0b01010111) == 0); //CTRL1 = 0b01010111 => 50Hz + 3 axis enable
	
	//Magneto initialisation
	while(twiWriteOneByte(accelAdd, 0x24, 0b01100100) == 0); //CTRL5 = 0b01100100 => High resolution and 6.25Hz
	while(twiWriteOneByte(accelAdd, 0x25, 0b00100000) == 0); //CTRL6 = 0b00100000 => +-4 gauss
	while(twiWriteOneByte(accelAdd, 0x26, 0b00000000) == 0); //CTRL7 = 0 => low power off and continuous conversion mode
	
	//Gyro initialisation
	while(twiWriteOneByte(gyroAdd, 0x39, 0b00000000) == 0); //LOW_ODR disabled
	while(twiWriteOneByte(gyroAdd, 0x23, 0x20) == 0); //CTRL4 = 0x20 => 2000dps full scale
	while(twiWriteOneByte(gyroAdd, 0x20, 0x0F) == 0); //CTRL1 = 0x0F => Normal power mode, all axis enabled	
	
	//Wait for stabilisation
	_delay_ms(20);
	
	//Calculate an average offset of sensors
	for(int8_t i = 0 ; i < 32 ; i++){
		
		int16_t sensorsValues[6];
		
		while(twiReadMultipleBytes(gyroAdd, 0x28, gyroSplitedValues, 6) == 0);
		sensorsValues[0] = ((gyroSplitedValues[1] << 8) | (gyroSplitedValues[0] & 0xff));
		sensorsValues[1] = ((gyroSplitedValues[3] << 8) | (gyroSplitedValues[2] & 0xff));
		sensorsValues[2] = ((gyroSplitedValues[5] << 8) | (gyroSplitedValues[4] & 0xff));
		
		while(twiReadMultipleBytes(accelAdd, 0x28, accelSplitedValues, 6) == 0);
		sensorsValues[3] = ((accelSplitedValues[1] << 8) | (accelSplitedValues[0] & 0xff));
		sensorsValues[4] = ((accelSplitedValues[3] << 8) | (accelSplitedValues[2] & 0xff));
		sensorsValues[5] = ((accelSplitedValues[5] << 8) | (accelSplitedValues[4] & 0xff));
		

		
		for(int8_t j = 0 ; j < 6 ; j++){
			AN_OFFSET[j] += sensorsValues[j];
		}
		
		_delay_ms(20); //Wait for values to be refreshed
	}
	
	for(int8_t i = 0 ; i < 6 ; i++){
		AN_OFFSET[i] = AN_OFFSET[i]/32;
	}
	
	AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5]; //ZEROED the Z accelerometer axis (remove gravity)
	
	//8-bits Timer 0 configuration
	TCCR0A = 0; //Normal mode
	TCCR0B |= 1<<CS01; //Prescaling /8 => 1 tick every us
	TIMSK0 |= 1<<TOIE0; //Interrup on overflow (every 256us)
	
}

uint8_t AhrsCompute(){

	//Check if counter overflowed
	uint8_t actualCount = t0OvfCount;
	if(actualCount > previousCount){
		pastCount += actualCount - previousCount;
	}
	else{
		pastCount += (256 - previousCount) + actualCount;
	}

	previousCount = actualCount;

	//Run loop at about 50Hz (20ms) => 1 count = 256us => 78 counts = 20ms
	//**************************
	//INSTEAD OF THAT, I COULD USE THE PMW GENERATOR TO SYNC THE LOOP AND AVOID AN INTERRUPT
	//**************************
	if(pastCount > 78){
	
		compassCounter++;
		
		G_Dt = (pastCount*256)/1000.0; // Real time of loop run. We use this on the DCM algorithm.
		G_Dt /= 1000.0;

		pastCount = 0;
		
		//Read gyro			
		while(twiReadMultipleBytes(gyroAdd, 0x28, gyroSplitedValues, 6) == 0);
		AN[0] = ((gyroSplitedValues[1] << 8) | (gyroSplitedValues[0] & 0xff));
		AN[1] = ((gyroSplitedValues[3] << 8) | (gyroSplitedValues[2] & 0xff));
		AN[2] = ((gyroSplitedValues[5] << 8) | (gyroSplitedValues[4] & 0xff));
		gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
		gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
		gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
		
		//Read accelerometer
		while(twiReadMultipleBytes(accelAdd, 0x28, accelSplitedValues, 6) == 0);
		AN[3] = ((accelSplitedValues[1] << 8) | (accelSplitedValues[0] & 0xff));
		AN[4] = ((accelSplitedValues[3] << 8) | (accelSplitedValues[2] & 0xff));
		AN[5] = ((accelSplitedValues[5] << 8) | (accelSplitedValues[4] & 0xff));
		accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
		accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
		accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
		
		if(compassCounter > 5){
			compassCounter = 0;
			
			//Read compass
			while(twiReadMultipleBytes(accelAdd, 0x08, accelSplitedValues, 6) == 0);
			MAN[0] = ((accelSplitedValues[1] << 8) | (accelSplitedValues[0] & 0xff));
			MAN[1] = ((accelSplitedValues[3] << 8) | (accelSplitedValues[2] & 0xff));
			MAN[2] = ((accelSplitedValues[5] << 8) | (accelSplitedValues[4] & 0xff));
			
			magnetom_x = SENSOR_SIGN[6] * MAN[0];
			magnetom_y = SENSOR_SIGN[7] * MAN[1];
			magnetom_z = SENSOR_SIGN[8] * MAN[2];
			
			//Calculate magnetic heading
			Compass_Heading();				
		}
		
		// Calculations
		Matrix_update(); 	
		Normalize();
		Drift_correction();
		Euler_angles();
		
		return 1;

	}
	
	return 0;
	
}