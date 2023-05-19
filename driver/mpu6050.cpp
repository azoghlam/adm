#include<WiringPi.h>
#include<wiringPiI2C.h>

#define _USE_MATH_DEFINES 

#include <stdlib.h> 
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <math.h>


#define Device_Address 0x68 

#define PWR_MGMT_1   0X6B 
#define SMPLRT_DIV   0X19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0X1c
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0X3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0X3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47




int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, filtacc, filtacc0, filtacc2, filtacc1 ;
float acc_angle[2];
float gyro_angle[2];
float tot_angle[2];

float elapsedtime, time, timeprev ;
float conv = 180/3.141592654 ;

int fd ;

short read_raw_data(int addr) {

short high_byte, low_byte, value ;
high_byte = wiringPiI2CReadReg8(fd, addr) ;
low_byte = wiringPiI2CReadReg8(fd, addr+1) ;
value = (high_byte << 8) | low_byte ;
return value ;
}



void update() {
 
 timeprev = time ;
 time = millis() ;
 elapsedtime = (time - timeprev) / 1000 ; 
//// accelerometer raw data //////////////////////////////////
accX = read_raw_data(ACCEL_XOUT_H);
accY = read_raw_data(ACCEL_YOUT_H);
accZ = read_raw_data(ACCEL_ZOUT_H);

////accelerometer X///////////
  acc_angle[0] = atan2((accY/16384.0)/sqrt(pow((accX/16384.0),2) + pow((accZ/16384.0),2)))*conv ;
////accelerometer Y///////////
  acc_angle[1] = atan2(-1*(accX/16384.0)/sqrt(pow((accY/16384.0),2) + pow((accZ/16384.0),2)))*conv;
////accelerometer filter//////  
  filtacc0 = 0.9*filtacc + 0.1*acc_angle[0]  ;
  filtacc2 = 0.9*filtacc1 + 0.1* acc_angle[1] ;

/// GYRO RAW DATA /////  
gyroX = read_raw_data(GYRO_XOUT_H);
gyroY = read_raw_data(GYRO_YOUT_H);
gyroZ = read_raw_data(GYRO_ZOUT_H);
///GYRO  X///////////
  gyro_angle[0]=gyroX/131.0;
////GYRO  Y//////////
  gyro_angle[1]=gyroY/131.0;
////GYRO  Z//////////
  gyro_angle[2]=gyroZ/131.0;

  
  /////////////////////////////// complementery filter //////////////////////
/*---X axis angle---*/
tot_angle[0]=0.96*(tot_angle[0] + gyro_angle[0]*elapsedtime) + 0.04* filtacc0 ;
/*---Y axis angle---*/
tot_angle[1]=0.96*(tot_angle[1] + gyro_angle[1]*elapsedtime) + 0.04* filtacc2 ;
/*---Z axis angle---*/
tot_angle[2] =  gyro_angle[2] ;
}

void init_MPU () {


wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
wiringPiI2CWriteReg8 (fd, CONFIG, 0x00);		/* Write to Configuration register */
wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
// ^^^ was 24
// wiringPiI2CWriteReg8 (fd, ACCEL_CONFIG,0x00); 
wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register ???*/  
gyro_angle[0] = 0 ;
gyro_angle[1]= 0 ;

update();

}