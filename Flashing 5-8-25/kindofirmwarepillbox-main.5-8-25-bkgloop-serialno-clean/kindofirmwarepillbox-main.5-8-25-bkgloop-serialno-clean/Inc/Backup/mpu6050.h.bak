


#ifndef _MPU6050_H
#define _MPU6050_H

#include <main.h>
#include "MadgwickAHRS.h"


#define DEV_ADDR      0xD0 // 0x68<<1
//----------------------------------------- 
// MPU6050
//----------------------------------------- 
#define SMPLRT_DIV    0x19
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C

/*  */
#define ACCEL_XOUT_H 0x3B 
#define ACCEL_XOUT_L 0x3C 
#define ACCEL_YOUT_H 0x3D 
#define ACCEL_YOUT_L 0x3E 
#define ACCEL_ZOUT_H 0x3F 
#define ACCEL_ZOUT_L 0x40 

/*  */
#define TEMP_OUT_H  0x41 
#define TEMP_OUT_L  0x42 

/*  */
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44 
#define GYRO_YOUT_H 0x45 
#define GYRO_YOUT_L 0x46 
#define GYRO_ZOUT_H 0x47 
#define GYRO_ZOUT_L 0x48 

#define PWR_MGMT_1  0x6B
#define WHO_AM_I    0x75 
#define SlaveAddress 0xD0 


void    mpu6050SendData(uint8_t regAddr, uint8_t regData);
uint8_t mpu6050ReadData(uint8_t regAddr);


uint8_t WhoAmIMPU(void);
uint8_t mpu6050Init(void);

int16_t mpu6050Read16Data(uint8_t regAddr);

void mpu6050GetAllReading(void);

void mpu6050Display(void);

void mpu6050Accl(void);
void mpu6050Gyro(void);


extern float xAccl;
extern float yAccl;
extern float zAccl;
extern float xGyro;
extern float yGyro;
extern float zGyro;
extern int  xMag;
extern int  yMag;
extern int  zMag;

void getBmx055(void);

extern uint8_t temperature, humidity; 

uint8_t HDC1080_Init(void);
uint8_t WhoAmIHDC(void);
int16_t HDC1080_Get_Data(uint8_t regAddr);
void HADC1080_Display(void);


#endif


