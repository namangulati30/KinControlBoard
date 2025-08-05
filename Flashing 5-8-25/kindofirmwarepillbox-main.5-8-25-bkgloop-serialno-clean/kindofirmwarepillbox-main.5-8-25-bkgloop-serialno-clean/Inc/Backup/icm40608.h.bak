
#ifndef _ICM_40608
#define _ICM_40608
#include "main.h"
#include "MadgwickAHRS.h"


#define DEV_ADDR_ICM 0x69<<1

#define WHO_AM_I_ICM 0x75

#define DRIVE_CONFIG 0x13

#define DEVICE_CONFIG 0x11

#define PWR_MGMNT 0x4E

#define TEMP_DATA 0x1D

#define ACCEL_DATA_X1 0x1F

#define ACCEL_DATA_Y1 0x21

#define ACCEL_DATA_Z1 0x23

#define GYRO_DATA_X1 0x25

#define GYRO_DATA_Y1 0x27

#define GYRO_DATA_Z1 0x29

#define GYRO_CONFIG_ICM 0x4F

uint8_t WhoAmIMPU_ICM(void);
uint8_t mpuReadData_ICM(uint8_t regAddr);
void mpuSendData_ICM(uint8_t regAddr, uint8_t regData);
int16_t mpuRead16Data_ICM(uint8_t regAddr);
float mpuTempData_ICM(void);
float mpuAccelX1_ICM(void);
uint8_t ICM40608_init(void);
void ICM40608GetAllReading(void);

#endif
