#include "icm40608.h"

uint8_t ICM40608_init()
{
	uint8_t datatrandsfer = 0x01;
	mpuSendData_ICM(DEVICE_CONFIG,datatrandsfer);
	HAL_Delay(20);
	
	 datatrandsfer = 0x08;
	mpuSendData_ICM(DRIVE_CONFIG,datatrandsfer);
	
	datatrandsfer = 0x01;
	mpuSendData_ICM(DEVICE_CONFIG,datatrandsfer);
	
	HAL_Delay(20);
	
	datatrandsfer = 0x1F;
	mpuSendData_ICM(PWR_MGMNT,datatrandsfer);

	return 1;

}

uint8_t WhoAmIMPU_ICM()
	{
	
	return mpuReadData_ICM(WHO_AM_I_ICM);
	
}
	

uint8_t mpuReadData_ICM(uint8_t regAddr) {
  uint8_t data[1] = {regAddr};
  uint8_t regData;
  HAL_I2C_Master_Transmit(&hi2c2, DEV_ADDR_ICM, data, 1, 10);
  HAL_I2C_Master_Receive(&hi2c2, DEV_ADDR_ICM, &regData, 1, 10);
  return regData;
}

void mpuSendData_ICM(uint8_t regAddr, uint8_t regData) {
  uint8_t data[2] = {regAddr, regData};
  HAL_I2C_Master_Transmit(&hi2c2, DEV_ADDR_ICM, data, 2, 10);
}

int16_t mpuRead16Data_ICM(uint8_t regAddr) {
  uint8_t Data_H, Data_L;
  uint16_t data;
  
  Data_H = mpuReadData_ICM(regAddr);
	HAL_Delay(10);
  Data_L = mpuReadData_ICM(regAddr + 1);
  data = (Data_H << 8) | Data_L;  // a
  return data;
}

float mpuTempData_ICM(){
	float temp;
	int16_t temperature = mpuRead16Data_ICM(TEMP_DATA);
	temp = (float)(((int16_t)mpuRead16Data_ICM(TEMP_DATA)/132.48) + 25);
	return temp;
}
float mpuAccelX1_ICM(){
	float temp;
	temp = (float)mpuRead16Data_ICM(GYRO_DATA_Y1)/16.4;
	return temp;
}

void ICM40608GetAllReading(void){
	xAccl = (float)mpuRead16Data_ICM(ACCEL_DATA_X1)/2048;	//*9.81;
	yAccl = (float)mpuRead16Data_ICM(ACCEL_DATA_Y1)/2048;	//*9.81;
	zAccl = (float)mpuRead16Data_ICM(ACCEL_DATA_Z1)/2048;	//*9.81;
	xGyro = (float)mpuRead16Data_ICM(GYRO_DATA_X1)/16.4;
	yGyro = (float)mpuRead16Data_ICM(GYRO_DATA_Y1)/16.4;
	zGyro = (float)mpuRead16Data_ICM(GYRO_DATA_Z1)/16.4;
	computeAngles();
//	printf("Pitch %.2f \r\n", pitch);
//	printf("Yaw %.2f \r\n", yaw);
//	printf("Roll %.2f \r\n", roll);
}

