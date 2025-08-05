/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#pragma arm section rodata = "serial_number"
const uint8_t serial_number[18] = {
    'S', 'N', '0', '0', '0', '0', '0', '1', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#pragma arm section
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* RTC_DateTypeDef gDate; 
 RTC_TimeTypeDef gTime; 

struct rtc_time
{
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
};

struct rtc_time rtc_time;
uint8_t time[10];
uint8_t date[10];

*/

    


uint32_t value_adc[12];
uint32_t lasttime=0, pwtime=0, piltime=0, rottime=0;
uint8_t isonb=0, batterydet=0;
uint8_t isinit1080, isinitMPU6050,mpuerror=0,hdcerror=0;
uint8_t targ_x,targ_z, mcuinit=0, checkpressure=0, checkstate[10],ismove_x=0,ismove_y=0, ismove_z=0,ismove_pill=0,returnstate=0, keepon=0;
uint16_t overtemp=40, overroll=20, overpitch=15, overhum=99;
uint8_t dacZ=0,	dacY=0, dacPill=0, dacDoor=0; 
uint8_t counter_machine_update = 0;
uint8_t counter_IMU_Update = 0;

uint8_t first_pump_value=255;
uint8_t first_motor_value=0;
uint32_t adc_8;
uint16_t adc_8m;
uint8_t NFCBUF[9];
float percentage_change_motor=0;
float percentage_change_pump=0;
float percentage_change_pump_avg=0;
extern uint32_t Motor_X_Steps;
extern uint32_t Motor_Y_Steps;
extern uint16_t Motor_Z_Steps;
extern uint8_t Check_Motor_X_Steps;
extern uint8_t Check_Motor_Y_Steps;
extern uint8_t Check_Motor_Z_Steps;
uint8_t Check_Pill_Picked;
extern uint8_t PickPills1;
extern uint8_t CancelPickPills;
extern uint8_t ReadNFC_PillB;
extern uint8_t EnablePillBoxIRQ;
void PickPills(void);
void Check_STPX_PickP(void);
uint8_t interrupt=0;
uint8_t Battery_plug=1;
uint8_t buff[255];
int32_t uid_len = 0;	
extern int16_t rotatereturn;
uint32_t time_x_start_pick_pill;
extern uint8_t opengate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t ch;
uint8_t ch_r;


int fputc(int c, FILE * f) {
  ch=c;
  HAL_UART_Transmit(&huart6, &ch, 1, 1000);
  return c;
}

int fgetc(FILE * F) {
  HAL_UART_Receive(&huart6, &ch_r, 1, 1000);
  return ch_r;
}

void __delay_ms(int32_t k)
{
	int32_t i,j ;
	for(i=0;i<k;i++)
		for(j=0;j<3000;j++)
		{
			//printf(" hi \n");
		}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim6){
		MadgwickAHRSupdateIMU(xGyro, yGyro, zGyro, xAccl, yAccl, zAccl);
	
	}
	if(htim == &htim1){
		stp = &stepper[3];
		stp->cnt++;
		//stepCtrl(3);
		//debug
	   
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
{
	if (first_pump_value<=21)
	{
	adc_8 += value_adc[0];
	//printf("callback");
	//first_pump_value = 0;
		first_pump_value++;
	}
	else{
		first_pump_value =255;
	}
	if (first_motor_value==1)
	{
	adc_8m = value_adc[2];
	first_motor_value=0;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_USART4_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	

	printf("%s \r\n", serial_number);


//  Flash_Write_NUM(0x08005C10, number);
//  RxVal = Flash_Read_NUM(0x08005C10);

//  Flash_Write_NUM(0x08012000, val);
//  RxVal = Flash_Read_NUM(0x08012000);
	
	
	HAL_GPIO_WritePin(PNEU_PUMP_GPIO_Port, PNEU_PUMP_Pin,GPIO_PIN_RESET );
	HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin,GPIO_PIN_RESET );
	HAL_GPIO_WritePin(FRNTDR_MAG_GPIO_Port, FRNTDR_MAG_Pin,GPIO_PIN_RESET );
	HAL_GPIO_WritePin(FRNTDR_LCK_GPIO_Port, FRNTDR_LCK_Pin,GPIO_PIN_RESET );
	
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	
	//pressure sensor
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)value_adc, 5); //original 4
	HAL_IWDG_Refresh(&hiwdg);
	//power mode
	//HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET) ;
	HAL_GPIO_WritePin(SYS_EN_GPIO_Port, SYS_EN_Pin, GPIO_PIN_SET) ;
	HAL_GPIO_WritePin(GPIOC, CPU_EN_Pin, GPIO_PIN_SET);
	printf("Firmware Version  :  V1.0 \r\n");
	
	//mpu6050
	HAL_TIM_Base_Start_IT(&htim6);
	//isinitMPU6050=mpu6050Init();
	ICM40608_init();
	printf("init \r\n");
	HAL_IWDG_Refresh(&hiwdg);
	//HDC1080 
	isinit1080=HDC1080_Init();
	HAL_IWDG_Refresh(&hiwdg);
	
	

	
	SPI1->CR1 &= ~(1<<7); 
	MFRC522_Init();
	
	
	HAL_IWDG_Refresh(&hiwdg);
	
	for(int i=0;i<5;i++){
		setdacstop(i);
	}
	setdac(ARM_X);
	setdac(ARM_Z);
	setdac(ROT_Y);
	setdac(rotProt);
	
	
	
	
	
	stperInit();
	HAL_IWDG_Refresh(&hiwdg);
	printf("init_stper\n");

	protocolInit();
	printf("init_protocol\n");
	
	protocolActionCallback(resetAll,0xff);
	HAL_UART_Receive_IT(&huart2, rxBuff, rxBuffLen);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 HAL_IWDG_Refresh(&hiwdg);
	HAL_UART_Receive_IT(&huart2, rxBuff, rxBuffLen);
		
		if (counter_machine_update >= 20){
			 sendLimSwitchToA33();	
			 sendMachineStatus(); 
			 counter_machine_update = 0;
		 }
		 counter_machine_update++;
		
		
		if(ARM_Z_dir!= temp_z || (ARM_Z_dir==1 &&(value_adc[4]>500))){
			if(dacZ==0) {
				setdac(ARM_Z);
				dacZ=1;
				stepstart=HAL_GetTick();
			}
			moveARM_Z(ARM_Z_dir);
		}	
			

		if(ARM_X_dir!= temp_x || (ARM_X_dir==1 && HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin)==1)||(ARM_X_dir==0 && HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin)==1)){
			setdac(ARM_X);
			moveARM_Xptp(ARM_X_dir);
			printf("time: %d\n", HAL_GetTick());
		}
		
		if(isZero_y ){
			StopStp(ARM_Z);
			setdacstop(ARM_Z);
			time_init=HAL_GetTick();
			setdac(ROT_Y);
			while(HAL_GPIO_ReadPin(CROT_ENCI_GPIO_Port, CROT_ENCI_Pin)){   
					HAL_IWDG_Refresh(&hiwdg);				
					stp = &stepper[ROT_Y];
					startstepper(ROT_Y, 0);
					stp->cnt=0;
			if((HAL_GetTick()-time_init)>10000){
					StopStp(ROT_Y);
					protocolActionCallback(initrotateCan,0xff);
					printf("stepper xxxxxxxx timeout!!!!!!!!!!!!!");
					break;
					}
			}
		
				StopStp(ROT_Y);
				setdacstop(ROT_Y);
				rotYcnt=0;
				temp_y=0;
				isZero_y=0;
				protocolActionCallback(initrotateCan,0);
		
		}
	
		
		if(isZero_p){
			StopStp(ARM_Z);
			setdacstop(ARM_Z);
			StopStp(rotProt);
				setdac(rotProt);
			startstepper(rotProt, 1);
			HAL_Delay(300);
	
			time_init=HAL_GetTick();
			while(HAL_GPIO_ReadPin(PROT_ENCZ_GPIO_Port, PROT_ENCZ_Pin)){
				HAL_IWDG_Refresh(&hiwdg);
				startstepper(rotProt,0);
				if((HAL_GetTick()-time_init)>30000){
					StopStp(rotProt);
					protocolActionCallback(initpillbox,0xff);
					printf("init pillbox timeout!!!!");
					break;
				}
			}
			HAL_IWDG_Refresh(&hiwdg);
			startstepper(rotProt, 0);
			HAL_Delay(300);
	
			StopStp(rotProt);
			setdacstop(rotProt);
			isZero_p=0;
			protocolActionCallback(initpillbox,1);
		}
		
		
		
	
			
	
		//printf(" Cup detect val %d \r\n", dataStream[PILLBX_DETE]);
		if(value_adc[PILLBOX_DET_ADC]<4){
		dataStream[PILLBX_DETE]=0; //no cup
		} else{
			dataStream[PILLBX_DETE]=1; // cup present
		}
		
		
			if(ismove_y==1){		
				if(HAL_GetTick()-stepstart>30000){		
					printf("stopY1");
					StopStp(ROT_Y);
					setdacstop(ROT_Y);
					ismove_y = 0;
					protocolActionCallback(rotateLimCan, 0xFF);
				}
			}
			
			if(ismove_pill==1){		
				if(HAL_GetTick()-stepstart>30000){		
					protocolActionCallback(rotatePillbox, 0xFF);
					printf("pill timeout\n");
					StopStp(rotProt);
					setdacstop(rotProt);
					ismove_pill=0;                             //added by naman on 31/08/23
				}
			}
		
			if (counter_IMU_Update >= 10 ){
			//ICM40608_init();
			if(WhoAmIMPU_ICM()!=0x39){
//				
				HAL_I2C_DeInit(&hi2c2);
				HAL_Delay(5);
				HAL_I2C_Init(&hi2c2);
				HAL_Delay(5);
				ICM40608_init();
				printf("Hello Resetting I2C \r\n");
			}
			
			ICM40608GetAllReading();
			//HDC1080_Init();
			HADC1080_Display();
			counter_IMU_Update=0;
			if(pitch>overpitch || pitch <-overpitch || 180-fabs(roll)>=overroll){
				//sendctriticalError(tilted);
		}
			
		if(dataStream[getHDC1080]>overtemp){
			sendctriticalError(overTemp);		
		}
	
		if(dataStream[getHDC1080+1]>overhum){
			sendctriticalError(overHumid);		
		}
		 }
			counter_IMU_Update++;
			
			if(PickPills1==1)
			{
				interrupt=0;
				PickPills();
				if(Check_Pill_Picked==1) 
				{
					buff[0] = PickPillh;
					buff[1] = dispense_check;
					buff[2] = 0;
					buff[3] = 0;
//				buff[4] = Motor_Z_Steps>>8;
//				buff[5] = Motor_Z_Steps;
					buff[4] =0;
					buff[5] = 0;
					printf("Successful \r\n");
					printf("Motor Z Steps %d \r\n", Motor_Z_Steps);
				HAL_UART_Transmit(&huart2, buff, 6,10);
				protocolReset();
				
				}
				else
				{
						Check_STPX_PickP();
				
				if(interrupt)
				{
					if(interrupt==5)
					{
					protocolActionCallback(CancelPillh, 1);	
					}
					else
					{
					buff[0] = PickPillh;
					buff[1] = dispense_check;
					buff[2] = 0;
					buff[3] = interrupt;
					buff[4] = 0;
					buff[5] = 0;
					HAL_UART_Transmit(&huart2, buff, 6,10);
					protocolReset();
					}
						
				}
				
				else
					{
					buff[0] = PickPillh;
					buff[1] = 255;
					buff[2] = 0;
					buff[3] = 0;
					buff[4] = 0;      
//				buff[4] = 0;        add a unique identifier to denote pill not picked in this byte							
					buff[5] = 0;
					HAL_UART_Transmit(&huart2, buff, 6,10);
					protocolReset();
					}
				}
		 PickPills1=0;
		 temp_x=3;
		 ARM_X_dir =3;
			}
	
		if((PickPills1 == 0) && (CancelPickPills ==1))
		{
			protocolActionCallback(CancelPillh,1);
			CancelPickPills = 0;
		}
		
		
	if(EnablePillBoxIRQ==1)
	{
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
		EnablePillBoxIRQ=0;
	}
	if(ReadNFC_PillB ==1)
	{
		readNFCSensor();
	}
	if(opengate==1)
	{
		HAL_GPIO_WritePin(FRNTDR_LCK_GPIO_Port, FRNTDR_LCK_Pin,GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(FRNTDR_LCK_GPIO_Port, FRNTDR_LCK_Pin,GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(FRNTDR_LCK_GPIO_Port, FRNTDR_LCK_Pin,GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(FRNTDR_LCK_GPIO_Port, FRNTDR_LCK_Pin,GPIO_PIN_RESET);
		opengate = 0;
		protocolActionCallback(frntLock,1);
	}
	
}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	if(GPIO_Pin == PROT_ENCI_Pin &&  HAL_GetTick()-piltime>2000 ){//){
		piltime=HAL_GetTick();
		//printf(" In the callback func  also ismove_pill %d \r\n",ismove_pill);
		if(ismove_pill==1){
			rotPcnt++;
		//	printf("Going still \r\n");
		}
		printf("c:%d t:	%d\n",rotPcnt, piltime);
		if(rotPcnt>=rottol_Prot){
			StopStp(rotProt);
			//printf("Stop now \r\n");
			HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
			ismove_pill=0;
			rotPcnt=0;
			piltime=HAL_GetTick();
			buff[0] = rotatePillbox;
			buff[1] = rotatereturn>>8;
			buff[2] = rotatereturn;
			HAL_UART_Transmit(&huart2, buff, 3, 10);
			protocolReset();
			//protocolActionCallback(rotatePillbox,rotatereturn);
		}
	}/**/
		/**/
		if(GPIO_Pin == CROT_ENCZ_Pin && HAL_GetTick()-rottime>70){
			rottime=HAL_GetTick();
			//printf("cnt: %d\n time:	%d\n",rotYcnt, rottime);
			if(ismove_y==1){
			rotYcnt++;
		}else;
			if(rotYcnt>=rottol_y/*+1*/){
				StopStp(ROT_Y);
				
				HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
				buff[0] = rotateLimCan;
				buff[1] = rotatereturn>>8;
				buff[2] = rotatereturn;
				HAL_UART_Transmit(&huart2, buff, 3, 10);
				protocolReset();
				//protocolActionCallback(rotateLimCan,temp_y);
				//ismove_y=3;
				ismove_y=0;
				rottime=HAL_GetTick();
				rotYcnt=0;
				rottol_y=0;
			}
		}
		
		if(GPIO_Pin == CE_MCU_WAKEUP_Pin)
		{
			if(Battery_plug==1)
		{
			Battery_plug=0;
			printf(" Battery out\r\n");
		}
		else{
			Battery_plug=1;
			printf(" Battery in\r\n");
		}
		}
	}



void PickPills()
{
	Check_STPX_PickP();
	HAL_Delay(300);
	percentage_change_pump = 0;
	percentage_change_pump_avg = 0;
	adc_8 = 0;
	Motor_Z_Steps = 0;
	printf("Start Pick Pills \r\n");
	setdac(ARM_X);
	setdac(ARM_Z);
	
	first_pump_value=1;
	while(first_pump_value!=255)
		{
			HAL_IWDG_Refresh(&hiwdg);
		}
		adc_8 = adc_8/20;
		printf( "Pump avg val %d \r\n", adc_8);
		HAL_Delay(200);
	
	percentage_change_pump =0;
	percentage_change_pump_avg =0;
	for(int i=0;i<=8;i++)
	{
	
		if((i>=1) && (percentage_change_pump>=8.0))
	{
			
			printf("Percentage Change Pump detected %f \r\n",percentage_change_pump);
			percentage_change_pump =0;
			percentage_change_pump_avg =0;

			for(int i =0;i<20;i++)
	{
		percentage_change_pump = abs(value_adc[0]-adc_8);
		percentage_change_pump = (percentage_change_pump/adc_8)*100;
		percentage_change_pump_avg = percentage_change_pump + percentage_change_pump_avg;

		HAL_Delay(10);
	}
	percentage_change_pump_avg = percentage_change_pump_avg/21;

	if(percentage_change_pump_avg>=8.0)
	{
		
		Check_Pill_Picked=1;
		printf("Perc change First Check %f \r\n",percentage_change_pump_avg);
			break;
		
	}
	else
		{
		adc_8 = 0;
		HAL_GPIO_WritePin(PNEU_PUMP_GPIO_Port, PNEU_PUMP_Pin,GPIO_PIN_RESET );
		HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin,GPIO_PIN_SET );
		HAL_Delay(100);
		HAL_GPIO_WritePin(PNEU_PUMP_GPIO_Port, PNEU_PUMP_Pin,GPIO_PIN_SET );
		HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin,GPIO_PIN_RESET );
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(500);
		HAL_IWDG_Refresh(&hiwdg);
			
	
		first_pump_value=1;
			while(first_pump_value!=255)
		{
			HAL_IWDG_Refresh(&hiwdg);
		}
		adc_8 = adc_8/20;
		printf( "Pump avg val %d \r\n", adc_8);

	}
			
		}
		else
		{
			if(i>=1)
			{
		adc_8 = 0;
		HAL_GPIO_WritePin(PNEU_PUMP_GPIO_Port, PNEU_PUMP_Pin,GPIO_PIN_RESET );
		HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin,GPIO_PIN_SET );
		HAL_Delay(100);
		HAL_GPIO_WritePin(PNEU_PUMP_GPIO_Port, PNEU_PUMP_Pin,GPIO_PIN_SET );
		HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin,GPIO_PIN_RESET );
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(500);
		HAL_IWDG_Refresh(&hiwdg);
			
	
		first_pump_value=1;
			while(first_pump_value!=255)
		{
			HAL_IWDG_Refresh(&hiwdg);
		}
		adc_8 = adc_8/20;
		printf( "Pump avg val %d \r\n", adc_8);
			}
		Check_Pill_Picked=0;
		Motor_Z_Steps = 0;
		}
		

	if(CancelPickPills==1)
			{
				Check_Pill_Picked=0;
		 CancelPickPills=0;
			printf("Interrupt CancelPickPill \r\n");
			interrupt=5;
				
				break;
			}
		if(dispense_check ==1)
		{
		if(value_adc[PILLBOX_DET_ADC]<4)
		{
			Check_Pill_Picked=0;
			printf("Interrupt Cup\r\n");
			interrupt=2;
			break;
		}
	}
		else if(dispense_check ==2)
		{
			if(check_pillbox() == 0)
			{
			Check_Pill_Picked=0;
			printf("Interrupt PillB\r\n");
			interrupt=2;
			break;
			}
		}
			
			
		if(HAL_GPIO_ReadPin(FRNTDR_CLS_GPIO_Port, FRNTDR_CLS_Pin)==1)
		{
			Check_Pill_Picked=0;
			printf("Interrupt Gate \r\n");
			interrupt=3;
				break;
		}	
		ICM40608GetAllReading();
//		if(pitch>overpitch || pitch <-overpitch || 180-fabs(roll)>=overroll)
//			{
//		Check_Pill_Picked=0;
//				printf("Interrupt tilt \r\n"); 
//				interrupt=4;
//				break;
//			}
			
			if(i==1)
		{
		Check_Motor_X_Steps=1;
		startstepper(ARM_X,0);
		printf("1 \r\n");
			
	
		}
			
			if(i==2)
		{
		Check_Motor_X_Steps=1;
		startstepper(ARM_X,0);
		printf("1 \r\n");
			
	
		}
		if(i==3)
		{
			printf("2 \r\n");
			Check_Motor_X_Steps=2;
			startstepper(ARM_X,0);
		}
		
		if(i==4)
		{
			printf("3 \r\n");
			Check_Motor_X_Steps=3;
			startstepper(ARM_X,1);
			setdac(ROT_Y);
			Check_Motor_Y_Steps=1;
			startstepper(ROT_Y,0);
			}
			
			
		
			if(i==5)
		{
			printf("4 \r\n");
			setdac(ROT_Y);
			Check_Motor_Y_Steps=1;
			startstepper(ROT_Y,0);
		}
		
		if(i==6)
		{
			
			printf("5 \r\n");
			Check_Motor_Y_Steps=3;
			startstepper(ROT_Y,1);
			
			
			
		}
		if(i==7)
		{
		printf("7 \r\n");
		Check_Motor_Y_Steps=1;
		startstepper(ROT_Y,1);
		}
		if(i==8)
		{
			Check_Motor_Y_Steps=2;
		startstepper(ROT_Y,0);
		}
		
		Check_Motor_Z_Steps =1;
		startstepper(ARM_Z,0);
		uint32_t Motor_Start_time=HAL_GetTick();
		uint32_t Motor_Current_time=HAL_GetTick();
		HAL_Delay(400);
		first_motor_value=1;
		while(first_motor_value){
			HAL_IWDG_Refresh(&hiwdg);
		}
	uint8_t bottom_reached_flag=0;
	uint8_t j=0;
	percentage_change_pump = 0;
	percentage_change_motor = 0;

	while( j == 0)
	{
	HAL_IWDG_Refresh(&hiwdg);
	
	percentage_change_motor = abs(value_adc[2]-adc_8m);
	percentage_change_pump = abs(value_adc[0]-adc_8);
	percentage_change_motor = (percentage_change_motor/adc_8m)*100;	
	percentage_change_pump = (percentage_change_pump/adc_8)*100;

	if(((percentage_change_motor>15) || (percentage_change_pump>8.0) || (Motor_Current_time-Motor_Start_time>2200)) && (bottom_reached_flag==0))     //(percentage_change_motor>7) and (Motor_Current_time-Motor_Start_time>4000)
		{
		Check_Motor_Z_Steps = 0;
		StopStp(ARM_Z);   
		startstepper(ARM_Z,1); 
		bottom_reached_flag=1;
		HAL_Delay(3);
		printf("SHould Start Going Up now \r\n");
		}
		if ((bottom_reached_flag==1) && (value_adc[4]<500))
		{ 
		
		printf("TRUE NOW STOP \r\n");
		bottom_reached_flag=0;
		StopStp(ARM_Z);                   
		j=1;
	}
	Motor_Current_time=HAL_GetTick();
	
	}
	}
	if(Check_Pill_Picked==0)
	{
	percentage_change_pump =0;
	percentage_change_pump_avg =0;

		for(int i =0;i<20;i++)
	{
		percentage_change_pump = abs(value_adc[0]-adc_8);
		percentage_change_pump = (percentage_change_pump/adc_8)*100;
		percentage_change_pump_avg = percentage_change_pump + percentage_change_pump_avg;
\
		HAL_Delay(10);
	}
	percentage_change_pump_avg = percentage_change_pump_avg/21;

	
	if(percentage_change_pump_avg>=8.0)
		{
			Check_Pill_Picked=1;
			
		}
	}
	///////////////////////////////////////////////////////
	printf("Here Now \r\n");
	if(Check_Pill_Picked==1)
	{
	
		while(value_adc[4]>300)
	{
		startstepper(ARM_Z,1);  
	}
	StopStp(ARM_Z);    
		time_x_start_pick_pill = HAL_GetTick();
	
	
	
	
		while(HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin)){
		if(CancelPickPills==1)
			{
				Check_Pill_Picked=0;
		 CancelPickPills=0;
				printf("Interrupt CancelPickPill \r\n");
			interrupt=5;
				
				break;
			}
	if(dispense_check ==1)
		{
		if(value_adc[PILLBOX_DET_ADC]<4)
		{
			Check_Pill_Picked=0;
			printf("Interrupt Cup\r\n");
			interrupt=2;
			break;
		}
	}
		else if(dispense_check ==2)
		{
			if(check_pillbox() == 0)
			{
			Check_Pill_Picked=0;
			printf("Interrupt PillB\r\n");
			interrupt=2;
			break;
			}
		}
		if(HAL_GPIO_ReadPin(FRNTDR_CLS_GPIO_Port, FRNTDR_CLS_Pin)==1)
		{
			Check_Pill_Picked=0;
			printf("Interrupt Gate \r\n");
			interrupt=3;
				break;
		}	
//		ICM40608GetAllReading();
//		if(pitch>overpitch || pitch <-overpitch || 180-fabs(roll)>=overroll)
//			{
//		Check_Pill_Picked=0;
//				printf("Interrupt tilt \r\n"); 
//				interrupt=4;
//				break;
//			}
//		
//	
		printf("Checking if we are at pick pos or not \r\n");
		HAL_IWDG_Refresh(&hiwdg);
		startstepper(ARM_X,1);
		if((HAL_GetTick()-time_x_start_pick_pill)>3000){
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
			break;
		}		
	}
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
		Check_Motor_X_Steps = 5;
		startstepper(ARM_X,0);
	
	
	while(Check_Motor_X_Steps!=0)
	{
		if(CancelPickPills==1)
			{
				Check_Pill_Picked=0;
		 CancelPickPills=0;
				printf("Interrupt CancelPickPill \r\n");
			interrupt=5;
				
				break;
			}
		if(dispense_check ==1)
		{
		if(value_adc[PILLBOX_DET_ADC]<4)
		{
			Check_Pill_Picked=0;
			printf("Interrupt Cup\r\n");
			interrupt=2;
			break;
		}
	}
		else if(dispense_check ==2)
		{
			if(check_pillbox() == 0)
			{
			Check_Pill_Picked=0;
			printf("Interrupt PillB\r\n");
			interrupt=2;
			break;
			}
		}
		if(HAL_GPIO_ReadPin(FRNTDR_CLS_GPIO_Port, FRNTDR_CLS_Pin)==1)
		{
			Check_Pill_Picked=0;
			printf("Interrupt Gate \r\n");
			interrupt=3;
				break;
		}	
//		ICM40608GetAllReading();
//		if(pitch>overpitch || pitch <-overpitch || 180-fabs(roll)>=overroll)
//			{
//		Check_Pill_Picked=0;
//				printf("Interrupt tilt \r\n"); 
//				interrupt=4;
//				break;
//			}
		
	
	}
	percentage_change_pump =0;
	percentage_change_pump_avg =0;

		for(int i =0;i<20;i++)
	{
		percentage_change_pump = abs(value_adc[0]-adc_8);
		percentage_change_pump = (percentage_change_pump/adc_8)*100;
		percentage_change_pump_avg = percentage_change_pump + percentage_change_pump_avg;

		HAL_Delay(10);
	}
	percentage_change_pump_avg = percentage_change_pump_avg/21;
	printf("PickPill Pill PressureSensor %f \r\n", percentage_change_pump_avg);
	if(percentage_change_pump_avg>=8.0)
		{
			
			Check_Pill_Picked = 1;
			percentage_change_pump = 0;
			percentage_change_pump_avg = 0;
			adc_8 = 0;
			
		}
		else
		{
			Check_Pill_Picked=0;
		}

	}

	percentage_change_pump = 0;
	percentage_change_pump_avg = 0;
	adc_8 = 0;
	setdacstop(ARM_X);
	setdacstop(ARM_Z);
	setdacstop(ROT_Y);

}

void Check_STPX_PickP(void)
{
	printf("Checking if we are at pick pos or not \r\n");
	if(HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin))
					{
						time_x_start_pick_pill = HAL_GetTick();
						setdac(ARM_X);
						while(HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin))
							{
						printf("Checking if we are at pick pos or not \r\n");
						HAL_IWDG_Refresh(&hiwdg);
						startstepper(ARM_X,1);
						if((HAL_GetTick()-time_x_start_pick_pill)>3000)
							{
							HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
								setdacstop(ARM_X);
								break;
								
								}		
							}
							HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
							setdacstop(ARM_X);
					}
	
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
