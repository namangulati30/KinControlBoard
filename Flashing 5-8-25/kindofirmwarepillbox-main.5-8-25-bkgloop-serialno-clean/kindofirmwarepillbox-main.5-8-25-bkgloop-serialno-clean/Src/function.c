#include "function.h"
#include "pn532_stm32f1.h"
const uint16_t X_STP=700, ROT_STP=4600;  //4800
uint8_t PILLBNFC_buff[11];
uint8_t BUFF_NFC[11];
extern uint8_t ReadNFC_PillB;

uint8_t temp_x=1, temp_z=1, temp_y=0, temp_pill=0, Door=3, rot=0;
uint8_t rotPcnt=0, rotYcnt=0, rottol_Prot=7, rottol_y=100;

uint32_t time_init=0, time_z=0, stepstart=0;
extern uint8_t Check_Motor_Z_Steps;
extern uint8_t Check_Motor_X_Steps;
void stperInit(void){
	
	
	HAL_IWDG_Refresh(&hiwdg);
	setdac(ARM_Z);
	time_init = HAL_GetTick();
	while(value_adc[4]>500){
		HAL_IWDG_Refresh(&hiwdg);
		printf(";lkdsafsdbcfbvbv %d\n", HAL_GPIO_ReadPin(ARM_Z_STP1_GPIO_Port, ARM_Z_STP1_Pin));
		printf(";------------------ %d\n", HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin));
		stp = &stepper[ARM_Z];
		startstepper(ARM_Z, 1); //1
		stp->cnt=0;
		if((HAL_GetTick()-time_init)>4000){
			StopStp(ARM_Z);
			protocolActionCallback(CtrlStpZ,0xFF);
			printf("stepper zzzzzzz timeout!!!!!!!!!!!!!");
			break;
		}
	}
	if((HAL_GetTick()-time_init)<=4000){
		temp_z=1;
	}
	StopStp(ARM_Z);
	setdacstop(ARM_Z);
	time_init=HAL_GetTick();
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	printf(";++++++++++++++++++++++++ %d\n", HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin));
	while(temp_z==1 && HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin)/*&& !HAL_GPIO_ReadPin(ARM_Z_STP1_GPIO_Port, ARM_Z_STP1_Pin)*/){	
		stp = &stepper[ARM_X];
		HAL_IWDG_Refresh(&hiwdg);
		printf(";++++++++++++++++++++++++ %d\n", HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin));
//		HAL_GPIO_WritePin(STP5_DIR_GPIO_Port, STP5_DIR_Pin, 1);
//		HAL_GPIO_WritePin(STP5_EN_GPIO_Port, STP5_EN_Pin, 0);
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		startstepper(ARM_X,1);
		stp->cnt=0;
		if((HAL_GetTick()-time_init)>3000){
			//HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
			StopStp(ARM_X);
			printf("stepper xxxxxxxx timeout!!!!!!!!!!!!!");
			protocolActionCallback(CtrlStpX,0xFF);
			
			break;
		}		
	}
	if((HAL_GetTick()-time_init)<=3000){
		temp_x=1;
	}
	
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	StopStp(ARM_X);
	time_init=HAL_GetTick();
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	if(!HAL_GPIO_ReadPin(FRNTDR_CLS_GPIO_Port, FRNTDR_CLS_Pin))
	{
		HAL_GPIO_WritePin(FRNTDR_MAG_GPIO_Port, FRNTDR_MAG_Pin,GPIO_PIN_SET);
	
	setdac(ROT_Y);
	while(HAL_GPIO_ReadPin(CROT_ENCI_GPIO_Port, CROT_ENCI_Pin)){
		stp = &stepper[ROT_Y];
		HAL_IWDG_Refresh(&hiwdg);
		startstepper(ROT_Y, 0);
		stp->cnt=0;
		//printf("PROT_ENCZ: %d\n", HAL_GPIO_ReadPin(PROT_ENCZ_GPIO_Port, PROT_ENCZ_Pin));
		if((HAL_GetTick()-time_init)>10000){
			StopStp(ROT_Y);
			protocolActionCallback(rotateLimCan,0xFF);
			printf("stepper yyyyyyyy timeout!!!!!!!!!!!!!");
			break;
		}
	}
	StopStp(ROT_Y);
	
		setdacstop(ROT_Y);
	rotYcnt=0;
	temp_y=0;
	}
	StopStp(ROT_Y);
	setdacstop(ROT_Y);
	rotYcnt=0;
	temp_y=0;
	HAL_GPIO_WritePin(FRNTDR_MAG_GPIO_Port, FRNTDR_MAG_Pin,GPIO_PIN_RESET);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	setdac(rotProt);
	
	startstepper(rotProt, 1);
	HAL_Delay(300);
	
		time_init=HAL_GetTick();
	while(HAL_GPIO_ReadPin(PROT_ENCZ_GPIO_Port, PROT_ENCZ_Pin)){
		HAL_IWDG_Refresh(&hiwdg);
		stp = &stepper[rotProt];
		startstepper(rotProt, 0);
		stp->cnt=0;
		if((HAL_GetTick()-time_init)>30000){

			StopStp(rotProt);
			protocolActionCallback(rotatePillbox,0xFF);
			printf("pillboxtimeout\n");
			break;
		}
	}
	startstepper(rotProt, 0);
	HAL_Delay(300);
	
	StopStp(rotProt);
	setdacstop(rotProt);
	rotPcnt=0;
	}



void moveARM_Z(uint8_t dir){


	if(dir){
		if(value_adc[4]>500){
			setdac(ARM_Z);
			startstepper(ARM_Z ,1);
			printf("stpZ 11 move\n");
			temp_z=3;	
		}else {
			StopStp(ARM_Z);
			setdacstop(ARM_Z);
		}
//	}else{
//		
//		if(value_adc[4]<760){
//			startstepper(ARM_Z,0);
//			
//			temp_z=3;	
//			printf("stpZ 21 down move\n");
////		}		else {
////			StopStp(ARM_Z);
////			printf("stpZ 22 stop\n");                               line 150 to 162 commented out by Naman on 06/09/23 for pre test
////			setdacstop(ARM_Z);
////			dacZ=0;
////			
////			uint32_t  cnt_z=HAL_GetTick()-time_z;
////			uint16_t stp_z=(cnt_z*9600)/1000;
////			
////			printf("cnt:	%d\n", stp_z);
////			if (temp_z != dir)
////				protocolActionCallback(CtrlStpZ, 2);
////			temp_z=0;	
//				HAL_Delay(1800);
//				StopStp(ARM_Z);
//				protocolActionCallback(CtrlStpZ, 2);
//			
//		}
		
	}
	if(HAL_GetTick()-stepstart>4000 && dir==0){		
		printf("stpZ movearm timeout\n");
		ARM_Z_dir=3;
		//protocolActionCallback(CtrlStpZ, 0xFF);
		StopStp(ARM_Z);
		setdacstop(ARM_Z);
		dacZ=0;
	}
}

void moveARM_Xptp(uint8_t dir){
	if(dir){
		startstepper(ARM_X,1);
		stepstart = HAL_GetTick();
		while(HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin)!=0)
		{
			printf("Moving ARM X to Pick Pos \r\n");
			HAL_IWDG_Refresh(&hiwdg);
			ismove_x=1;
			temp_x=3;
			
			if(HAL_GetTick()-stepstart>3000){		
				ARM_X_dir=3;
				temp_x=3;
				protocolActionCallback(CtrlStpX, 0xFF);
				StopStp(ARM_X);		
				printf("time: %d\n", HAL_GetTick()-stepstart);
				break;
	}
			
		}
		if(ARM_X_dir!=3)
		{
			HAL_Delay(10);
			StopStp(ARM_X);
			stepper[ARM_X].cnt=0;
			//protocolActionCallback(CtrlStpZ,1);
			protocolActionCallback(CtrlStpX,1);
			//stepstart=HAL_GetTick();
			temp_x=1;
			ismove_x=0;
		}
		
		
		
		
//		if(HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin)){
//			HAL_GPIO_WritePin(STP5_DIR_GPIO_Port, STP5_DIR_Pin, 1);
//			HAL_GPIO_WritePin(STP5_EN_GPIO_Port, STP5_EN_Pin, 0);
//			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//		startstepper(ARM_X,1);
//			ismove_x=1;
//			temp_x=3;
//			
//		}else {
//			StopStp(ARM_X);
//			stepper[ARM_X].cnt=0;
//			//protocolActionCallback(CtrlStpZ,1);
//			protocolActionCallback(CtrlStpX,1);
//			stepstart=HAL_GetTick();
//			temp_x=1;
//			ismove_x=0;
//		}
	}else{
		
		//stepstart=HAL_GetTick();
		
		startstepper(ARM_X,0);
		stepstart = HAL_GetTick();
		while(HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin)!=0)
		{
			printf("Moving ARM X to Drop Pos \r\n");
			HAL_IWDG_Refresh(&hiwdg);
			ismove_x=1;
			temp_x=3;
			if(HAL_GetTick()-stepstart>3000){		
				ARM_X_dir=3;
				temp_x=3;
				protocolActionCallback(CtrlStpX, 0xFF);
				StopStp(ARM_X);
		
				printf("time: %d\n", HAL_GetTick()-stepstart);
				break;
	}
			
		}
		if(ARM_X_dir!=3)
		{
			HAL_Delay(10);
			StopStp(ARM_X);
			stepper[ARM_X].cnt=0;
			//protocolActionCallback(CtrlStpZ,1);
			protocolActionCallback(CtrlStpX,0);
			//stepstart=HAL_GetTick();
			temp_x=0;
			ismove_x=0;
		
		}
		
		
		
//		if(HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin)){

//				HAL_GPIO_WritePin(STP5_DIR_GPIO_Port, STP5_DIR_Pin, 0);
//			HAL_GPIO_WritePin(STP5_EN_GPIO_Port, STP5_EN_Pin, 0);
//			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//		startstepper(ARM_X,0);
//			ismove_x=1;
//			temp_x=3;
//		}		else {
//			StopStp(ARM_X);
//			//protocolActionCallback(CtrlStpZ,1);
//			protocolActionCallback(CtrlStpX,(uint16_t) stepper[ARM_X].cnt);
//			stepstart=HAL_GetTick();
//			temp_x=0;		
//			ismove_x=0;
//		}
		
	}
	
//	if(HAL_GetTick()-stepstart>3000){		
//		ARM_X_dir=3;
//		protocolActionCallback(CtrlStpX, 0xFF);
//		StopStp(ARM_X);
//		//setdacstop(ARM_X);
//		
//		printf("time: %d\n", HAL_GetTick()-stepstart);
//	}
	
}

/**/void moveARM_X(uint16_t stp, uint8_t dir){   //2000 per point
	//printf("stp: %d\n", X_STP*stp);
	stepInit(ARM_X, stp);
	startstepper(ARM_X,dir);
	ismove_x=1;
	//protocolActionCallback(ctrlStepperX,dir);
}
/*
void moveARM_Z_pulse(uint16_t stp, uint8_t dir){
	stepInit(ARM_Z, stp);
	startstepper(ARM_Z, dir);
	ismove_z=1;
}
*/
void moveROT_Y(uint16_t i, uint8_t dir){
	rottol_y=i;
	rotYcnt=0;
	
	dacY=1;
	ismove_y=1;
	printf("%d, %d", rottol_y, dir) ;
	startstepper(ROT_Y,dir);
	NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)7) & 0x1FUL));
		
}

void rotateProt(uint32_t i, uint8_t dir){
	//printf("dir: %d\n", i);
	rottol_Prot=i;
	//HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	//stepInit(rotProt, i);
	dacPill=1;
	rotPcnt=0;
	//printf("stp here\n");
	startstepper(rotProt,dir);
	ismove_pill=1;
	//rottol_Prot=i;
}

//void chuteDoor (uint8_t dir){
//	if(dir){
//		if(HAL_GPIO_ReadPin(PILLDR_OPN_GPIO_Port, PILLDR_OPN_Pin)){
//		startstepper(ChuteDoor,0);
//			Door=3;
//		}else {
//			StopStp(ChuteDoor);
//			setdacstop(ChuteDoor);
//			dacDoor=0; 
//			Door=1;
//			protocolActionCallback(ctrlChuteDoor,1);
//			
//		}
//	}else{
//		if(HAL_GPIO_ReadPin(PILLDR_CLS_GPIO_Port, PILLDR_CLS_Pin)){
//		startstepper(ChuteDoor,1);
//			Door=3;
//		}	else {
//			StopStp(ChuteDoor);
//			setdacstop(ChuteDoor);	
//			dacDoor=0; 
//			Door=0;	
//			protocolActionCallback(ctrlChuteDoor,0);
//					
//		}
//	}
///*	if(HAL_GetTick()-stepstart>30000){		
//		protocolActionCallback(ctrlChuteDoor, 0xFF);
//		StopStp(ChuteDoor);
//		setdacstop(ChuteDoor);
//	}
//	*/
//}


void readLimSwitch(void) {
//	uint8_t checkbit[10];
  //dataStream[PILLBX_HALL] = HAL_GPIO_ReadPin(PILLBX_HALL_GPIO_Port, PILLBX_HALL_Pin);
  dataStream[PILLDR_OPN] = HAL_GPIO_ReadPin(PILLDR_OPN_GPIO_Port, PILLDR_OPN_Pin);
  dataStream[PILLDR_CLS] = HAL_GPIO_ReadPin(PILLDR_CLS_GPIO_Port, PILLDR_CLS_Pin);
  //dataStream[PILLBX_DETE] = (uint8_t) VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
  dataStream[FRNTDR_BUTT] = HAL_GPIO_ReadPin(FRNTDR_BUTT_GPIO_Port, FRNTDR_BUTT_Pin);
  dataStream[FRNTDR_CLS] = HAL_GPIO_ReadPin(FRNTDR_CLS_GPIO_Port, FRNTDR_CLS_Pin);
  
  dataStream[ARM_X_STP1] = HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin);
  dataStream[ARM_X_STP2] = HAL_GPIO_ReadPin(ARM_X_STP2_GPIO_Port, ARM_X_STP2_Pin);
  dataStream[ARM_Z_STP1] = HAL_GPIO_ReadPin(ARM_Z_STP1_GPIO_Port, ARM_Z_STP1_Pin);
  dataStream[ARM_Z_STP2] = HAL_GPIO_ReadPin(ARM_Z_STP2_GPIO_Port, ARM_Z_STP2_Pin);
  
  dataStream[PROT_ENCI] = HAL_GPIO_ReadPin(PROT_ENCI_GPIO_Port, PROT_ENCI_Pin);
  dataStream[PROT_ENCZ] = HAL_GPIO_ReadPin(PROT_ENCZ_GPIO_Port, PROT_ENCZ_Pin);
  dataStream[CROT_ENCI] = HAL_GPIO_ReadPin(CROT_ENCI_GPIO_Port, CROT_ENCI_Pin);
  dataStream[CROT_ENCZ] = HAL_GPIO_ReadPin(CROT_ENCZ_GPIO_Port, CROT_ENCZ_Pin);
	
	

}






	
void readNFCSensor (void) {

	SPI1->CR1 &= ~(1<<7);
	MFRC522_Init();
	uint8_t str[4]={0};
	memset(BUFF_NFC,0,9);
	memset(PILLBNFC_buff,0,9);
	for(int i=0;i<11;i++)
	{
		HAL_IWDG_Refresh(&hiwdg);
		 //printf("MFRC522_Request %x%x%x%x\r\n", str[0], str[1], str[2], str[3]);
	if (!MFRC522_Request(PICC_REQIDL, str))
		{
		if (!MFRC522_Anticoll(str))
			{
      //printf("MFRC522_Request %x%x%x%x\r\n", str[0], str[1], str[2], str[3]);
			MFRC522_Read(7,BUFF_NFC);
			
		//printf("BUFF %s \r\n", PILLBNFC_buff);
				break;
			//printf("BUFF %s \r\n", BUFF_NFC);
			}
		
		//printf("MFRC522_Request %x%x%x%x\r\n", str[0], str[1], str[2], str[3]);
		
	}
	HAL_Delay(100);
	}
		PILLBNFC_buff[0] = 11;
		PILLBNFC_buff[1] = nfcid;
		PILLBNFC_buff[2] = 0;
		PILLBNFC_buff[3] = BUFF_NFC[2];
		PILLBNFC_buff[4] = BUFF_NFC[3];
		PILLBNFC_buff[5] = BUFF_NFC[4];
		PILLBNFC_buff[6] = BUFF_NFC[5];
		PILLBNFC_buff[7] = BUFF_NFC[6];
		PILLBNFC_buff[8] = BUFF_NFC[7];
	
	MFRC522_AntennaOff();
		HAL_UART_Transmit(&huart2, PILLBNFC_buff, 9,10);
//			NFCBUF[1] = 0;
//      NFCBUF[2]= 0;
//      NFCBUF[3] = 0;
//      NFCBUF[4] = 0;
//		  NFCBUF[5] = 0;
//			NFCBUF[6] = 0;
			protocolReset();
		printf("BUFF %c%c%c%c%c%c \r\n", PILLBNFC_buff[3],PILLBNFC_buff[4],PILLBNFC_buff[5],PILLBNFC_buff[6],PILLBNFC_buff[7],PILLBNFC_buff[8]);
	ReadNFC_PillB =0;
  //printf("MFRC522_Request %x%x%x%x\r\n", str[0], str[1], str[2], str[3]);
	}


uint8_t check_pillbox(void)
{
	
	nfcid = 2;
	MFRC522_Init();
	uint8_t str[4]={0};
	uint8_t read_tag =0;
	for(int i=0;i<11;i++)
	{
		HAL_IWDG_Refresh(&hiwdg);
	if (!MFRC522_Request(PICC_REQIDL, str))
		{
		read_tag =1;
				break;
	}
	}
	MFRC522_AntennaOff();
	return read_tag;
	
}


uint32_t time_x_start_pick_pill;


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


void Check_STPX_DropP(void)
{
	printf("Checking if we are at drop pos or not \r\n");
	if(HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin))
					{
						time_x_start_pick_pill = HAL_GetTick();
						setdac(ARM_X);
						while(HAL_GPIO_ReadPin(ARM_X_STP1_GPIO_Port, ARM_X_STP1_Pin))
							{
						printf("Checking if we are at drop pos or not \r\n");
						HAL_IWDG_Refresh(&hiwdg);
						startstepper(ARM_X,0);
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

uint8_t active_pill_drop(void)
	{
		Check_STPX_DropP();
		printf(" HE %d \r\n", value_adc[4]);
		if (value_adc[4]>500)
		{
			return 1;
		}
		printf(" HERE NOW \r\n");
		Check_Motor_Z_Steps = 1;
		startstepper(ARM_Z,0);
		while(Check_Motor_Z_Steps) HAL_IWDG_Refresh(&hiwdg);
		Check_Motor_X_Steps=6;
		startstepper(ARM_X,1);
		while(Check_Motor_X_Steps) HAL_IWDG_Refresh(&hiwdg);
		Check_Motor_X_Steps=6;
		startstepper(ARM_X,0);
		while(Check_Motor_X_Steps) HAL_IWDG_Refresh(&hiwdg);
		stepz_move_up();   
		return 0;
	}

void stepz_move_up(void)
{
	while(value_adc[4]>300)
		{
		startstepper(ARM_Z,1);  
		}
		StopStp(ARM_Z); 
	
}
