/*
 * Copyright (c) 2020 PeanutKing Solution
 *
 * @file        protocol.c
 * @summary     Kin pill dispenser stm32 android protocol
 * @version     1.0
 * @author      Jack Kwok
 * @data        1 August 2020
 */



#include "protocol.h"

// Variables
uint8_t addr;

uint8_t dataStream[256];
uint8_t txBuff[50];
uint8_t txBuffLen;
uint8_t rxBuff[10];
uint8_t rxBuffIdx;
uint8_t rxBuffLen;
protocolStatus protocolState;
uint8_t ARM_Z_dir=1, ARM_X_dir=1, isLimit_y=0, CDoor=0,ispick=0, isZero_y=0, isZero_p=0, pnpX=1, pnpZ=1, lockcnt=0, depthsense=0;
uint32_t checktime;
uint8_t funcIdx = 0, Ydir=1, Pdir=1, Xdir=1, Zdir=1;
int16_t rotate=0, moveXcnt=0, moveZcnt=0,rotatereturn=0;
parameter_t para;
uint8_t PickPills1=0;
uint8_t CancelPickPills=0;
uint8_t ReadNFC_PillB=0;
uint8_t EnablePillBoxIRQ=0;
uint8_t ActiveDropEn = 0;
extern uint8_t Battery_plug;
extern uint8_t serial_number[18]; 
uint8_t dispense_check;
uint8_t opengate;
func_t func1[funcListLen] = {
  {initrotateCan      , STP_Y, 2},
  {rotatePillbox , STP_PILLBX,  2},
  {ctrlPump      , PNEU_PUMP,   1},
  {ctrlLED       , PANEL_LED0,  5},
  {ctrlGate      , FRNTDR_CTRL, 1},
  
  {readAllLimits , PILLBX_HALL, 14},
  {readAllSensors, getMPU6050,  10},
  
  {readNFC       , NFC_DATA,    1},
  {checkMachineStatus, STP_X,   1},
  
  {CtrlStpX, STP_X, 1},
  {CtrlStpZ, STP_Z, 2},
  {rotateLimCan   , STP_Y, 2},
	
  {ctriticalError , STP_X, 4},
  {PressureSensor  , STP_X, 0},
	{initpillbox   , STP_X, 1},
	{updateParameter ,STP_X, 8},
  {returnPills ,STP_X, 1},
	
	{ctrlValve, STP_Y, 1},
	{cntLock, STP_Y, 1},
	{frntLock, STP_Y, 1},
	
	
	{batteryoff, STP_Y,1},
	{Nothingh, Nothing, 1},
	{PickPillh, PickPill, 1},
	{CancelPillh, CancelPill, 1},
	{Serialnumber, STP_Y, 1},
	{ActiveDrop, STP_Y, 1}
};


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
//		/**/if(ispick==1){
//				printf("adc0:	%d\n", value_adc[PNEU_ADC]);	
//				if (value_adc[PNEU_ADC]>700){
//					
//					sendctriticalError(pilldrop);
//					ispick=0;
//				}
//			}
		
    printf("RAW:   %d, \n", rxBuff[0]);
		dataAnalysis();
	
		
    HAL_UART_Receive_IT(huart, rxBuff, rxBuffLen);
		

  }
}

void sendDataToA33() {
  HAL_UART_Transmit(&huart2, dataStream + func1[funcIdx].r, txBuffLen,10);
}

void protocolInit(void) {
  protocolState = Idle;
  rxBuffLen = 1;
}

void protocolReset(void) {
  protocolState = Idle;
  rxBuffLen = 1;
}

uint16_t CalcCRC16(uint8_t* buf, uint16_t len)
{
    uint16_t poly = 0xA001;
    uint16_t crc = 0;
    for(uint16_t b = 0; b < len; b++) {
        uint8_t v = *buf;
        for(uint8_t i = 0; i < 8; i++) {
            if((v & 0x01) ^ (crc & 0x01)) {
                crc >>= 1;
                crc ^= poly;
            }
            else {
                crc >>= 1;
            }
            v >>= 1;
        }
        buf++;
    }
    return crc;
}

uint16_t data16[10] = {0};

void protocolActionCallback(uint8_t addr, uint16_t data) {
	//uint8_t blank[3]={0, 0, 0};
  //HAL_UART_Transmit(&huart2, blank, 3,10);
//	printf("pillbox commacnd check number add before  :  %d\r\n", addr);
//	printf("pillbox commacnd check number tx buf before  :  %d\r\n", txBuff[0]);
	if(data==0xff){
			printf("255error");
	}
	txBuff[0] = addr;
	txBuff[1] = data&0xff;
	txBuff[2] = data>>8;

  printf("success:  %d\n",HAL_UART_Transmit(&huart2, txBuff, 3, 1000));
  protocolReset();
}
void protocolDataWrong(uint8_t addr, uint16_t data) {
	//uint8_t blank[3]={0, 0, 0};
  //HAL_UART_Transmit(&huart2, blank, 3,10);
//	printf("pillbox commacnd check number add before  :  %d\r\n", addr);
//	printf("pillbox commacnd check number tx buf before  :  %d\r\n", txBuff[0]);
	txBuff[0] = addr;
	txBuff[1] = data&0xff;
	txBuff[2] = 0xDF;
	//printf("feedback:  \r\n" );
//	printf("pillbox commacnd check number add after :  %d\r\n", txBuff[0]);
//	printf("pillbox commacnd check number tx buf after :  %d\r\n", addr);
	//HAL_UART_Transmit(&huart6, txBuff, 3,1000);
	//printf("DataWrong:  %d %d %d \r\n",txBuff[0], txBuff[1], txBuff[2]);
  printf("success:  %d\n",HAL_UART_Transmit(&huart2, txBuff, 3, 1000));
  protocolReset();
}

void protocolPromise(uint8_t addr) {
	//uint8_t blank[3]={0, 0, 0};
  //HAL_UART_Transmit(&huart2, blank, 3,10);
//	printf("pillbox commacnd check number add before  :  %d\r\n", addr);
//	printf("pillbox commacnd check number tx buf before  :  %d\r\n", txBuff[0]);
	txBuff[0] = addr;
//	txBuff[1] = 0x2e&0xff;
//	txBuff[2] = 0>>8;
	txBuff[1] =  0x2e;
	txBuff[2] = 0x0A;
//	txBuff[1] =  0x2e;
//	txBuff[2] = 0x0A;
//	printf("pillbox commacnd check number add after :  %d\r\n", txBuff[0]);
//	printf("pillbox commacnd check number tx buf after :  %d\r\n", addr);
	//HAL_UART_Transmit(&huart6, txBuff, 2,1000);
	//printf("\n");
  //printf("Promise:  %d %d %d \r\n",txBuff[0], txBuff[1], txBuff[2]);
	//HAL_UART_Transmit(&huart2, &addr, 1, 100);
	printf("success:  %d\n",HAL_UART_Transmit(&huart2, txBuff, 3, 1000));
}
void dataAnalysis(void) {
  //uint8_t regAddr = func1[funcIdx].r;
  switch(protocolState) {
    case Idle:
      addr  = rxBuff[0];
      funcIdx = 100;
      for (uint8_t i=0; i<funcListLen; i++) {
        if (addr == func1[i].h){
          funcIdx = i;

				}
      }
			
      if (funcIdx == 100){
				protocolActionCallback(addr,0xff);
				return;
			}				
			
      protocolState = ReceiveData;
			
			HAL_IWDG_Refresh(&hiwdg);
      switch(addr) {
				
				case PressureSensor:
					txBuff[0]=value_adc[PNEU_ADC]>>8;
					txBuff[1]=value_adc[PNEU_ADC];
					protocolReset();
					break;
				
				case Nothingh:
				//protocolActionCallback(Nothingh,0);
					//protocolReset();
					break;
				
				 
				 
        case  readAllLimits:
					printf("readlimit\n");
          readLimSwitch();
          txBuff[0] = addr;
          for (uint8_t i=0; i<func1[funcIdx].l; i++) {
            txBuff[1+i] = dataStream[func1[funcIdx].r+i];
          }
					txBuff[1+func1[funcIdx].l]= CalcCRC16(txBuff, func1[funcIdx].l+1)>> 8;
					txBuff[2+func1[funcIdx].l]= CalcCRC16(txBuff, func1[funcIdx].l+1)>> 8;
          HAL_UART_Transmit(&huart2, txBuff, func1[funcIdx].l+1+2,10);
          protocolReset();
          break;
        case  readAllSensors:
          txBuff[0] = addr;
          for (uint8_t i=0; i<func1[funcIdx].l; i++) {
            txBuff[1+i] = dataStream[func1[funcIdx].r+i];
          }
          HAL_UART_Transmit(&huart2, txBuff, func1[funcIdx].l+1,10);
          protocolReset();
          break;
       
       
        case  checkMachineStatus:
          printf("checkMachineStatus:\n");
          sendMachineStatus();
          protocolReset();
          break;
				
				
							
						
						
					//}
					//else
				//	{
//					printf("Testing:  ");
//					protocolActionCallback(initpillbox,0);
//					protocolReset();
//					printf("should be be here \r\n");
//          printf("initpillbox X\n");
//          break;
				//	}            // initpillbox is 26
					
					

					
      }
      if ( protocolState == ReceiveData ) {
        rxBuffLen = func1[funcIdx].l;
				HAL_UART_Receive(&huart2, rxBuff, rxBuffLen,20);
				dataAnalysis();
				
			}
      else if ( protocolState == SendData ) {
        printf("tx: ");
        for (int i=0; i<func1[funcIdx].l; i++) {
          printf("%d, ", dataStream[func1[funcIdx].r+i]);
        }
        printf("\n");
        // ----------------------
        
        
        // send -----------------
        txBuff[0] = addr;
        for (uint8_t i=0; i<func1[funcIdx].l; i++) {
          txBuff[1+i] = dataStream[func1[funcIdx].r+i];
        }
        txBuffLen = func1[funcIdx].l + 1;
        HAL_UART_Transmit(&huart2, txBuff, txBuffLen,10);
        protocolReset();
      }
      break;
      
    case ReceiveData:
			
			
			printf("addr: %d \n", addr );
     printf("RAW: %x %x %x %x   \r\n", rxBuff[0], rxBuff[1], rxBuff[2], rxBuff[3]);
			
      switch(addr) {
				case Serialnumber:
					protocolPromise(addr);
					txBuff[0] = addr;
					txBuff[1] = 1;
					txBuff[2] = 0;
					HAL_UART_Transmit(&huart2, txBuff, 3, 1000);
					HAL_UART_Transmit(&huart2, serial_number, 18,10);
					protocolReset();
					break;
				
				case ActiveDrop:
					ActiveDropEn =1;
					protocolPromise(addr);
					
					break;
				
				case batteryoff:
					protocolPromise(addr);
					protocolActionCallback(batteryoff,1);
					HAL_GPIO_WritePin(SYS_EN_GPIO_Port, SYS_EN_Pin, GPIO_PIN_RESET) ;
					HAL_GPIO_WritePin(GPIOC, CPU_EN_Pin, GPIO_PIN_RESET);
				break;
				case  CtrlStpX:
					pnpX=1;
					stepper[ARM_X].cnt=0;
         
				if((rxBuff[0]!=0) && (rxBuff[0]!=1))
				{
					protocolDataWrong(CtrlStpX,rxBuff[0]);
					break;
					
				}
				 ARM_X_dir = rxBuff[0];
				protocolPromise(addr);
        //  printf("CtrlStpX: %d \n", ARM_X_dir);
					if(temp_x==ARM_X_dir)  {
						protocolActionCallback(CtrlStpX,temp_x);
						//HAL_Delay(5);
					}
					stepstart=HAL_GetTick();
          break;
					
				

				case Nothingh:
				protocolActionCallback(Nothingh,1);
				protocolPromise(addr);
					//protocolReset();
					break;
				
				case PickPillh:
				if((	rxBuff[0] !=1) && (rxBuff[0] !=2))
				{
					protocolDataWrong(readNFC,nfcid);
					break;
				}
				dispense_check = rxBuff[0];
				PickPills1=1;
				protocolPromise(addr);
				 
					break;
				 
				 case CancelPillh:
						protocolPromise(addr);
						 CancelPickPills=1;
						
				
				
				
					break;
				

				/**/
				case  initrotateCan:
					protocolPromise(addr);

					data16[0]= rxBuff[1] | rxBuff[0] << 8;
					if(ismove_y==0)
					{	
						if (!HAL_GPIO_ReadPin(CROT_ENCI_GPIO_Port, CROT_ENCI_Pin)){
							protocolActionCallback(initrotateCan,0);
						}else{
						isZero_y=1;
						setdac(ROT_Y);
						}
				
					}else protocolActionCallback(initrotateCan,0);
					stepstart=HAL_GetTick();
          //printf("initrotateCan X: %d", data16[0]);
          break;
					
        case  rotateLimCan:
					rotate = rxBuff[1] | rxBuff[0] << 8;
					rotatereturn = rotate;
					printf(" Rotate		%i \r\n" , 	rotate);
					rottime=HAL_GetTick();
				
					temp_y=(temp_y+60+rotate)%60;
					if(rotate>0){
						Ydir=1;
					}else  {
						Ydir=0;
						rotate=-rotate;
					}
					
					if(rotate==0){
						//printf("rot: 255");
						 protocolActionCallback(rotateLimCan,rotate);
						break;
					}
					if(rotate>60){
						protocolDataWrong(rotateLimCan,rotate);
						break;
					}
					else {
						protocolPromise(addr);
						//if(ismove_y!=1){
							isLimit_y=1;					
							setdac(ROT_Y);
					//	EnablePillBoxIRQ=1;
							//HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
						NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)7) & 0x1FUL));
						
							ismove_y=1;
						rottol_y = rotate;
						rotYcnt = 0;
						startstepper(ROT_Y,!Ydir);                                 // not to revert back to debug 
						
					//	printf(" Rotate %d \r\n",rotate );
						//	moveROT_Y(rotate,Ydir);                                     
						//moveROT_Y(rotate,!Ydir);                                         Changed By Naman on 23/7/24 to accomodate the new CAN ROT Motor 
						/*}else {
							printf("timeout\n");
							protocolActionCallback(rotateLimCan,252);
						}*/
					}
         // printf("rotateLimCan lim: %d", rotate);	
					stepstart=HAL_GetTick();
				
          break;
        case  rotatePillbox:
				HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
				NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)7) & 0x1FUL));
				EnablePillBoxIRQ=1;
          rotate = rxBuff[1] | rxBuff[0] << 8;	
					rotatereturn = rotate;
					piltime = HAL_GetTick();
				//printf("bt:%d\n", piltime);
					temp_pill=(temp_pill+7+rotate)%7;
					//printf("rotatePillbox: %d \n", rotate);
					if(rotate>0){
						Pdir=1;
					}
					else  {
						Pdir=0;
						rotate=-rotate;
					}
					if(rotate==0){
						 protocolActionCallback(rotatePillbox,rotate);
						break;
					}
					if(rotate>7){
						protocolDataWrong(rotatePillbox,rotate);
						break;
					}
					
					else
					{			
						protocolPromise(addr);
						//if(ismove_pill==0){
						ismove_pill=1;
							setdac(rotProt);
							HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
							rotateProt(rotate, !Pdir);                        //changed by naman on 12/7/24 so accomodate new Stepper PillB Motor        
						//}
					//	else protocolActionCallback(rotatePillbox,0);
					}
					stepstart=HAL_GetTick();
          break;
						

				case cntLock:
					data16[0] = rxBuff[0];
				if((	rxBuff[0]!=0) && (	rxBuff[0] !=1))
				{
					protocolDataWrong(cntLock,data16[0]);
					break;
				}
				protocolPromise(addr);
				  if(data16[0] ){
						setdac(ROT_Y);
						
					}
					else{
						printf("unlock");
						setdacstop(ROT_Y);
						lockcnt=1;
					}
					printf("cntLock: %d \n", rxBuff[0] );
					protocolActionCallback(cntLock,data16[0]);
				break;
				
				case initpillbox:
					protocolPromise(addr);
						isZero_p=1;
				
          break; 
				
        case ctrlValve:
          data16[0] = rxBuff[0];
				if((	rxBuff[0] !=0) && (	rxBuff[0] !=1))
				{
					protocolDataWrong(ctrlValve,data16[0]);
					break;
				}
					protocolPromise(addr);
					HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin, (GPIO_PinState)!data16[0] );
					protocolActionCallback(ctrlValve,data16[0]);
          printf("ctrlValve: %d \n", data16[0]);
          break;
				
        case ctrlPump:
          data16[0] = rxBuff[0];
				if((rxBuff[0]!=0) && (rxBuff[0] !=1))
				{
					protocolDataWrong(ctrlPump,data16[0]);
					break;
				}
					protocolPromise(addr);
				  checktime=HAL_GetTick();
					HAL_GPIO_WritePin(PNEU_PUMP_GPIO_Port, PNEU_PUMP_Pin, (GPIO_PinState)data16[0] );
					HAL_GPIO_WritePin(PNEU_VALV_GPIO_Port, PNEU_VALV_Pin, (GPIO_PinState)!data16[0] );
					printf("ctrlPump: %d \n", data16[0]);
				/*if(data16[0]==1) {
					checkpressure=1;
				}
				 else {
					 checkpressure=0;
					 //protocolActionCallback(ctrlPump,0);
				 }*/
					protocolActionCallback(ctrlPump,data16[0]);
					ispick=0;
          break;
        case ctrlGate:
					
          data16[0] = rxBuff[0];
				if((	rxBuff[0] !=0) && (rxBuff[0] !=1))
				{
					protocolDataWrong(ctrlGate,data16[0]);
					break;
				}
				protocolPromise(addr);
          printf("ctrlGate: %d \n", data16[0]);
					HAL_GPIO_WritePin(FRNTDR_MAG_GPIO_Port, FRNTDR_MAG_Pin, (GPIO_PinState)data16[0] );
				protocolActionCallback(ctrlGate,data16[0]);
          break;
				
				case frntLock:
					protocolPromise(addr);
				
				opengate = 1;
					break;
				
				case updateParameter:
				overtemp=rxBuff[1] | rxBuff[0] << 8;
				overroll=rxBuff[3] | rxBuff[2] << 8;
				overpitch=rxBuff[5] | rxBuff[4] << 8;
				overhum=rxBuff[7] | rxBuff[6] << 8;
				protocolActionCallback(updateParameter,1);
				break;
				
			

			
			case ctrlLED:   //0:led , 1:pattern, 2,3,4:color(r, g, b), 5 brightenss
				protocolPromise(addr);
				txBuff[0] = 7;
				txBuff[1] = rxBuff[0];
				txBuff[2] = rxBuff[1];
				HAL_UART_Transmit(&huart2, txBuff, 3, 10);
				protocolReset();

			
			
				break;
			
			
			case returnPills:
        printf("returnPills: %d \n", rxBuff[0]);
        break;
			
//        case initpillbox:
//					protocolPromise(addr);
//						isZero_p=1;
//					
//          break;
			case  readNFC: 
				printf("readNFC: \n");
				//printf("func1[16].h: %d\n" ,  func1[16].h );
				nfcid=rxBuff[0];
				//nfcRead();
				//readNFCSensor();
			if((	nfcid !=1) && (nfcid !=2))
				{
					protocolDataWrong(readNFC,nfcid);
					break;
				}
				protocolPromise(addr);
				ReadNFC_PillB =1;
//				if(nfcid == 1){
//				SPI1->CR1 |= (1<<7);       //lsbfirst i.e lsbbit 1
//				//ReadNFC_Can=1;
//				ReadNFC_PillB =1;
//					
//			}
//			else if(nfcid == 2)
//			{
//				SPI1->CR1 &= ~(1<<7);      //msbfirst i.e lsbbit 0
//				ReadNFC_PillB=1;
//			}

				break;
				
        default:
          printf("default: %d \n", addr);
          
      }
      protocolReset();
      break;
      
			
			
    case SendData:
      HAL_UART_Transmit_IT(&huart2, dataStream, txBuffLen);
		
      protocolReset();
      break;
    case SetParameter:
      break;
    default:
      break;
  }
}



void sendMachineStatus(void) {
  // gen fake data
 // dataStream[getHDC1080]    = temperature;       //temperature
 // dataStream[getHDC1080+1]  = humidity;       //humid in %
 
	
  //tiltAngle;
  dataStream[ROLL_H]     = (uint16_t)roll >> 8;
  dataStream[ROLL_L]     = (uint16_t)roll;
  dataStream[PITCH_H]    = (uint16_t)pitch >> 8;
  dataStream[PITCH_L]    = (uint16_t)pitch;
  
  
  txBuff[0] = checkMachineStatus;
  txBuff[1] = dataStream[getHDC1080];   //temperature
  txBuff[2] = dataStream[getHDC1080+1]; //humid
  txBuff[3] = dataStream[ROLL_H];   //tiltAngle;
   txBuff[4] = dataStream[ROLL_L]; 
	 txBuff[5] = dataStream[PITCH_H]; 
	 txBuff[6] = dataStream[PITCH_L]; 
 /*   for (uint8_t i=0; i<6; i++) {
    txBuff[7+i] = dataStream[INT_RTC_YEAR+i];
  }*/ 
	txBuff[10]=value_adc[PNEU_ADC]>>8;
	txBuff[11]=value_adc[PNEU_ADC];
//  HAL_UART_Transmit(&huart6, txBuff, 10,1000);
	txBuff[7]=dataStream[FRNTDR_CLS];
	txBuff[8]=dataStream[FRNTDR_BUTT];
	txBuff[9]=dataStream[PILLBX_DETE];
	
	txBuff[12] =  stepper[ARM_X].cnt>>8;  
	txBuff[13] = (uint8_t) stepper[ARM_X].cnt;  
	
	
	txBuff[14] =  dataStream[NFC_DATA];
	txBuff[15] = dataStream[NFC_DATA+1];
  txBuff[16] = dataStream[NFC_DATA+2];
  txBuff[17] = dataStream[NFC_DATA+3];
	txBuff[18] = value_adc[2]>>2;
	//printf("Xpos: %d,  tx0 :  %d,  tx1:  %d\n", stepper[0].cnt, txBuff[12], txBuff[13]);
  txBuff[19] = 2;                    //firmwareVer
//	for (int i=1; i<18;i++){
//		txBuff[i]+=100;
//	}
  HAL_UART_Transmit(&huart1, txBuff, 19,10);
}

//sprintf((char*)time,"%02d:%02d:%02d\n",gTime.Hours, gTime.Minutes, gTime.Seconds);
/* Display date Format: dd-mm-yy */
//sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);

void sendLimSwitchToA33(void) {
  readLimSwitch();
  txBuff[0] = readAllLimits;
  for (uint8_t i=0; i<14; i++) {
    txBuff[1+i] = dataStream[PILLBX_HALL+i];
  }
	//printf("cup_present: %d\n", txBuff[4]);
//	for (int i = 1; i<15; i++){
//		txBuff[i]=txBuff[i] + 200;
//	}
	txBuff[15] = Battery_plug;
  HAL_UART_Transmit(&huart1, txBuff,14+1+1,100);
	//HAL_UART_Transmit(&huart6, txBuff,14+1, 1000);
	//printf("\n");
}

void sendctriticalError(errorCode_t er) {
  txBuff[0] = ctriticalError;
  txBuff[1] = er;
	printf(" error:  %d" , er);
  HAL_UART_Transmit(&huart1, txBuff, 2,10);
	
}

/*
  errorCriteriaTable
  -	Over temperature 40? Celsius
  -	Tilted over 20?
  -	Humidity level over 70%
  -	No pillbox (feedback by any Process API)
  -	NFC tag wrong OR Null (feedback by Process API)
  -	Retry failed (feedback by Process API)
*/

void updateParameterTable(void) {
  para.temp = rxBuff[0];
  para.tilt = rxBuff[1];
  para.humi = rxBuff[2];
  for(uint8_t i=0; i<5; i++) {
    para.x[i] = rxBuff[3+i];
  }
  for(uint8_t i=0; i<5; i++) {
    para.y[i] = rxBuff[8+i];
  }
}

void errorCriteriaTable(void) {
  txBuff[0] = 0;
  txBuff[1] = temp;
  txBuff[2] = dataStream[getHDC1080];
  txBuff[3] = 0;//noPillbox;
  txBuff[4] = dataStream[NFC_DATA];
  txBuff[5] = dataStream[NFC_DATA+1];
  txBuff[6] = dataStream[NFC_DATA+2];
  txBuff[7] = dataStream[NFC_DATA+3];
  //txBuff[5] = RetryFailed;
  HAL_UART_Transmit(&huart2, txBuff, 8, 10);
	 protocolReset();
}






