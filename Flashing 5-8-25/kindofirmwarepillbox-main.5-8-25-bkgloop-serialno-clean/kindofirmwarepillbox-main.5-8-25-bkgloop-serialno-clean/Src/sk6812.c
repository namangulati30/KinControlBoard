// Peripheral usage
#include "stm32f0xx_hal.h"

#include "sk6812.h"


 
uint32_t Motor_X_Steps=0;
uint32_t Motor_Y_Steps=0;
uint8_t Check_Motor_X_Steps=0;
uint8_t Check_Motor_Y_Steps=0;
uint16_t Motor_Z_Steps=0;
uint8_t Check_Motor_Z_Steps=0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  
		
	 	if(htim == &htim15){
			if(Check_Motor_Z_Steps ==1)
			{
				Motor_Z_Steps++;
				if(Motor_Z_Steps>2400)
				{
					Check_Motor_Z_Steps=0;
					HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
					Motor_Z_Steps=0;
				}
			}
//			stp = &stepper[4];
//			stp->cnt++;
//			if(stp->cnt>stp->steps && pnpX==0)  { 
//				HAL_TIM_PWM_Stop_IT(&htim15, TIM_CHANNEL_1);
//			//stepCtrl(0);
//				protocolActionCallback(ctrlStepperX,1);
//				ismove_x=0;
//				stp->cnt=0;
//			}
			
			
	
   
		}/**/
		else if(htim == &htim17){
			stp = &stepper[1];
			stp->cnt++;
			
   
		}	
		
		if(htim == &htim1){

			if(Check_Motor_X_Steps==1)
			{
				Motor_X_Steps++;
				if(Motor_X_Steps>1000)
				{
					Check_Motor_X_Steps=0;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					Motor_X_Steps=0;
				}
				
			}
			if(Check_Motor_X_Steps==2)
			{
				Motor_X_Steps++;
				if(Motor_X_Steps>2000)
				{
					Check_Motor_X_Steps=0;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					Motor_X_Steps=0;
				}
				
			}
			
			if(Check_Motor_X_Steps==3)
			{
				Motor_X_Steps++;
				if(Motor_X_Steps>4000)
				{
					Check_Motor_X_Steps=0;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					Motor_X_Steps=0;
					
				}
				
			}
			if(Check_Motor_X_Steps==5)
			{
				Motor_X_Steps++;
				if(Motor_X_Steps>8000)
				{
					Check_Motor_X_Steps=0;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					Motor_X_Steps=0;
					
				}
			
			}
			if(Check_Motor_X_Steps==6)
			{
				Motor_X_Steps++;
				if(Motor_X_Steps>3600)
				{
					Check_Motor_X_Steps=0;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					Motor_X_Steps=0;
					
				}
			
			}
				
			//stepCtrl(3);
			

		}

		
			if(htim == &htim16)
				{
		if(Check_Motor_Y_Steps==1)
			{
				Motor_Y_Steps++;
				if(Motor_Y_Steps>500)
				{
					Check_Motor_Y_Steps=0;
					StopStp(ROT_Y);
					Motor_Y_Steps=0;
				}
				
			}
			if(Check_Motor_Y_Steps==2)
			{
				Motor_Y_Steps++;
				if(Motor_Y_Steps>1000)
				{
					Check_Motor_Y_Steps=0;
					StopStp(ROT_Y);
					Motor_Y_Steps=0;
				}
				
			}
			if(Check_Motor_Y_Steps==3)
			{
				Motor_Y_Steps++;
				if(Motor_Y_Steps>1500)
				{
					Check_Motor_Y_Steps=0;
					StopStp(ROT_Y);
					Motor_Y_Steps=0;
				}
				
			}
		}
		
}
