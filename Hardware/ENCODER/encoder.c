#include "encoder.h"

/**************************************************************************
ܣλʱȡ
ڲʱ
  ֵٶֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
	 {
	   case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0; break;
	   case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0; break;	
	   default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
///**************************************************************************
//ܣTIM3жϷ
//ڲ
//  ֵ
//**************************************************************************/
//void TIM4_IRQHandler(void)//жϴΪգжϱ־λж
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)//ж
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //жϱ־λ 	
//}
///**************************************************************************
//ܣTIM2жϷ
//ڲ
//  ֵ
//**************************************************************************/
//void TIM2_IRQHandler(void)//жϴΪգжϱ־λж
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM2,TIM_FLAG_Update)==SET)//ж
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //жϱ־λ 	  
//}
