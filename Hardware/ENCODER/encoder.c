#include "encoder.h"

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
//函数功能：TIM3中断服务函数
//入口参数：无
//返回  值：无
//**************************************************************************/
//void TIM4_IRQHandler(void)//中断处理函数为空，清除中断标志位后结束中断
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)//溢出中断
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除中断标志位 	
//}
///**************************************************************************
//函数功能：TIM2中断服务函数
//入口参数：无
//返回  值：无
//**************************************************************************/
//void TIM2_IRQHandler(void)//中断处理函数为空，清除中断标志位后结束中断
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM2,TIM_FLAG_Update)==SET)//溢出中断
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位 	  
//}
