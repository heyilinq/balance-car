
#include "delay.h"

static u8  fac_us=0;							//usʱ			   
static u16 fac_ms=0;							//msʱ,ucos,ÿĵms

void delay_init()
{
	SysTick->CTRL &= ~(1<<2) ;   //SysTickʹⲿʱԴAHBʱӵ1/8   72MHz/8 = 9MHz
	fac_us= 9;                   //SysTickһҪ 1/9MHz   9Ҫ 9* 1/9MHz = 1us  ʱdelay_usֵǡҪٸ1us,delay_msͬ
	fac_ms=(u16)fac_us*1000;     //1ms = 1000us
}

/**************************************************************************
Function: Delay functionus
Input   : nusThe number of us to delay
Output  : none
ܣʱus
ڲnusҪʱus	
  ֵ
**************************************************************************/			    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 								//ʱ	  		 
	SysTick->VAL=0x00;        								//ռ
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//ʼ	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//ȴʱ䵽   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//رռ
	SysTick->VAL =0X00;      					 				//ռ	 
}
/**************************************************************************
Function: Delay functionms
Input   : musThe number of ms to delay
Output  : none
ܣʱus
ڲmusҪʱms	
  ֵ
**************************************************************************/
//עnmsķΧ
//SysTick->LOADΪ24λĴ,,ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLKλΪHz,nmsλΪms
//72M,nms<=1864
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;						//ʱ(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;												//ռ
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//ʼ  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//ȴʱ䵽   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//رռ
	SysTick->VAL =0X00;       								//ռ	  	    
} 
