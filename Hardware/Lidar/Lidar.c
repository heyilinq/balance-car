
#include "Lidar.h"
#include <string.h>
#include "sys.h"



float Follow_KP =10,Follow_KD =1,Follow_KI = 0.001;	

PointDataProcessDef PointDataProcess[225] ;//225
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[225];      //Сϡ桢ֱߡELE״ϵ״


/**************************************************************************
Function: data_process
Input   : none
Output  : none
ܣݴ
ڲ
  ֵ
**************************************************************************/
//һ֡պд
void data_process(void) //ݴ
{
	static u8 data_cnt = 0;
	u8 i,m,n;
	u32 distance_sum[8]={0};//2ľ͵
	LD_Successful_Receive_flag=1;
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//16ĿʼǶ
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//16ĽǶ
	float area_angle[8]={0};//һ֡ݵ8ƽǶ
	if((start_angle>350)&&(end_angle<13))//Ϊһ֡13ȣ35010ηΧӣangle180Χ
		end_angle +=360;
	for(m=0;m<8;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/8*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}
	for(i=0;i<16;i++)
	{
		switch(i)
		{
			case 0:case 1:
				distance_sum[0] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//0~1ľ
			  break;
			case 2:case 3:
				distance_sum[1] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//2~3ľ
			  break;
			case 4:case 5:
				distance_sum[2] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//4~5ľ
			  break;
			case 6:case 7:
				distance_sum[3] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//6~7ľ
			  break;
			case 8:case 9:
				distance_sum[4] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//8~9ľ
			  break;
			case 10:case 11:
				distance_sum[5] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//10~11ľ
			  break;
			case 12:case 13:
				distance_sum[6] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//12~13ľ
			  break;
			case 14:case 15:
				distance_sum[7] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//14~15ľ
			  break;
			default:break;
								
		}
		  
	}
	for(n=0;n<8;n++)
	{
		PointDataProcess[data_cnt+n].angle = area_angle[n];
	  PointDataProcess[data_cnt+n].distance = distance_sum[n]/2;
	}
	data_cnt +=8;
	if(data_cnt>=225)
	{
		for(i=0;i<225;i++)
		{
			Dataprocess[i].angle=PointDataProcess[i].angle;
			Dataprocess[i].distance=PointDataProcess[i].distance;
		}
		data_cnt = 0;
	}
		
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
ܣ״pid
ڲ: ǰĿ
  ֵĿٶ
**************************************************************************/	 	
//״pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//ƫ
	OutPut=-2.5*Bias-100*(Bias-Last_Bias);//λʽPID  //
	Last_Bias=Bias;                                       		 			//һƫ
	return OutPut;                                          	
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
ܣ״pid
ڲ: ǰĿ
  ֵĿٶ
**************************************************************************/	 	

float Lidar_follow_PID(float Current_Distance,float Target_Distance)//PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//ƫ
	OutPut=-0.15*Bias-0.1*(Bias-Last_Bias);//λʽPID  //
	Last_Bias=Bias;                                       		 			//һƫ
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
ܣ״תpid
ڲ: ǰǶȺĿǶ
  ֵתٶ
**************************************************************************/	 	
//״תpid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)		                                 				 //ƫĻ
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //ƫ
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=-Follow_KP*Bias-Follow_KI*Integral_bias-Follow_KD*(Bias-Last_Bias);	//λʽPID
	Last_Bias=Bias;                                       					 		//һƫ
	if(Turn_Off(Angle_Balance,Voltage)== 1)								//رգʱ
		Integral_bias = 0;
	return OutPut;                                           					 	//
	
}




