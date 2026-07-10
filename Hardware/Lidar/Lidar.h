
#ifndef __LIDAR_H
#define	__LIDAR_H

#include "main.h"
#include "sys.h"



#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define Length_ 0x3A

#define POINT_PER_PACK 16






typedef struct PointData
{
	uint8_t distance_h;
	uint8_t distance_l;
	uint8_t Strong;

}LidarPointStructDef;

typedef struct PackData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t ver_len;
	
	uint8_t speed_h;
	uint8_t speed_l;
	uint8_t start_angle_h;
	uint8_t start_angle_l;	
	LidarPointStructDef point[POINT_PER_PACK];
	uint8_t end_angle_h;
	uint8_t end_angle_l;
	uint8_t crc;
}LiDARFrameTypeDef;

typedef struct PointDataProcess_
{
	u16 distance;
	float angle;
}PointDataProcessDef;


extern PointDataProcessDef PointDataProcess[225];//锟斤拷锟斤拷225锟斤拷锟斤拷锟斤拷
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef Dataprocess[225];//锟斤拷锟斤拷小锟斤拷锟斤拷锟较★拷锟斤拷锟芥、锟斤拷直锟竭★拷ELE锟阶达拷锟斤拷系锟斤拷状锟斤拷锟斤拷锟�

void LIDAR_USART_Init(void);
void  UART5_IRQHandler(void);
void data_process(void);
float Distance_Adjust_PID(float Current_Distance,float Target_Distance);
void Get_Target_Encoder(float Vx,float Vz);
int Incremental_PI_Left (int Encoder,int Target);
int Incremental_PI_Right (int Encoder,int Target);
float Follow_Turn_PID(float Current_Angle,float Target_Angle);
float Lidar_follow_PID(float Current_Distance,float Target_Distance);//锟斤拷锟斤拷锟斤拷锟絇ID
#endif


