#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

#define PI 3.14159265							//PIԲ
#define Control_Frequency  200.0	//ȡƵ
#define Diameter_67  67.0 				//ֱ67mm 
#define EncoderMultiples   4.0 		//Ƶ
#define Encoder_precision  13.0 	// 13
#define Reduction_Ratio  30.0			//ٱ30
#define Perimeter  210.4867 			//ܳλmm

//Сģʽ
#define Normal_Mode							0
#define Ultrasonic_Avoid_Mode               1
#define Ultrasonic_Follow_Mode              2
#define Lidar_Avoid_Mode					3
#define Lidar_Follow_Mode					4
#define Lidar_Straight_Mode                 5
#define ELE_Line_Patrol_Mode				7
#define CCD_Line_Patrol_Mode				6

//ģʽĲ
#define  avoid_Distance 350//Ͼ300mm
#define avoid_Angle1 50 //ϵĽǶȣ310~3600~50ķΧ
#define avoid_Angle2 310
#define avoid_speed 30    //ٶ
#define turn_speed 1000;    //תٶ

//״ֱߵĲ
#define Initial_speed 30//СĳʼٶȴΪ200mmÿ
#define Limit_time 500   //ʱ䣬5msж*ֵ=ʱ 3s
#define refer_angle1  71 //ĽǶ1
#define refer_angle2  74 //ĽǶ2

//״
#define Follow_distance 1500  //״ģʽԶ

#define Barrier_Detected						1
#define No_Barrier								0
#define tracking_speed 40      //Сһ300mm/sٶ
#define Detect_distance 700//Ϊ700mm


#define DIFFERENCE 100
#define INT PAin(12)   //PA12ӵMPU6050ж

int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Key(void);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
u8 Choose(void);
void Lidar_Avoid(void);
void Lidar_Straight(void);
void Lidar_Follow(void);
void CCD_Mode(void);
int CCD_turn(u8 CCD,float gyro);//ת
int ELE_turn(float gyro);//ת
void ELE_Mode(void);
void Select_Zhongzhi(void);
#endif
