#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  13.0 	//编码器精度 13线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm

//小车各模式定义
#define Normal_Mode                0  //普通平衡
#define Ultrasonic_Avoid_Mode      1  //超声波避障
#define Ultrasonic_Follow_Mode     2  //超声波跟随

//避障模式的参数
#define  avoid_Distance 350       //避障距离350mm
#define avoid_Angle1 50           //避障的角度
#define avoid_Angle2 310
#define avoid_speed 30            //避障速度
#define turn_speed 1000;          //避障转向速度

#define DIFFERENCE 100
#define INT PAin(12)               //PA12连接到MPU6050的中断引脚

int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Key(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
u8 Choose(void);
void Select_Zhongzhi(void);
#endif
