/**
 * ============================================================
 * B570 平衡小车 - 姿态滤波模块
 * ============================================================
 * 提供两种角度融合算法：
 * 1. 卡尔曼滤波 Kalman_Filter_x/y  — 精度高，CPU占用中等
 * 2. 互补滤波 Complementary_Filter  — 简单快速，只有一个参数 K1=0.02
 *
 * 两种算法都解决的问题：
 *   加速度计 → 长期准确但振动时抖动大
 *   陀螺仪   → 短期平滑但积分后漂移
 *   滤波器   → 短期信陀螺仪 + 长期用加速度修正 → 干净的角度
 *
 * 调用频率: 每 5ms 一次（由 control.c 中断驱动）
 * ============================================================
 */

#include "filter.h"

/**************************************************************************
Function: Simple Kalman filter  (卡尔曼滤波 - X轴)
Input   : Accel:加速度换算的角度(°)；Gyro：角速度(°/s)
Output  : 融合后的角度

【两状态卡尔曼滤波】
状态向量: [角度, 陀螺仪零偏]
Q_angle=0.001: 过程噪声（越小越信陀螺仪模型）
Q_gyro =0.003: 陀螺仪零偏噪声
R_angle=0.500: 测量噪声（越小越信加速度计）

五个步骤: 预测 → 协方差预测 → 算卡尔曼增益 → 修正 → 更新协方差
dt=0.005: 采样周期 5ms

入口参数：加速度获取的角度、角速度
返回  值：x轴角度(°)
**************************************************************************/
float dt=0.005;		  //每5ms进行一次滤波                 
float Kalman_Filter_x(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 过程噪声的协方差
	float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle=0.5;		// 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	return angle;
}
/**************************************************************************
Function: First order complementary filtering  (一阶互补滤波 - X轴)
Input   : angle_m:加速度换算角；gyro_m:陀螺仪角速度
Output  : 融合角度

【公式】
angle = K1*加速度角度 + (1-K1)*(上次角度 + 角速度*dt)
      = 2% 信加速度计 + 98% 信陀螺仪积分

K1=0.02: 越大越信加速度计（响应快但抖动大）、越小越信陀螺仪（平滑但滞后）

入口参数：加速度获取的角度、角速度
返回  值：x轴角度(°)
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
	 static float angle;
	 float K1 =0.02; 
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	 return angle;
}
/**************************************************************************
Function: Simple Kalman filter
Input   : acceleration、angular velocity
Output  : none
函数功能：获取y轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角速度
**************************************************************************/
float Kalman_Filter_y(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 过程噪声的协方差
	float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle=0.5;		// 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	   //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	//输出值(后验估计)的微分=角速度
	return angle;
}
/**************************************************************************
Function: First order complementary filtering
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角速度
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
	 static float angle;
	 float K1 =0.02; 
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	 return angle;
}


