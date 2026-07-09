#include "control.h"	
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
/**************************************************************************
Function: Control function
Input   : none
Output  : none
ܣеĿƴ붼
         5msⲿжMPU6050INTŴ
         ϸ֤ݴʱͬ	
ڲ
  ֵ				 
**************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//ѹر
	static u8 Flag_Target;																//ƺرṩ10ms׼
	int Encoder_Left,Encoder_Right;             					//ұ
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//ƽ⻷PWMٶȻPWMתPWM
	if(GPIO_Pin==MPU6050_EXTI_Pin)
	{  
			Flag_Target=!Flag_Target;
			Get_Angle(Way_Angle);                     					//̬5msһΣߵĲƵʿԸƿ˲ͻ˲Ч
			Encoder_Left=-Read_Encoder(3);            					//ȡֱֵǰΪΪ
			Encoder_Right=-Read_Encoder(4);           					//ȡֱֵǰΪΪ
																													//ATIM2_CH1,ATIM4_CH2,ļͬ
			Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//תٶȣmm/s

			if(Flag_Target==1)                        					//10msһ
			{
				
				Voltage_Temp=Get_battery_volt();		    					//ȡصѹ		
				Voltage_Count++;                       						//ƽֵ
				Voltage_All+=Voltage_Temp;              					//βۻ
				if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//ƽֵ						                                               
			}                                         					//10msһ
			if(delay_flag==1)
			{
				delay_50++;
				if(delay_50==10) delay_50=0,delay_flag=0,LD_Successful_Receive_flag=0;  		//ṩ50msľ׼ʱʾҪ50ms߾ʱ
			}
			if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode)		
			   Read_Distane();                           					//ȡֵ
//			if(Flag_follow==0&&Flag_avoid==0)	Led_Flash(100);   //LED˸;ģʽ 1sıһָʾƵ״̬	
//			if(Flag_follow==1||Flag_avoid==1)	Led_Flash(0);     //LED;/ģʽ	
			Key();                                    		    //ɨ谴״̬ ˫ԸıС״̬
			Select_Zhongzhi();
			Lidar_Avoid();                                      //״ģʽ
		    Lidar_Follow();                                         //״ģʽ
		    Lidar_Straight();                                       //״ֱģʽ
			CCD_Mode();                                          //CCDѲ
		    ELE_Mode();                                          //Ѳ
			Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //ƽPID Gyro_BalanceƽٶȼԣǰΪΪ
			Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //ٶȻPID	סٶȷСʱҪҪܿһ
			if(Mode ==CCD_Line_Patrol_Mode)                     //CCDѭµת򻷿 
			   Turn_Pwm=CCD_turn(CCD_Zhongzhi,Gyro_Turn);
		    else if(Mode==ELE_Line_Patrol_Mode)                 //ELEѭµת򻷿
			   Turn_Pwm=ELE_turn(Gyro_Turn);
		    else
			   Turn_Pwm=Turn(Gyro_Turn);							//תPID     
			
			Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //ֵPWM
			Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //ֵPWM
																													//PWMֵʹСǰʹС
			Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
			Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM޷
			if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//ǷС
				Flag_Stop=1;	                           					//͹رյ
			if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//ǷС
				Flag_Stop=0;	                           					//¾
			if(Turn_Off(Angle_Balance,Voltage)==0)     					//쳣
				Set_Pwm(Motor_Left,Motor_Right);         					//ֵPWMĴ  
	}
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angleGyroangular velocity
Output  : balanceVertical control PWM
ܣֱPD		
ڲAngle:ǶȣGyroٶ
  ֵbalanceֱPWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//ƽĽǶֵ ͻе
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //ƽƵĵPWM  PD   kpPϵ kdDϵ 
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_leftLeft wheel encoder readingencoder_rightRight wheel encoder reading
Output  : Speed control PWM
ܣٶȿPWM		
ڲencoder_leftֱencoder_rightֱ
  ֵٶȿPWM
**************************************************************************/
//޸ǰٶȣ޸Target_Velocity磬ĳ60ͱȽ
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  //================ңǰ˲====================// 
		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; ///ģʽ,ٶ
		else 											        Target_Velocity = 50;
		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //յǰź
		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //յź
	    else  Movement=Move_X;	
	
   //=============ܣ/ϣ==================// 
	  if(Mode==Ultrasonic_Follow_Mode&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //
			 Movement=Target_Velocity/Flag_velocity;
		if(Mode==Ultrasonic_Follow_Mode&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Flag_velocity;
		if(Mode==Ultrasonic_Avoid_Mode&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //
			 Movement=-Target_Velocity/Flag_velocity;
		
   //================ٶPI=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //ȡٶƫ=Ŀٶȣ˴Ϊ㣩-ٶȣұ֮ͣ 
		Encoder_bias *= 0.86;		                                          //һ׵ͨ˲       
		Encoder_bias += Encoder_Least*0.14;	                              //һ׵ͨ˲ٶȱ仯 
		Encoder_Integral +=Encoder_bias;                                  //ֳλ ʱ䣺10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //ңݣǰ
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //޷
		if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //޷	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //ٶȿ	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//رպ
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
ܣת 
ڲZ
  ֵתPWM
    ߣȤƼݸ޹˾
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//޸תٶȣ޸Turn_Amplitude
	//===================ңת=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
	 else Kd=0;   //תʱȡǵľ еģPID˼
  //===================תPD=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;//ZǽPD
	 return turn;								 				 //תPWMתΪתΪ
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_leftLeft wheel PWMmotor_rightRight wheel PWM
Output  : none
ܣֵPWMĴ
ڲPWMPWM
  ֵ
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    AIN1=1,			AIN2=0; //ǰ 
	else           			  AIN1=0,			AIN2=1; //
	PWMA=myabs(motor_left);	
  if(motor_right>0)			BIN1=1,			BIN2=0;	//ǰ
	else 	        			  BIN1=0,			BIN2=1; //
	PWMB=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : INInput  maxMaximum value  minMinimum value
Output  : Output
ܣPWMֵ 
ڲIN  max޷ֵ  min޷Сֵ
  ֵ޷ֵ
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
ܣ޸С״̬ 
ڲ
  ֵ
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)
	{ 
		Flag_Stop=!Flag_Stop;
	}		//Сͣ
	tmp2=Long_Press();                   
  if(tmp2==1) Flag_Show=!Flag_Show;	//ƽλģʽСʾֹͣ

}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angleCar inclinationvoltageVoltage
Output  : 1abnormal0normal
ܣ쳣رյ		
ڲangleСǣvoltageѹ
  ֵ1쳣  0
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//صѹ10Vرյ
	{	                                                 //Ǵ40ȹرյ
		temp=1;                                          //Flag_Stop1ƹرյ
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
	
/**************************************************************************
Function: Get angle
Input   : wayThe algorithm of getting angle 1DMP  2kalman  3Complementary filtering
Output  : none
ܣȡǶ	
ڲwayȡǶȵ㷨 1DMP  2 3˲
  ֵ
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //ȡMPU6050¶ȴݣƱʾ¶ȡ
	if(way==1)                           //DMPĶȡݲɼж϶ȡϸѭʱҪ
	{	
		Read_DMP();                      	 //ȡٶȡٶȡ
		Angle_Balance=Pitch;             	 //ƽ,ǰΪΪ
		Gyro_Balance=gyro[0];              //ƽٶ,ǰΪΪ
		Gyro_Turn=gyro[2];                 //תٶ
		Acceleration_Z=accel[2];           //Zٶȼ
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //ȡX
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //ȡY
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //ȡZ
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //ȡXٶȼ
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //ȡXٶȼ
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //ȡZٶȼ
		if(Gyro_X>32768)  Gyro_X-=65536;                 //ת  Ҳͨshortǿת
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //ת  Ҳͨshortǿת
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //ת
		if(Accel_X>32768) Accel_X-=65536;                //ת
		if(Accel_Y>32768) Accel_Y-=65536;                //ת
		if(Accel_Z>32768) Accel_Z-=65536;                //ת
		Gyro_Balance=-Gyro_X;                            //ƽٶ
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //ǣתλΪ	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //ǣתλΪ
		accel_x=Accel_X/16384;                           //MPU6050ʼΪٶȼΪ2gõԭʼΪ16λݣλΪλ
		accel_y=Accel_Y/16384;                           //Զȡλ32768Ӧ2g16384ݷֱʣԭʼݳ16384õΪm/S^2
		accel_z=Accel_Z/16384;
		gyro_x=Gyro_X/16.4;                              //ת̡2000/sӦ16.4õԭʼΪ32768Ӧ2000/s
		gyro_y=Gyro_Y/16.4;                              //32768/2000 = 16.4Ҳɲ鿴ֲõ
		if(Way_Angle==2)		  	
		{
			 Pitch= -Kalman_Filter_x(Accel_Angle_x,gyro_x);//˲λΪ
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//˲
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //ƽ
		Gyro_Turn=Gyro_Z;                                 //תٶ
		Acceleration_Z=Accel_Z;                           //Zٶȼ
	}

}
/**************************************************************************
Function: Absolute value function
Input   : aNumber to be converted
Output  : unsigned int
ܣֵ
ڲaҪֵ
  ֵ޷
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : AccelerationZ-axis accelerationAngleThe angle of balanceencoder_leftLeft encoder countencoder_rightRight encoder count
Output  : 1picked up  0No action
ܣСǷ
ڲAccelerationzٶȣAngleƽĽǶȣencoder_leftencoder_rightұ
  ֵ1:С  0Сδ
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //һ
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //1Сӽֹ
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //ڶ
	 {
			if(++count1>200)       count1=0,flag=0;                       //ʱٵȴ2000msصһ
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //2С0ȸ
			flag=2; 
	 } 
	 if(flag==2)                                                       //
	 {
		  if(++count2>100)       count2=0,flag=0;                        //ʱٵȴ1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //3С̥Ϊﵽת   
      {
				flag=0;                                                                                     
				return 1;                                                    //⵽С
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balanceLeft encoder countRight encoder count
Output  : 1put down  0No action
ܣСǷ񱻷
ڲƽǶȣұ
  ֵ1С   0Сδ
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //ֹ      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //1С0ȸ
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //ʱٵȴ 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //2С̥δϵʱΪת  
      {
				flag=0;
				flag=0;
				return 1;                         //⵽С
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
ܣתΪٶȣmm/s
ڲ
  ֵ
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//ת  ת=5msÿΣ*ȡƵ/Ƶ/ٱ/
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//ٶ=ת*ܳ
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//ٶ=ת*ܳ
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_leftLeft wheel encoder readingencoder_rightRight wheel encoder reading
Output  : none
ܣѡСģʽ
ڲencoder_left  encoder_rightұ
  ֵ
**************************************************************************/
u8 Choose(void)
{
	static int count;
	u8 tmp;
    oled_show_once();
	count += myabs(Read_Encoder(4));
	if(count>6&&count<180)	//ͨģʽ
	{
		Mode = 0;
	}
	if(count>180&&count<360)	//ģʽ 
	{	
		Mode = 1;
	}
	if(count>360&&count<540)	//ģʽ
	{		
		Mode = 2;
	}
	if(count>540&&count<720)   //״
	{
		Mode = 3;
	}
	if(count>720&&count<900)   //״
	{
		Mode = 4;
	}
	if(count>900&&count<1080)   //״ֱ
	{
		Mode = 5;
	}
	if(count>1080&&count<1260)   //CCDѲ
	{
		Mode = 6;
	}
	if(count>1260&&count<1440)   //Ѳ
	{
		Mode = 7;
	}
	if(count>1440)
		Mode=0,count=0;
	tmp=click_N_Double(50);
	if(tmp==1) 
	{
		OLED_Clear();
		return 0;
	}
	else return 1;
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
ܣ״ǰģʽ
ڲ
  ֵ
**************************************************************************/
void Lidar_Avoid(void)
{
	u8 i;
	u8 avoid_Num=0;//Ҫϵĵ
	float Angle_Sum=0;//ȷϰһı
	u8 too_close = 0;//жϰǷ̫ı
	if(Mode==Lidar_Avoid_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			if((Dataprocess[i].angle<avoid_Angle1)||(Dataprocess[i].angle>avoid_Angle2))//Сǰ100ȷΧ
			{		
				if((Dataprocess[i].distance>0)&&(Dataprocess[i].distance<avoid_Distance))//С300mmҪ
				{
					Distance=Dataprocess[i].distance;
					avoid_Num++;
					if(Dataprocess[i].angle>310) Angle_Sum += (Dataprocess[i].angle-360);//310~360תΪ-500
					else if(Dataprocess[i].angle<50) Angle_Sum+= Dataprocess[i].angle;
					if(Dataprocess[i].distance<150)			too_close++;//̫Ҫ
				}
			}
		}
		if(avoid_Num<8)
		{
		  Move_X=avoid_speed;                                           //Сһ200mm/sٶȣҪ800
			Move_Z=0;
		}
		else if(avoid_Num>8)
		{
			 Move_X=0;
	    	if(too_close>10) Move_X=-avoid_speed,Move_Z=0;              //̫һ
				else
				{
					if(Angle_Sum>0)      
					{
						Move_Z=-turn_speed;//ϰ￿ңת
					}
					else   Move_Z=turn_speed; //ϰ￿ת
				}					
		}
 }
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
ܣ״ģʽ
ڲ
  ֵ
**************************************************************************/
void Lidar_Follow(void)
{
	u8 i;
	u8 follow_num=0;                //жϸĵ
	u16 mini_distance = 65535;      //Ҫľ룬Сľ
	static float angle =0;                 //ĽǶ
	static float last_angle = 0;           //һǶ
	u8 data_count = 0;
	if(Mode==Lidar_Follow_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			 if((0<Dataprocess[i].distance) && (Dataprocess[i].distance<Follow_distance))//0~1500mmѡĵ
			 {
				 follow_num++;
				 if(Dataprocess[i].distance<mini_distance)                  //жϳСĵ
				 {
					 mini_distance = Dataprocess[i].distance;
					 angle = Dataprocess[i].angle;
					 Distance = mini_distance;                                     //oledʾҪľ
				 }
			 }
	  }
	if(angle>180)
		  angle -= 360;				//0--360ת0--180-180--0˳ʱ룩
	if(angle-last_angle>10 ||angle-last_angle<-10)	//һ10ȵҪж
	{
		if(++data_count == 60)		//60βɼֵ(300ms)ϴεıȴ10ȣʱΪЧֵ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//С10ȵĿֱΪЧֵ
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(follow_num>5) 	
	{
		Move_X=Lidar_follow_PID(mini_distance,300);//ľpidʱֱٶȻҪСһ(MoveķΧ0~800)
		Move_Z=Follow_Turn_PID(angle,0);//תPIDֱת
	}
	else
	{
		Move_X = 0;
		Move_Z = 0;
	}
	if(Move_X>60)    Move_X=60;
 }
}
/**************************************************************************
Function: Lidar_Straight
Input   : none
Output  : none
ܣ״ֱģʽ
ڲ
  ֵ
**************************************************************************/
void Lidar_Straight(void) 
{
	static u16 target_distance=0;
	u8 i;
	u16 current_distance=target_distance;
	static u16 Limit_distance=0;   //״̽
	if(Mode==Lidar_Straight_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		 Move_X=Initial_speed;//Сһʼٶ
		 for(i=0;i<225;i++)
	  {
		  if((Dataprocess[i].angle>71)&&(Dataprocess[i].angle<74))//ȡ״7075ȷΧĵȽϵ
		 {
			 if(determine<Limit_time) //ģʽתStraightģʽ3ȷҪľ
			 {
				 target_distance=Dataprocess[i].distance;
				 Limit_distance=target_distance+200;//Ŀ200mm,ҪʧСת
				 determine++;
				 if(determine==(Limit_time-1)) determine=Limit_time;
			 }
			 if(Dataprocess[i].distance<(Limit_distance))//һ״̽
			 {
				 current_distance=Dataprocess[i].distance;//ȷ
			   Distance=Dataprocess[i].distance;
			 }
		 }
	 }
	 Move_Z=Distance_Adjust_PID(current_distance,target_distance);//״pid
	}
}

/**************************************************************************
ܣCCDȡֵ
ڲ
  ֵ
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //ֵ̬㷨ȡСֵ
     for(i=5;i<123;i++)   //߸ȥ5
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //Сֵ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //ȡֵ
	 for(i = 5;i<118; i++)   //Ѱ
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱұ
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//λ
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //ߵƫ̫
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //ȡһεֵ
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //һεƫ
}

/**************************************************************************
Function: CCD_Mode
Input   : none
Output  : none
ܣCCDѲģʽ
ڲ: 
  ֵ
**************************************************************************/	 	
void CCD_Mode(void)
{
	static u8 Count_CCD = 0;								//CCDƵ
	if(Mode == CCD_Line_Patrol_Mode && Flag_Left !=1 &&Flag_Right !=1)
	{
		Move_X = tracking_speed;			//CCDѲٶ
		if(++Count_CCD == 4)								//ڿƵʣ4*5 = 20msһ
		{
			RD_TSL(); 
            Find_CCD_Zhongzhi();			
            Count_CCD = 0;			
		}
//		else if(Count_CCD>4)  Count_CCD = 0;
	    
		
	}
}

/**************************************************************************
ܣCCDģʽת  Ѳ
ڲCCDȡ Z
  ֵתPWM
    ߣƽС֮
**************************************************************************/
int CCD_turn(u8 CCD,float gyro)//ת
{
	  float Turn;     
      float Bias,kp=30,Kd=0.12;	  
	  Bias=CCD-64;
	  Turn=Bias*kp+gyro*Kd;
	  return Turn;
}

/**************************************************************************
Function: ELE_Mode
Input   : none
Output  : none
ܣѲģʽ
ڲ: 
  ֵ
**************************************************************************/	 	
void ELE_Mode(void)
{
	if(Mode == ELE_Line_Patrol_Mode && Flag_Left!=1 &&Flag_Right!=1)
	{
		int Sum = 0;
		Sensor_Left = Get_Adc(2);
		Sensor_Middle = Get_Adc(1);
		Sensor_Right = Get_Adc(9);
		Sum = Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;			
		Sensor = Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);
//		if(Detect_Barrier() == No_Barrier)		//⵽ϰ
				Move_X=tracking_speed;       //Сһ300mm/sٶ
//		else									//ϰ
//		{
//			Move_X = 0;
//		}	
	}
	
}

/**************************************************************************
ܣELEģʽת
ڲֱֱZ
  ֵתPWM
    ߣƽС֮
**************************************************************************/
int ELE_turn(float gyro)//ת
{
	float Turn;     
	float Bias,kp=60,Kd=0.2;	  
	Bias=Sensor-100;
	Turn=Bias*kp+gyro*Kd;
	  return Turn;
}


/**************************************************************************
Function: Select_Zhongzhi
Input   : none
Output  : none
ܣСеֵѡ
ڲ
  ֵ
**************************************************************************/
void Select_Zhongzhi(void)                   //еֵѡ񣬱ⰲװϵѲߡCCDѲװʱСǰ
{
	if(Mode == ELE_Line_Patrol_Mode)
		Middle_angle = -9;
	else if(Mode == CCD_Line_Patrol_Mode)
		Middle_angle = -4;
	else   Middle_angle = 0;
}

