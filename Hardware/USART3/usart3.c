
#include "usart3.h"

u8 Usart2_Receive_buf[1];          //3жݴŵĻ
u8 Usart2_Receive;                 //Ӵ3ȡ
u8 Usart3_Receive_buf[1];          //3жݴŵĻ
u8 Usart3_Receive;                 //Ӵ3ȡ
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //ջص
{
	
	if(UartHandle->Instance == USART3)
	{
    static	int uart_receive=0;//ر
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	uart_receive=Usart3_Receive_buf[0]; 
		Usart3_Receive=uart_receive;
		if(uart_receive==0x59)  Flag_velocity=2;  //ٵĬֵ
		if(uart_receive==0x58)  Flag_velocity=1;  //ٵ
		
	  if(uart_receive>10)  //Ĭʹ
    {			
			if(uart_receive==0x5A)	    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ
			else if(uart_receive==0x41)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//ǰ
			else if(uart_receive==0x45)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
																	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    
																	Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ
  	}
		if(uart_receive<10)     //appΪMiniBalanceV1.0  ΪMiniBalanceV1.0ңָΪA~H HEXС10
		{			
//			Flag_velocity=1;//лٵ
			if(uart_receive==0x00)	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ
			else if(uart_receive==0x01)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//ǰ
			else if(uart_receive==0x05)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//
			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
														Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //
			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //
														Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//ɲ
  	}	

		
		if(Usart3_Receive==0x7B) Flag_PID=1;   //APPָʼλ
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APPָֹͣλ

		 if(Flag_PID==1)  //ɼ
		 {
				Receive[i]=Usart3_Receive;
				i++;
		 }
		 if(Flag_PID==2)  //
		 {
			  if(Receive[3]==0x50) 				 PID_Send=1;
			  else if(Receive[1]!=0x23) 
				{								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					switch(Receive[1])
					{
						case 0x30:  Balance_Kp=Data;break;
						case 0x31:  Balance_Kd=Data;break;
						case 0x32:  Velocity_Kp=Data;break;
						case 0x33:  Velocity_Ki=Data;break;
						case 0x34:  Turn_Kp=Data;break; 
					  case 0x35:  Turn_Kd=Data;break; 
						case 0x36:  break; //Ԥ
						case 0x37:  break; //Ԥ
						case 0x38:  break; //Ԥ
					}
				}				 
			    Flag_PID=0;
					i=0;
					j=0;
					Data=0;
					memset(Receive, 0, sizeof(u8)*50);//
		 } 	
		 
		HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));//3صִ֮Ҫٴοжϵȴһνжϵķ
	}
	else if(UartHandle->Instance == USART2)
	{
		static u8 state = 0;//״̬λ	
		static u8 crc_sum = 0;//У
		static u8 cnt = 0;//һ֡16ļ
		u8 temp_data;
		temp_data=Usart2_Receive_buf[0]; 
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//ͷ̶
				{
					Pack_Data.header_0= temp_data;
					state++;
					//У
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//ͷ̶
				{
					Pack_Data.header_1 = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 2:
				if(temp_data == Length_)//̶ֳ
				{
					Pack_Data.ver_len = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//ٶȸ߰λ
				state++;
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//ٶȵͰλ
				state++;
				crc_sum += temp_data;
				break;
			case 5:
				Pack_Data.start_angle_h = temp_data;//ʼǶȸ߰λ
				state++;
				crc_sum += temp_data;
				break;
			case 6:
				Pack_Data.start_angle_l = temp_data;//ʼǶȵͰλ
				state++;
				crc_sum += temp_data;
				break;
			
			case 7:case 10:case 13:case 16:
			case 19:case 22:case 25:case 28:
			case 31:case 34:case 37:case 40:
			case 43:case 46:case 49:case 52:
				Pack_Data.point[cnt].distance_h = temp_data;//16ľݣֽ
				state++;
				crc_sum += temp_data;
				break;
			
			case 8:case 11:case 14:case 17:
			case 20:case 23:case 26:case 29:
			case 32:case 35:case 38:case 41:
			case 44:case 47:case 50:case 53:
				Pack_Data.point[cnt].distance_l = temp_data;//16ľݣֽ
				state++;
				crc_sum += temp_data;
				break;
			
			case 9:case 12:case 15:case 18:
			case 21:case 24:case 27:case 30:
			case 33:case 36:case 39:case 42:
			case 45:case 48:case 51:case 54:
				Pack_Data.point[cnt].Strong = temp_data;//16ǿ
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			
			case 55:
				Pack_Data.end_angle_h = temp_data;//Ƕȵĸ߰λ
				state++;
				crc_sum += temp_data;			
				break;
			case 56:
				Pack_Data.end_angle_l = temp_data;//ǶȵĵͰλ
				state++;
				crc_sum += temp_data;
				break;
			case 57:
				Pack_Data.crc = temp_data;//У
				state = 0;
				cnt = 0;
				if(crc_sum == Pack_Data.crc)
				{
					data_process();//ݴУȷˢ´洢
				}
				else 
				{
					memset(&Pack_Data,0,sizeof(Pack_Data));//
				}
				crc_sum = 0;//У
				break;
			default: break;
	  }	
     HAL_UART_Receive_IT(&huart2,Usart2_Receive_buf,sizeof(Usart2_Receive_buf));//2ж
	}
}

