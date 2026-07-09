
#ifndef __IOI2C_H
#define __IOI2C_H

#include "sys.h"

//IO
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //SDA 

//IICв
void IIC_Init(void);           //ʼIICIO				 
int IIC_Start(void);					 //IICʼź
void IIC_Stop(void);	  			 //IICֹͣź
void IIC_Send_Byte(u8 txd);		 //IICһֽ
u8 IIC_Read_Byte(unsigned char ack);//IICȡһֽ
int IIC_Wait_Ack(void); 			 //IICȴACKź
void IIC_Ack(void);						 //IICACKź
void IIC_NAck(void);					 //IICACKź

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

//------------------End of File----------------------------
