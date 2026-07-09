#include "ioi2c.h"
#include "delay.h"

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : 1
ܣģIICʼź
ڲ
  ֵ1
**************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();     //sda
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;//ǯסI2Cߣ׼ͻ 
	return 1;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
ܣģIICź
ڲ
  ֵ
**************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//I2C߽ź
	delay_us(1);							   	
}

/**************************************************************************
Function: IIC wait the response signal
Input   : none
Output  : 0No response received1Response received
ܣIICȴӦź
ڲ
  ֵ0ûյӦ1յӦ
**************************************************************************/
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDAΪ  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//ʱ0 	   
	return 1;  
} 

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
ܣIICӦ
ڲ
  ֵ
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
	
/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
ܣIICӦ
ڲ
  ֵ
**************************************************************************/    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**************************************************************************
Function: IIC sends a byte
Input   : txdByte data sent
Output  : none
ܣIICһֽ
ڲtxd͵ֽ
  ֵ
**************************************************************************/	  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//ʱӿʼݴ
    for(t=0;t<8;t++)
    {              
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1; 	  
			delay_us(1);   
			IIC_SCL=1;
			delay_us(1); 
			IIC_SCL=0;	
			delay_us(1);
    }	 
} 	 
  
/**************************************************************************
Function: IIC write data to register
Input   : addrDevice addressregRegister addresslen;Number of bytesdataData
Output  : 0Write successfully1Failed to write
ܣIICдݵĴ
ڲaddr豸ַregĴַlen;ֽdata
  ֵ0ɹд룻1ûгɹд
**************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
Function: IIC read register data
Input   : addrDevice addressregRegister addresslen;Number of bytes*bufData read out
Output  : 0Read successfully1Failed to read
ܣIICĴ
ڲaddr豸ַregĴַlen;ֽ*bufݻ
  ֵ0ɹ1ûгɹ
**************************************************************************/

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**************************************************************************
Function: IIC reads a byte
Input   : ackSend response signal or not1Send0Do not send
Output  : receiveData read
ܣIICȡһλ
ڲackǷӦźţ1ͣ0
  ֵreceiveȡ
**************************************************************************/ 
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDAΪ
    for(i=0;i<8;i++ )
	 {
			IIC_SCL=0; 
			delay_us(2);
			IIC_SCL=1;
			receive<<=1;
			if(READ_SDA)receive++;   
			delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //ACK 
    else
        IIC_NAck();//nACK  
    return receive;
}

/**************************************************************************
Function: IIC reads a byte
Input   : I2C_AddrDevice IIC addressaddr:Register address
Output  : resData read
ܣȡָ豸ָĴһֵ
ڲI2C_Addr豸IICַaddr:Ĵַ
  ֵresȡ
**************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //д
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //͵ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//һֹͣ	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
  IIC_Stop();//һֹͣ

	return res;
}
 
/**************************************************************************
Function: IIC continuous reading data
Input   : devTarget device IIC addressreg:Register address
					lengthNumber of bytes*data:The pointer where the read data will be stored
Output  : countNumber of bytes read out-1
ܣIIC
ڲdevĿ豸IICַreg:Ĵַlengthֽ
					*data:ݽҪŵָ
  ֵcountֽ-1
**************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //д
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //͵ַ
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //ACKĶ
		 else                  data[count]=IIC_Read_Byte(0);  //һֽNACK
	}
    IIC_Stop();//һֹͣ
    return count;
}
/**************************************************************************
Function: Writes multiple bytes to the specified register of the specified device
Input   : devTarget device IIC addressregRegister addresslengthNumber of bytes
					*dataThe pointer where the read data will be stored
Output  : 1
ܣֽдָ豸ָĴ
ڲdevĿ豸ַregĴַlengthҪдֽ
					*dataҪдݵ׵ַ
  ֵ1Ƿɹ
**************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //д
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //͵ַ
  IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//һֹͣ

    return 1; //status == 0;
}

/**************************************************************************
Function: Reads a byte of the specified register of the specified device
Input   : devTarget device IIC addressregRegister address*dataThe pointer where the read data will be stored
Output  : 1
ܣȡָ豸ָĴһֵ
ڲdevĿ豸ַregĴַ*dataҪдݵ׵ַ
  ֵ1Ƿɹ
**************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************************************************************
Function: Write a byte to the specified register of the specified device
Input   : devTarget device IIC addressregRegister addressdataData to be writtenwill be stored
Output  : 1
ܣдָ豸ָĴһֽ
ڲdevĿ豸ַregĴַdataҪд
  ֵ1
**************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
Function: Read, modify, and write multiple bits in a byte of the specified device specified register
Input   : devTarget device IIC addressregRegister addresslengthNumber of bytes
					bitStartStart bit of target bytedataStores the value of the target byte bit to be changed
Output  : 1success0fail
ܣ ޸ д ָ豸 ָĴһֽ еĶλ
ڲdevĿ豸ַregĴַbitStartĿֽڵʼλ
					dataŸıĿֽλֵ
  ֵ1ɹ0ʧ
**************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}


/**************************************************************************
Function: Read, modify, and write one bit in a byte of the specified device specified register
Input   : devTarget device IIC addressregRegister address
					bitNumTo modify the bitnum bit of the target bytedataWhen it is 0, the target bit will be cleared, otherwise it will be set
Output  : 1success0fail
ܣ ޸ д ָ豸 ָĴһֽ е1λ
ڲdevĿ豸ַregĴַbitNumҪ޸ĿֽڵbitNumλ
					dataΪ0ʱĿλ壬򽫱λ
  ֵ1ɹ0ʧ
**************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}


