#include "gpio_based_i2c.h"

#define I2C_SCL_OUT         (PCSet(10))
#define I2C_SDA_OUT         (PCSet(11))
#define I2C_SDA_IN          (PCGet(11))

void I2C_Delay()
{
  // delay 5us
  volatile uint8_t i, j;
  for (i = 0; i < 168; i++)
    for(j = 0; j < 5; j++);

}

void I2C_Start(void)
{
	I2C_SCL_OUT = 1;
	I2C_SDA_OUT = 1;
	I2C_Delay();
	I2C_SDA_OUT = 0;
	I2C_Delay();
	I2C_SCL_OUT = 0;
	I2C_Delay();	
}

void I2C_Stop(void)
{
	I2C_SCL_OUT = 0;
	I2C_SDA_OUT = 0;
	I2C_Delay();
	I2C_SCL_OUT = 1;
	I2C_Delay();
	I2C_SDA_OUT = 1;
	I2C_Delay();
}

//产生应答信号
void I2C_Ack(void)
{
	I2C_SCL_OUT = 0;
	I2C_Delay();
	I2C_SDA_OUT = 0;
	I2C_Delay();
	I2C_SCL_OUT = 1;
	I2C_Delay();
	I2C_SCL_OUT = 0;
	I2C_Delay();
}

//产生非应答信号
void I2C_Nack(void)
{
	I2C_SCL_OUT = 0;
	I2C_Delay();
	I2C_SDA_OUT = 1;
	I2C_Delay();
	I2C_SCL_OUT = 1;
	I2C_Delay();
	I2C_SCL_OUT = 0;
	I2C_Delay();
}


void I2C_Send_Byte(uint8_t send_byte)
{
	uint8_t i;
	
	for(i = 0 ; i < 8 ; i++)
	{
		I2C_SDA_OUT = (send_byte & 0x80) >> 7;
		I2C_Delay();
		I2C_SCL_OUT = 1;
		I2C_Delay();
		I2C_SCL_OUT = 0;
		if(i==7)
		{
			I2C_SDA_OUT = 1;	//发送最后一位之后，释放SDA总线，方便接收来自接收端的应答信号
		}
		I2C_Delay();
		send_byte <<= 1;
	}
}


uint8_t I2C_Read_Byte()
{
	uint8_t i;
	uint8_t receive = 0;

	for(i = 0; i < 8; i++)
	{
		receive <<= 1;
		I2C_SCL_OUT = 1;
		I2C_Delay();
		if(I2C_SDA_IN == 1)
		{
			receive++;
		}
		I2C_SCL_OUT = 0;
		I2C_Delay();
	}
	return receive;
}

uint8_t I2C_Wait_Ack(void)
{
	uint8_t ack_flag = 0;
	
	I2C_SDA_OUT = 1;	//释放SDA总线
	I2C_Delay();
	I2C_SCL_OUT = 1;
	I2C_Delay();
	if(I2C_SDA_IN == 1)
	{	
		ack_flag = 1;	//非应答
	}
	else
	{
		ack_flag = 0;	//应答
	}
	I2C_SCL_OUT = 0;
	I2C_Delay();
	return ack_flag;
}
