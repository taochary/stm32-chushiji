#include "main.h" 
#include "EEPROM.h" 
#include "AHT20.h"


void SDA0_Pin_Output_High(void)   //将PB9配置为输出 ， 并设置为高电平， PB9作为I2C的SDA
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

void SDA0_Pin_Output_Low(void)  //将P9配置为输出  并设置为低电平
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void SDA0_Pin_IN_FLOATING(void)  //SDA配置为浮空输入
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SCL0_Pin_Output_High(void) //SCL输出高电平，P8作为I2C的SCL
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
}

void SCL0_Pin_Output_Low(void) //SCL输出低电平
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
}

void Init_I2C0_Sensor_Port(void) //初始化I2C接口,输出为高电平
{	
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	
}
void I2C0_Start(void)		 //I2C主机发送START信号
{
	SDA0_Pin_Output_High();
	SensorDelay_us(8);
	SCL0_Pin_Output_High();
	SensorDelay_us(8);
	SDA0_Pin_Output_Low();
	SensorDelay_us(8);
	SCL0_Pin_Output_Low();
	SensorDelay_us(8);   
}

uint8_t Receive_ACK0(void)   //看是否有回复ACK
{
	uint16_t CNT;
	CNT = 0;
	SCL0_Pin_Output_Low();	
	SDA0_Pin_IN_FLOATING();
	SensorDelay_us(4);	
	SCL0_Pin_Output_High();	
	SensorDelay_us(4);	
	while((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))  && CNT < 100) 
	CNT++;
	if(CNT == 100)
	{
		return 0;
	}
 	SCL0_Pin_Output_Low();	
	SensorDelay_us(8);	
	return 1;
}

void Send_ACK0(void)		  //主机回复ACK信号
{
	SCL0_Pin_Output_Low();	
	SensorDelay_us(8);	
	SDA0_Pin_Output_Low();
	SensorDelay_us(8);	
	SCL0_Pin_Output_High();	
	SensorDelay_us(8);
	SCL0_Pin_Output_Low();	
	SensorDelay_us(8);
	SDA0_Pin_IN_FLOATING();
	SensorDelay_us(8);
}

void Send_NOT_ACK0(void)	//主机不回复ACK
{
	SCL0_Pin_Output_Low();	
	SensorDelay_us(8);
	SDA0_Pin_Output_High();
	SensorDelay_us(8);
	SCL0_Pin_Output_High();	
	SensorDelay_us(8);		
	SCL0_Pin_Output_Low();	
	SensorDelay_us(8);
    SDA0_Pin_Output_Low();
	SensorDelay_us(8);
}

void Stop_I2C0(void)	  //一条协议结束
{
	SCL0_Pin_Output_Low();	
	SDA0_Pin_Output_Low();
	SensorDelay_us(8);
	SCL0_Pin_Output_High();	
	SensorDelay_us(8);
	SDA0_Pin_Output_High();
	SensorDelay_us(8);
}

void EEPROM_Init(void)   //初始化
{	
	Init_I2C0_Sensor_Port();
}

void I2C_Send_Byte(uint8_t txd)
{
	uint8_t i=0;

	SDA0_Pin_Output_High();
	SCL0_Pin_Output_Low();

	for(i=0;i<8;i++)
	{
			if((txd&0x80)>0) //0x80  1000 0000
					SDA0_Pin_Output_High();
			else
					SDA0_Pin_Output_Low();

			txd<<=1;
			delay_us(5);
			SCL0_Pin_Output_High();
			delay_us(5); 
			SCL0_Pin_Output_Low();
			delay_us(5);
	}
}

uint8_t I2C_Read_Byte(uint8_t ack)
{
	uint8_t i=0,receive=0;

	SDA0_Pin_IN_FLOATING();
	for(i=0;i<8;i++)
	{
			SCL0_Pin_Output_Low();
			delay_us(2);
			SCL0_Pin_Output_High();
			receive<<=1;
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
				receive++;
			delay_us(1); 
	}
	if(ack==0)
		Send_NOT_ACK0();
	else
		Send_ACK0();
	return receive;
}

uint8_t ReadOneByte(uint16_t addr)
{
	uint8_t temp=0;
	I2C0_Start(); 
	I2C_Send_Byte(0xA0);//1010000
	Receive_ACK0();
	delay_us(5);
	I2C_Send_Byte(addr>>8);
	Receive_ACK0();
	I2C_Send_Byte(addr%256);
	Receive_ACK0();
	delay_us(5);
	I2C0_Start();
	I2C_Send_Byte(0xA1);//10100001
	Receive_ACK0();
	temp=I2C_Read_Byte(0); 
	Send_NOT_ACK0();
	Stop_I2C0(); 
	return temp; 
}

void WriteOneByte(uint16_t addr,uint8_t dt)
{
	I2C0_Start();
	I2C_Send_Byte(0xA0);
	Receive_ACK0();
	delay_us(6);
	I2C_Send_Byte(addr>>8); 
	Receive_ACK0();
	I2C_Send_Byte(addr%256); 
	Receive_ACK0();
	delay_us(6);
	I2C_Send_Byte(dt);
	Receive_ACK0();
	Stop_I2C0();
	Delay_1ms(10);
}
