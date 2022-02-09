#include "stm32f4xx_hal.h"  


void SDA0_Pin_Output_High(void)  ; //将PB9配置为输出 ， 并设置为高电平， PB9作为I2C的SDA
void SDA0_Pin_Output_Low(void);  //将P9配置为输出  并设置为低电平
void SDA0_Pin_IN_FLOATING(void);  //SDA配置为浮空输入
void SCL0_Pin_Output_High(void); //SCL输出高电平，P8作为I2C的SCL
void SCL0_Pin_Output_Low(void); //SCL输出低电平
void Init_I2C0_Sensor_Port(void); //初始化I2C接口,输出为高电平
void I2C0_Start(void);		 //I2C主机发送START信号
uint8_t Receive_ACK0(void);   //看EEPROM是否有回复ACK
void Send_ACK0(void)	;	  //主机回复ACK信号
void Send_NOT_ACK0(void);	//主机不回复ACK
void Stop_I2C0(void);	  //一条协议结束
void EEPROM_Init(void);   //初始化EEPROM
uint8_t ReadOneByte(uint16_t addr);
void WriteOneByte(uint16_t addr,uint8_t dt);
