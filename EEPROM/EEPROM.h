#include "stm32f4xx_hal.h"  


void SDA0_Pin_Output_High(void)  ; //��PB9����Ϊ��� �� ������Ϊ�ߵ�ƽ�� PB9��ΪI2C��SDA
void SDA0_Pin_Output_Low(void);  //��P9����Ϊ���  ������Ϊ�͵�ƽ
void SDA0_Pin_IN_FLOATING(void);  //SDA����Ϊ��������
void SCL0_Pin_Output_High(void); //SCL����ߵ�ƽ��P8��ΪI2C��SCL
void SCL0_Pin_Output_Low(void); //SCL����͵�ƽ
void Init_I2C0_Sensor_Port(void); //��ʼ��I2C�ӿ�,���Ϊ�ߵ�ƽ
void I2C0_Start(void);		 //I2C��������START�ź�
uint8_t Receive_ACK0(void);   //��EEPROM�Ƿ��лظ�ACK
void Send_ACK0(void)	;	  //�����ظ�ACK�ź�
void Send_NOT_ACK0(void);	//�������ظ�ACK
void Stop_I2C0(void);	  //һ��Э�����
void EEPROM_Init(void);   //��ʼ��EEPROM
uint8_t ReadOneByte(uint16_t addr);
void WriteOneByte(uint16_t addr,uint8_t dt);
