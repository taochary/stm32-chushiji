/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "AHT20.h"
#include "EEPROM.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEST_IP_ADDR0   (uint8_t) 192
#define DEST_IP_ADDR1   (uint8_t) 168
#define DEST_IP_ADDR2   (uint8_t) 2
#define DEST_IP_ADDR3   (uint8_t) 19

#define UDP_SERVER_PORT   (uint16_t) 8192   /* define the UDP local connection port */
#define UDP_CLIENT_PORT   (uint16_t) 8192   /* define the UDP remote connection port */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

uint8_t   data[100];
__IO uint32_t message_count = 0;
struct udp_pcb *upcb;
uint32_t CT_data[2];
uint16_t c1,t1;
uint8_t ip_client[4] = {0};//0x10-0x13
uint8_t ip_dest[4] = {0};//0x20-0x23
uint8_t dest_port[2] = {0};
uint8_t client_port[2] = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	__HAL_TIM_ENABLE(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<us) //??,??
	{
	}
	__HAL_TIM_DISABLE(&htim6);
}

void udp_echoclient_connect(void)
{
  ip_addr_t DestIPaddr;
	uint16_t server_port,local_port;
  err_t err;
  server_port=(dest_port[0]<<8)+dest_port[1];
	local_port=(client_port[0]<<8)+client_port[1];
  /* Create a new UDP control block  */
  upcb = udp_new();
  
  if (upcb!=NULL)
  {
    /*assign destination IP address */
    IP4_ADDR( &DestIPaddr, ip_dest[0], ip_dest[1], ip_dest[2], ip_dest[3] );
    
		upcb->local_port=local_port;
    /* configure destination IP address and port */
    err= udp_connect(upcb, &DestIPaddr, server_port);
    
    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      udp_recv(upcb, udp_receive_callback, NULL);  
    }
		else
		{
			udp_remove(upcb);
		}
  }
}

void udp_echoclient_send(void)
{
  struct pbuf *p;
  
  data[0]=(t1>>8)&0xff;
	data[1]=t1&0xff;
  data[2]=(c1>>8)&0xff;
	data[3]=c1&0xff;
  /* allocate pbuf from pool*/
  p = pbuf_alloc(PBUF_TRANSPORT,4, PBUF_POOL);
  
  if (p != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(p, (char*)data, 4);
    
    /* send udp data */
    udp_send(upcb, p); 
    
    /* free pbuf */
    pbuf_free(p);
  }
}

static void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	uint8_t dat[16];
	uint8_t buf[16]={0};
	struct pbuf *ptemp;
	buf[0]='G';
	buf[1]='E';
	buf[2]='T';
	buf[3]=ip_client[0];
	buf[4]=ip_client[1];
	buf[5]=ip_client[2];
	buf[6]=ip_client[3];
	buf[7]=client_port[0];
	buf[8]=client_port[1];
	buf[9]=ip_dest[0];
	buf[10]=ip_dest[1];
	buf[11]=ip_dest[2];
	buf[12]=ip_dest[3];
	buf[13]=dest_port[0];
	buf[14]=dest_port[1];
	buf[15]=0xff;
  if(p!=NULL)
	{
		if(p->len==3)
		{
			memcpy((void*)dat,p->payload,p->len);
			if((dat[0]=='G')&&(dat[1]=='E')&&(dat[2]=='T'))
			{
				ptemp= pbuf_alloc(PBUF_TRANSPORT,16, PBUF_POOL);
			  pbuf_take(ptemp, (char*)buf,16);
				udp_send(upcb,ptemp);
				
			}
		}
		else if(p->len==16)
		{
			memcpy((void*)dat,p->payload,p->len);
			if((dat[0]=='S')&&(dat[1]=='E')&&(dat[2]=='T'))
			{
				WriteOneByte(0x10,dat[3]);
				WriteOneByte(0x11,dat[4]);
				WriteOneByte(0x12,dat[5]);
				WriteOneByte(0x13,dat[6]);
				WriteOneByte(0x14,dat[7]);
				WriteOneByte(0x15,dat[8]);
				WriteOneByte(0x20,dat[9]);
				WriteOneByte(0x21,dat[10]);
				WriteOneByte(0x22,dat[11]);
				WriteOneByte(0x23,dat[12]);
				WriteOneByte(0x24,dat[13]);
				WriteOneByte(0x25,dat[14]);
			}
		}
	}
  
  /* Free receive pbuf */
  pbuf_free(p);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//	Init_I2C0_Sensor_Port();
//	ip_client[0]=ReadOneByte(0x10);
//	ip_client[1]=ReadOneByte(0x11);
//	ip_client[2]=ReadOneByte(0x12);
//	ip_client[3]=ReadOneByte(0x13);
//	client_port[0]=ReadOneByte(0x14);
//	client_port[1]=ReadOneByte(0x15);
//	ip_dest[0]=ReadOneByte(0x20);
//	ip_dest[1]=ReadOneByte(0x21);
//	ip_dest[2]=ReadOneByte(0x22);
//	ip_dest[3]=ReadOneByte(0x23);
//	dest_port[0]=ReadOneByte(0x24);
//	dest_port[1]=ReadOneByte(0x25);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	Init_I2C0_Sensor_Port();
	ip_client[0]=ReadOneByte(0x10);
	ip_client[1]=ReadOneByte(0x11);
	ip_client[2]=ReadOneByte(0x12);
	ip_client[3]=ReadOneByte(0x13);
	client_port[0]=ReadOneByte(0x14);
	client_port[1]=ReadOneByte(0x15);
	ip_dest[0]=ReadOneByte(0x20);
	ip_dest[1]=ReadOneByte(0x21);
	ip_dest[2]=ReadOneByte(0x22);
	ip_dest[3]=ReadOneByte(0x23);
	dest_port[0]=ReadOneByte(0x24);
	dest_port[1]=ReadOneByte(0x25);
  MX_LWIP_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	udp_echoclient_connect();
	AHT20_Init();
	Delay_1ms(500);
	/***********************************************************************************/
	/**///②上电第一次发0x71读取状态字，判断状态字是否为0x18,如果不是0x18,进行寄存器初始化
	/***********************************************************************************/
	if((AHT20_Read_Status()&0x18)!=0x18)
	{
	AHT20_Start_Init(); //重新初始化寄存器
	Delay_1ms(10);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MX_LWIP_Process();
		HAL_GPIO_TogglePin(GPIOE, STA_LED_Pin);
		AHT20_Read_CTdata_crc(CT_data);  //crc校验后，读取AHT20的温度和湿度数据 


		c1 = CT_data[0]*100*10/1024/1024;  //计算得到湿度值c1（放大了10倍）
		t1 = CT_data[1]*200*10/1024/1024-500;//计算得到温度值t1（放大了10倍）
		if(c1>=650)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	
		else if(c1<=450)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		if(c1>=850)
			HAL_GPIO_WritePin(GPIOE, ERR_LED_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOE, ERR_LED_Pin, GPIO_PIN_RESET);
    udp_echoclient_send();
		Delay_1ms(500);
	 }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STA_LED_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PHY_RESET_GPIO_Port, PHY_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OPEN_LOCK_GPIO_Port, OPEN_LOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STA_LED_Pin ERR_LED_Pin */
  GPIO_InitStruct.Pin = STA_LED_Pin|ERR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PHY_RESET_Pin */
  GPIO_InitStruct.Pin = PHY_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PHY_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OPEN_LOCK_Pin */
  GPIO_InitStruct.Pin = OPEN_LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OPEN_LOCK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
