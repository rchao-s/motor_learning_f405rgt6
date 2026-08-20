/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
 

#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
#define LENGTH 50

typedef enum {
    MODE_DUTY = 0,
    MODE_RPM  = 1
} WorkMode_t;

WorkMode_t current_mode = MODE_DUTY;

#define POLE_PAIRS  2           // 电机极对数
#define EDGES_PER_REV (POLE_PAIRS * 6)  // 每转边沿数 = 6
#define MAX_PERIOD_US 1000000   // 最大间隔 1s，防止异常
#define RPM_FILTER_SIZE 4

#define DUTY_MIN_RPM_MODE   10.0f
#define DUTY_MAX_RPM_MODE   99.0f
#define RPM_DEADBAND        30
//#define STARTUP_DUTY        30.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

uint8_t RxBuff[LENGTH];
volatile uint8_t hall_state=0;
volatile uint32_t last_edge_time = 0;    // 上次霍尔边沿时间戳 (us)
volatile uint32_t hall_period_us = 0;    // 霍尔边沿间隔 (us)
volatile uint32_t last_pi_tick = 0;

uint16_t pwm_ccr=50;     //arr�?100，ccr是多少，占空比就是多�?
uint16_t duty_target=50; //占空比的目标�?

uint16_t rpm_target = 3000;  // 转�?�目标�?? RPM
uint32_t current_rpm;

uint8_t START_CMD=0;// 电机转或不转

uint32_t dutyacc=0;  //梯形加减速计数，每隔�?段时间就改变�?次占空比，实现�?�渐加减�?

float rpm_Kp=0.001f;
float rpm_Ki=0.00002f;

int16_t error_sum=0;


uint32_t rpm_filter_buf[RPM_FILTER_SIZE] = {0};
uint8_t rpm_filter_idx = 0;
float pwm_cmd_f = 30.0f;
float rpm_integral = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void	duty_target2pwm_ccr(void);	
void  six_step(void);
void  stop_pwm(void);
uint8_t read_hall_state(void);
void dma_printf(const char *format, ...);
void update_rpm(void);
void PI2pwm_ccr(void);
void print_status(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
///////////////////////////////////////////////////////////////////////////////////////////

HAL_TIM_Base_Start(&htim2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxBuff, sizeof(RxBuff));
	printf("OK\r\n");
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_ccr);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_ccr);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_ccr);
	
	hall_state=read_hall_state();
	//pre_position();		
	
	
	
////////////////////////////////////////////////////////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm_ccr);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pwm_ccr);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pwm_ccr);  

		if(START_CMD==1)
		{
			if(current_mode == MODE_DUTY)
			{
				duty_target2pwm_ccr();
				six_step();
			}
			else if(current_mode == MODE_RPM)
			{
				if (HAL_GetTick() - last_pi_tick >= 20)
			{
        last_pi_tick = HAL_GetTick();
        PI2pwm_ccr();
			}
				six_step();
			}
		}
		else
		{
			stop_pwm();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_A_Pin|L_B_Pin|L_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L_A_Pin L_B_Pin L_C_Pin */
  GPIO_InitStruct.Pin = L_A_Pin|L_B_Pin|L_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_A_Pin HALL_B_Pin HALL_C_Pin */
  GPIO_InitStruct.Pin = HALL_A_Pin|HALL_B_Pin|HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//uint8_t A,B,C;
	if (GPIO_Pin==HALL_A_Pin || GPIO_Pin==HALL_B_Pin || GPIO_Pin==HALL_C_Pin)
	{				
		update_rpm();
		hall_state=read_hall_state();
		
		//hall_state=(A << 2) | (B << 1) | C;// 等价�? 4*A + 2*B + C，但效率更高
	}


}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	{
    if (huart->Instance == USART3)
    {
        if (Size >= sizeof(RxBuff)) Size = sizeof(RxBuff) - 1;
        RxBuff[Size] = '\0';

        while (Size > 0 && (RxBuff[Size - 1] == '\r' || RxBuff[Size - 1] == '\n'))
        {
            RxBuff[Size - 1] = '\0';
            Size--;
        }

        if (strncmp((char*)RxBuff, "duty ", 5) == 0)
        {
            START_CMD = 1;
            current_mode = MODE_DUTY;
            duty_target = atoi((char*)&RxBuff[5]);
            if (duty_target < 10) duty_target = 10;
            if (duty_target > 99) duty_target = 99;
            print_status();
        }
        else if (strncmp((char*)RxBuff, "rpm ", 4) == 0)
        {
            START_CMD = 1;
            current_mode = MODE_RPM;
            rpm_target = atoi((char*)&RxBuff[4]);
            error_sum = 0;
            print_status();
        }
        else if (strcmp((char*)RxBuff, "mode_duty") == 0)
        {
            START_CMD = 1;
            current_mode = MODE_DUTY;
            print_status();
        }
        else if (strcmp((char*)RxBuff, "mode_rpm") == 0)
        {
            START_CMD = 1;
            current_mode = MODE_RPM;
            error_sum = 0;
            print_status();
        }
        else if (strcmp((char*)RxBuff, "start") == 0)
        {
            START_CMD = 1;
            print_status();
        }
        else if (strcmp((char*)RxBuff, "stop") == 0)
        {
            START_CMD = 0;
            print_status();
        }
        else if (strcmp((char*)RxBuff, "status") == 0)
        {
            print_status();
        }
        else
        {
            dma_printf("Unknown: %s\r\n", RxBuff);
        }

        memset(RxBuff, 0, sizeof(RxBuff));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxBuff, sizeof(RxBuff));
    }
	
	}
}
void duty_target2pwm_ccr()
{
	if(pwm_ccr!=duty_target)
			{
				dutyacc++;
				if(dutyacc==10000)
				{
					dutyacc=0;
					if(pwm_ccr<duty_target)
						pwm_ccr++;
					else
						pwm_ccr--;
					if (pwm_ccr%10==0) dma_printf("pwm_ccr:%d\r\n",pwm_ccr);
					if(pwm_ccr==duty_target) dma_printf("pwm_ccr:%d\r\n",pwm_ccr);
				}
			}
			else
			{
				pwm_ccr=duty_target;
				dutyacc=0;
			}
}	
void six_step()
{

	hall_state=read_hall_state();
	switch(hall_state)
	{

		case 4:
		{
			HAL_TIM_PWM_Start	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_RESET);
			break;
		}
		
		case 5:
		{
			HAL_TIM_PWM_Start	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_SET);
			break;
		}
		
		case 1:
		{
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_SET);
			break;
		}
		
		case 3:
		{
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_RESET);
			break;
		}
		case 2:
		{
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_RESET);
			break;
		}
		
		case 6:
		{
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_RESET);
			break;
		}
		default:
    stop_pwm();
    break;

	}
}
void stop_pwm()
{
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop	(&htim1,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin	(GPIOB,L_A_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin	(GPIOB,L_C_Pin,GPIO_PIN_RESET);
}

uint8_t read_hall_state(void)
{
    uint8_t A, B, C;

    A = HAL_GPIO_ReadPin(GPIOC, HALL_A_Pin);
    B = HAL_GPIO_ReadPin(GPIOC, HALL_B_Pin);
    C = HAL_GPIO_ReadPin(GPIOC, HALL_C_Pin);

    return (A << 2) | (B << 1) | C;
}


void PI2pwm_ccr(void)
{

    int32_t rpm;
    int32_t error_rpm;
    float duty_delta;

    rpm = (int32_t)current_rpm;
    error_rpm = (int32_t)rpm_target - rpm;

//    /* 启动阶段：没测速时先给一个能转起来的占空比 */
//    if (rpm == 0)
//    {
//        if (pwm_cmd_f < STARTUP_DUTY)
//        {
//            pwm_cmd_f = STARTUP_DUTY;
//        }

//        pwm_ccr = (uint16_t)(pwm_cmd_f + 0.5f);
//        return;
//    }

    /* 小误差死区 */
    if (error_rpm > -RPM_DEADBAND && error_rpm < RPM_DEADBAND)
    {
        pwm_ccr = (uint16_t)(pwm_cmd_f + 0.5f);
        return;
    }

    /* 积分 */
    rpm_integral += (float)error_rpm;

    if (rpm_integral > 50000.0f)  rpm_integral = 50000.0f;
    if (rpm_integral < -50000.0f) rpm_integral = -50000.0f;

    duty_delta = rpm_Kp * (float)error_rpm + rpm_Ki * rpm_integral;

    /* 每 20ms 最多变 1% duty，防止猛冲 */
    if (duty_delta > 1.0f)  duty_delta = 1.0f;
    if (duty_delta < -1.0f) duty_delta = -1.0f;

    /* 关键：不要直接把 duty_delta 转成 int，要先累加到 float */
    pwm_cmd_f += duty_delta;

    if (pwm_cmd_f < DUTY_MIN_RPM_MODE) pwm_cmd_f = DUTY_MIN_RPM_MODE;
    if (pwm_cmd_f > DUTY_MAX_RPM_MODE) pwm_cmd_f = DUTY_MAX_RPM_MODE;

    pwm_ccr = (uint16_t)(pwm_cmd_f + 0.5f);

}

void update_rpm(void)
{
     uint32_t period_us;
    uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t single_rpm;
    
    if (last_edge_time == 0) {
        last_edge_time = current_time;
        return;
    }
    
    period_us = current_time - last_edge_time;
    last_edge_time = current_time;
    
    if (period_us < 50 || period_us > 200000) {
        return;
    }
    
    single_rpm = 60000000UL / (period_us *EDGES_PER_REV);
    if (single_rpm > 20000) single_rpm = 20000;
    
    // 低速（周期 > 10ms）：不滤波，立即响应
    if (period_us > 10000) {
        current_rpm = single_rpm;
        return;
    }
    
    // 中高速：移动平均滤波
    rpm_filter_buf[rpm_filter_idx] = single_rpm;
    rpm_filter_idx++;
    if (rpm_filter_idx >= RPM_FILTER_SIZE) rpm_filter_idx = 0;
    
    uint32_t sum = 0;
    for (int i = 0; i < RPM_FILTER_SIZE; i++) {
        sum += rpm_filter_buf[i];
    }
    current_rpm = sum / RPM_FILTER_SIZE;
}



void print_status(void)
{
    if (current_mode == MODE_DUTY)
    {
        dma_printf("State: %s, Mode: DUTY\r\n"
                   "Duty Target: %d, Actual: %d\r\n"
                   "Speed: %d RPM\r\n",
                   START_CMD ? "RUN" : "STOP",
                   duty_target, pwm_ccr,
                   current_rpm);
    }
    else  // MODE_RPM
    {
        dma_printf("State: %s, Mode: RPM\r\n"
                   "RPM Target: %d, Actual: %d\r\n"
                   "Duty: %d\r\n",
                   START_CMD ? "RUN" : "STOP",
                   rpm_target, current_rpm,
                   pwm_ccr);
    }
}



int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;

}



void dma_printf(const char *format, ...)
{
    static char buffer[256];  // 静�?�缓冲区，避免被�?�?
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer) - 1, format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer))
    {
        // 使用 DMA 发�??
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)buffer, len);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


/* USER CODE END 4 */

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
