/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  int MOTOR_FL = 0;
  int MOTOR_FR = 1;
  int MOTOR_RL = 2;
  int MOTOR_RR = 3;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motorControl(double vel1, double vel2, double vel3, double vel4);
void actControl(int type, int pwm1, int pwm2, int pwm3, int pwm4);
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
  union {
  	  double v;
  	  uint8_t bytes[sizeof(double)];
  } v[4];
  int cnt = 0;
  uint8_t byte;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);

  while (1)
  {

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

   /* USER CODE BEGIN 3 */
    cnt = 0;
	while(1)
	{
		if (HAL_UART_Receive(&huart2, &byte, 1, 100) == HAL_OK)
		{
			v[(int)(cnt/8)].bytes[cnt%8] = byte;
			cnt++;
			HAL_UART_Transmit(&huart2, &byte, 1, 100);
			if(cnt>=32)
			{
				cnt = 0;
				//motor control
				motorControl(v[0].v, v[1].v, v[2].v, v[3].v);
			}
		}
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
		  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void controlRotateDirection(int motorNum, double vel)
{
	switch(motorNum)
	{
	case 0:
		if(vel > 0.0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
		}
		else{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);
		}
		break;
	case 1:
			if(vel > 0.0){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
			}
			else{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			}
			break;
	case 2:
			if(vel > 0.0){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			}
			else{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
			}
			break;

	case 3:
			if(vel > 0.0){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
			}
			else{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
			}
			break;
	}
}

void motorControl(double vel1, double vel2, double vel3, double vel4)
{
	//double abs1 = abs(vel1) > abs(vel2) ? abs(vel1) : abs(vel2);
	//double abs2 = abs(vel3) > abs(vel4) ? abs(vel3) : abs(vel4);
	//double abs_max = abs1 > abs2 ? abs1 : abs2;
	if(vel1 == 0.0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
	}
	else{
		controlRotateDirection(0, vel1);
		if(vel1 <0.0){
			vel1 = vel1* -1;
		}
		int pwm = (vel1 / 8.0) * 500.0;
		TIM1->CCR1 = pwm;
	}
	if(vel2 == 0.0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
	}
	else{
		controlRotateDirection(1, vel2);
		if(vel2<0.0){
			vel2 = vel2* -1;
		}
		int pwm = (vel2 / 8.0) * 500.0;
		TIM1->CCR2 = pwm;
	}
	if(vel3 == 0.0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
	}
	else{
		controlRotateDirection(2, vel3);
		if(vel3<0.0){
			vel3 = vel3 * -1;
		}
		int pwm = (vel3 / 8.0) * 500.0;
		TIM1->CCR3 = pwm;
	}
	if(vel4 == 0.0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
	}
	else{
		controlRotateDirection(3, vel4);
		if(vel4<0.0){
			vel4 =vel4 * -1;
		}
		int pwm = (vel4 / 8.0) * 500.0;
		TIM1->CCR4 = pwm;
	}
}

void actControl(int type, int pwm1, int pwm2, int pwm3, int pwm4)
{
	switch(type)
	{
		case 0:  //forward
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
			TIM1->CCR1 = pwm1;//543;
			TIM1->CCR2 = pwm2;//512;
			TIM1->CCR3 = pwm3;//537;
			TIM1->CCR4 = pwm4;//512;
			break;
		case 1:  //backward
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
			TIM1->CCR1 = pwm1;//543;
			TIM1->CCR2 = pwm2;//512;
			TIM1->CCR3 = pwm3;//537;
			TIM1->CCR4 = pwm4;//512;
			break;
		case 2:  //clockwise angular -
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
			TIM1->CCR1 = pwm1;//543;
			TIM1->CCR2 = pwm2;//512;
			TIM1->CCR3 = pwm3;//537;
			TIM1->CCR4 = pwm4;//512;
			break;
		case 3:  //counterclockwise angular +
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
			TIM1->CCR1 = pwm1;//543;
			TIM1->CCR2 = pwm2;//512;
			TIM1->CCR3 = pwm3;//537;
			TIM1->CCR4 = pwm4;//512;
			break;
		case 4:  //stop
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
