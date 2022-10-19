/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t hour, minute, second;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Display7Seg(uint8_t num);

void Enable7Seg(uint8_t index);

void UpdateLed7Seg(uint8_t index);

void UpdateClockBuffer();

const int MAX_LED_MATRIX = 8;
int index_led_matrix = 0;
int index_shift_left = 0;

uint16_t matrix_buffer[8] = {0x1800, 0x3C00, 0x6600, 0x6600, 0x7E00, 0x7E00, 0x6600, 0x6600};
uint16_t newMatrixBuffer[8] = {0x1800, 0x3C00, 0x6600, 0x6600, 0x7E00, 0x7E00, 0x6600, 0x6600};

void ShiftLeft();
void updateLEDMatrix(int index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TIME_CYCLE = 10;

uint8_t	timer0_counter = 0;
uint8_t	timer0_flag = 0;

uint8_t	timer1_counter = 0;
uint8_t	timer1_flag = 0;

void SetTimer0(uint16_t duration){
	timer0_counter = duration / TIME_CYCLE;
	timer0_flag = 0;
}

void SetTimer1(uint16_t duration){
	timer1_counter = duration / TIME_CYCLE;
	timer1_flag = 0;
}

void RunTimer0(){
	if(timer0_counter > 0){
		timer0_counter--;
		if(timer0_counter <= 0){
			timer0_flag = 1;
		}
	}
}

void RunTimer1(){
	if(timer1_counter > 0){
		timer1_counter--;
		if(timer1_counter <= 0){
			timer1_flag = 1;
		}
	}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  hour = 15; minute = 8; second = 50;

  uint8_t indexLed			= 0;

  SetTimer0(1000);
  SetTimer1(250);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//  UpdateClockBuffer();

//	  if(timer0_flag == 1){
//		  SetTimer0(1000);
//		//  ShiftLeft();
//		  minute++;
////		  if(second >= 60){
////			  second = 0;
////			  minute++;
////		  }
//		  if(minute >= 60){
//			  minute = 0;
//			  hour++;
//		  }
//		  if(hour > 24){
//			  hour = 0;
//		  }
//
//		  HAL_GPIO_TogglePin(DOT_GPIO_Port, DOT_Pin);
//	  }

	  if(timer0_flag == 1){
		  SetTimer0(1000);
		  ShiftLeft();
	  }
	  if(timer1_flag == 1){
		  SetTimer1(100);
//		  if(indexLed > 3) indexLed = 0;
//		  UpdateLed7Seg(indexLed++);
		  if(index_led_matrix > 8) index_led_matrix = 0;
		  updateLEDMatrix(index_led_matrix++);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENM0_Pin|DOT_Pin|led_Pin|en0_Pin
                          |en1_Pin|en2_Pin|en3_Pin|ENM1_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, seg0_Pin|seg1_Pin|seg2_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|seg3_Pin|seg4_Pin|seg5_Pin
                          |seg6_Pin|ROW0_Pin|ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENM0_Pin DOT_Pin led_Pin en0_Pin
                           en1_Pin en2_Pin en3_Pin ENM1_Pin
                           ENM2_Pin ENM3_Pin ENM4_Pin ENM5_Pin
                           ENM6_Pin ENM7_Pin */
  GPIO_InitStruct.Pin = ENM0_Pin|DOT_Pin|led_Pin|en0_Pin
                          |en1_Pin|en2_Pin|en3_Pin|ENM1_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : seg0_Pin seg1_Pin seg2_Pin ROW2_Pin
                           ROW3_Pin ROW4_Pin ROW5_Pin ROW6_Pin
                           ROW7_Pin seg3_Pin seg4_Pin seg5_Pin
                           seg6_Pin ROW0_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = seg0_Pin|seg1_Pin|seg2_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|seg3_Pin|seg4_Pin|seg5_Pin
                          |seg6_Pin|ROW0_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static uint32_t led7Seg[10] = { 0x003F0040, 0x00060079, 0x005B0024,
  								0x004F0030, 0x00660019, 0x006D0012,
  								0x007D0002, 0x00070078, 0x007F0000,
  								0x006F0010 };  //GPIOB
static uint32_t led7SegEnable[4] = {0x002001C0, 0x004001A0, 0x00800160,
								0x010000E0};    //GPIOA

uint8_t ledBuffer[4]		= {0, 0, 0, 0};
void Display7Seg(uint8_t num){
	if(num >= 0 && num <= 9){
		GPIOB->BSRR = led7Seg[num];
	}
}

void Enable7Seg(uint8_t index){
	if(index >= 0 && index <= 3){
		GPIOA->BSRR = led7SegEnable[index];
	}
}

void UpdateLed7Seg(uint8_t index){
	if(index >= 0 && index <= 3){
		Enable7Seg(index);
		Display7Seg( ledBuffer[index] );
	}
}

void UpdateClockBuffer(){
	ledBuffer[0] = hour / 10;
	ledBuffer[1] = hour % 10;
	ledBuffer[2] = minute / 10;
	ledBuffer[3] = minute % 10;
}


//const int MAX_LED_MATRIX = 8;
//int index_led_matrix = 0;
//uint8_t matrix_buffer[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
void ShiftLeft(){
//shift left
	for(uint8_t i = 0; i < MAX_LED_MATRIX; i++){
		newMatrixBuffer[i] = newMatrixBuffer[i] << 1;
	}

}

void updateLEDMatrix(int index){
    GPIOB->ODR = ~(newMatrixBuffer[index]);
    switch (index){
        case 0:
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM1_Pin | ENM2_Pin| ENM3_Pin| ENM4_Pin |
        								ENM5_Pin | ENM6_Pin| ENM7_Pin, SET);
			break;
        case 1:
        	HAL_GPIO_WritePin(GPIOA, ENM1_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM2_Pin| ENM3_Pin| ENM4_Pin |
        								ENM5_Pin | ENM6_Pin| ENM7_Pin, SET);
            break;
        case 2:
        	HAL_GPIO_WritePin(GPIOA, ENM2_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin| ENM3_Pin| ENM4_Pin |
        								ENM5_Pin | ENM6_Pin| ENM7_Pin, SET);
            break;
        case 3:
        	HAL_GPIO_WritePin(GPIOA, ENM3_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin| ENM2_Pin| ENM4_Pin |
        								ENM5_Pin | ENM6_Pin| ENM7_Pin, SET);
            break;
        case 4:
        	HAL_GPIO_WritePin(GPIOA, ENM4_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin| ENM2_Pin| ENM3_Pin |
        								ENM5_Pin | ENM6_Pin| ENM7_Pin, SET);
            break;
        case 5:
        	HAL_GPIO_WritePin(GPIOA, ENM5_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin| ENM2_Pin| ENM3_Pin |
        								ENM4_Pin | ENM6_Pin| ENM7_Pin, SET);
            break;
        case 6:
        	HAL_GPIO_WritePin(GPIOA, ENM6_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin| ENM2_Pin| ENM3_Pin |
        								ENM4_Pin | ENM5_Pin| ENM7_Pin, SET);
            break;
        case 7:
        	HAL_GPIO_WritePin(GPIOA, ENM7_Pin, RESET);
        	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin| ENM2_Pin| ENM3_Pin |
        								ENM4_Pin | ENM5_Pin| ENM6_Pin, SET);
            break;
        default:
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		RunTimer0();
		RunTimer1();
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
