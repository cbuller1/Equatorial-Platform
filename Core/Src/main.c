/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// functions for motor run and direction.
void RunMotor(void);
void NormalDirection(void);
void FastNormalDirection(void);
void StopMotor(void);
void ReverseDirection(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define HOME_SWITCH_PIN GPIO_PIN_8
#define MAX_SWITCH_PIN GPIO_PIN_6
#define MOMENTARY_SWITCH_PIN GPIO_PIN_4
#define DIR_PIN GPIO_PIN_1
#define STEP_PIN GPIO_PIN_2
#define DIR_PORT GPIOA
#define STEP_PORT GPIOA
int stepDelay = 22000; // motor speed delay in us to match rotation of earth (linear distance of 174.2mm)
int lastState = 0; // if state is 0, unit does not move.
int direction = 1; // direction 1 starts platform in normal direction.

// timer setup for stepper motor delay
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  startState = HAL_GPIO_ReadPin(GPIOB, MOMENTARY_SWITCH_PIN); // START/STOP SWITCH

   // CHECK START SWITCH STATE
      	  if (startState == GPIO_PIN_RESET){
      		  lastState = 1; // sets motor state to run
      	  }

      	  else {
      		  StopMotor();
      	  }

   // START MOTOR BASED ON START SWITCH STATE
      	  RunMotor();

   // STOP AND REVERSE DIRECTION AT MAXIMUM SWITCH
      	  if (HAL_GPIO_ReadPin(GPIOB, MAX_SWITCH_PIN) == GPIO_PIN_RESET) {
    		  	StopMotor();
    		  	HAL_Delay(1000);
    		  	     // KEEP MOTOR GOING UNTIL IT REACHES HOME SWITCH
    		  	  	  while (HAL_GPIO_ReadPin(GPIOB, MAX_SWITCH_PIN) == GPIO_PIN_RESET) {
    		  	  		  lastState = 1;
    		  	  		  direction = 0; // 0 reverses the direction
    		  	  		  RunMotor();
    		  }
      	  }

  // STOP AND REVERSE DIRECTION AT HOME SWITCH BUT STOPS MOTOR UNTIL SYSTEM POWER RESET AFTER CLEARING SWITCH
      	  if (HAL_GPIO_ReadPin(GPIOB, HOME_SWITCH_PIN) == GPIO_PIN_RESET) {
                 StopMotor();
                 HAL_Delay(1000);

                   	  // REVERSE DIRECTION AND KEEP MOTOR GOING UNTIL IT CLEEARS SWITCH
                 	  while (HAL_GPIO_ReadPin(GPIOB, HOME_SWITCH_PIN) == GPIO_PIN_RESET) {
                 		  lastState = 1;
                 		  direction = 2; // 2 RUNS MOTOR AT HIGH SPEED TO CLEAR SWITCH QUICKLY
                 		  RunMotor();
                 	  }
                      // Stop Motor until System Reset
                      while (startState == GPIO_PIN_RESET) {
                    	 StopMotor();

                     }

      	  	  }
}



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}

void RunMotor(void) {

	 if (lastState == 1){

		  	        	  if (direction == 1){
		  	        		  NormalDirection();
		  	        	  }
		  	        	  if (direction == 2) {
		  	        		  FastNormalDirection();
		  	        	  }
		  	        	  else if (direction == 0){
		  	        		  ReverseDirection();

		  	        	  }
		  }
}

// RUNS PLATFORM IN NORMAL DIRECTION
void NormalDirection(void) {
    int i;
    // Set direction to clockwise
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);

    for (i = 0; i < 3; i = i + 1) {
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        microDelay(stepDelay);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        microDelay(stepDelay);
    }
	    // Monitor limit switches to change motor direction and stop when necessary
	}

// RUNS PLATFORM QUICKLY TO CLEAR HOME SWITCH
void FastNormalDirection(void) {
    int i;
    // Set direction to clockwise
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);

    for (i = 0; i < 3; i = i + 1) {
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        microDelay(stepDelay/200);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        microDelay(stepDelay/200);
    }
	}


// REVERSES PLATFORM/MOTOR DIRECTION
void ReverseDirection(void) {
    int i;
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

    for (i = 0; i < 3; i = i + 1) {
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        microDelay(stepDelay/100);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        microDelay(stepDelay/100);
    }
}


  void StopMotor(void) {
      // Stop the motor
	  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
	  lastState = 0;
  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1; // PRESCALE TIMER TO 64 IN ORDER TO RUN TIMER AT FREQUENCY OF 1MHZ FOR MOTOR DELAY
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  /*Configure GPIO pins : PB4 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
