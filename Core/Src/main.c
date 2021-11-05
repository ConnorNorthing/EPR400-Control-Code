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
#define sampleSize 2500
#define ADC_BufferLength (sampleSize * 2)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

uint16_t adcBuffer[ADC_BufferLength];
uint16_t adcDistanceCount = 0;

//Conversion ratio variable:
uint16_t MM_PPR_Conversion = 0;

//Segment identification variable:
/* if the colourSelector = 0 -> White segment excited
 * if the colourSelector = 1 -> Brown segment excited
 * if the colourSelector = 2 -> Yellow segment excited
 */
int colourSelector = 0;

/* Flag variables used to determine which direction the motor
 * may run.
 */
int forward = 1;
int forwardFinal = 1;
int reverse = 0;

/* Directional distance variables used to tell the motor how
 * far to travel in either of the two directions. Because the
 * motor moves in incremental steps, there are minimum travel
 * distances that the motor can move, which are approximately
 * a minimum of 6mm increments.
 */
int forwardDistance = 0;
int reverseDistance = 0;
int forwardFinalDistance = 0;

//Temporary placeholder variables
int stop_reset = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adcBuffer, ADC_BufferLength);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while(!__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_11)){
		  motor_ReturnHome();

		  if (forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
			  run_Forward_Sequence(forwardDistance);
			  stop_Motor();
			  adcDistanceCount = 0;
			  forward = 0;
			  forwardFinal = 0;
			  reverse = 1;
		  }
		  else if(reverse && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)){
			  run_Reverse_Sequence(reverseDistance);
			  stop_Motor();
			  adcDistanceCount = 0;
			  reverse = 0;
			  forward = 0;
			  forwardFinal = 1;
		  }
		  else if(forwardFinal && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
			  run_Forward_Sequence(forwardFinalDistance);
			  stop_Motor();
			  adcDistanceCount = 0;
			  forwardFinal = 0;
			  reverse = 0;
			  forward = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1500;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, White_Phase_Pin|Brown_Phase_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Yellow_Phase_GPIO_Port, Yellow_Phase_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : White_Phase_Pin Brown_Phase_Pin */
  GPIO_InitStruct.Pin = White_Phase_Pin|Brown_Phase_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Yellow_Phase_Pin */
  GPIO_InitStruct.Pin = Yellow_Phase_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Yellow_Phase_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Stop_Interrupt_Pin Right_Limit_Switch_Pin Left_Limit_Switch_Pin */
  GPIO_InitStruct.Pin = E_Stop_Interrupt_Pin|Right_Limit_Switch_Pin|Left_Limit_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*******************************/
/* Setting up code for the ADC */
/*******************************/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adcBuffer, ADC_BufferLength);
}

/*******************************/
/* Forward Operation of Motor  */
/*******************************/
void run_Forward_Sequence(int distance) {
	/* Inside this function, the goal is to cycle through the output pins
	 * associated to the motor segments to successfully move in the
	 * forward direction. The pins excitation or pin HIGH/LOW allocation
	 * is as follows:
	 * 1. White (Pin A2)
	 * 2. Brown (Pin A4)
	 * 3. Yellow (Pin A6)
	 * Exciting these three cable colours in this order will move the
	 * motor forward successfully (left to right on the rail).
	 *
	 * Before this can be done, I first need to convert the ADC reading to
	 * a valid distance measurement to tell the forward sequence to know
	 * when to stop cycling. */

	int adcCountReq = distance * MM_PPR_Conversion;

	while (!__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_11)){
		/* Inside this while loop, I want to cycle through the output pins
		 * connected to the relays controlling which motor segment to excite.
		 */
		if (adcDistanceCount < adcCountReq && forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
			/* If all of the criteria for the if statement are met, the first
			 * of the three segments will be excited by writing the digital
			 * output pin high as follows:
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	//White
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
			colourSelector = 0;
			HAL_Delay(500);
		}

		if(adcDistanceCount < adcCountReq && forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
			/* If all the criteria for the if statement are met, the next
			 * segment will be excited by writing the digital output pin
			 * high as follows:
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
			colourSelector = 1;
			HAL_Delay(500);
		}

		if(adcDistanceCount < adcCountReq && forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
			/* If all the criteria for the if statement are met, the final
			 * segment of the motor will be excited by writing the digital
			 * output pin high as follows:
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
			colourSelector = 2;
			HAL_Delay(500);
		}
	}
}

/*******************************/
/* Reverse Operation of Motor  */
/*******************************/

void run_Reverse_Sequence(int distance){
	/* Similarly as for the run_Forward_Sequence() function, the goal is to just
	 * reverse the direction of travel for the motor by reversing the excitation
	 * cycling of the motor segments. Therefore, the excitation cycling will
	 * will following the pattern below:
	 * 1. Yellow
	 * 2. Brown
	 * 3. White
	 * Exciting in this order will cause for the motor to run in the reverse
	 * direction (right to left on the rail)
	 *
	 * Again, the ADC PPR value needs to be determined first to allow the
	 * function to determine when the reverse sequence needs to stop running:*/

	int adcCountReq = distance * MM_PPR_Conversion;

	while (!__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_11)) {
		/* Inside this while loop, I want to cycle through the output pins
		 * connected to the relays controlling which motor segment to excite.
		 * I first need to do the timing considerations before I can program
		 * the cycling sequence. */

		if (adcDistanceCount < adcCountReq && forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)){
			/* If all of the criteria for the if statement are met, the first
			 * of the three segments will be excited by writing the digital
			 * output pin high as follows:
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
			colourSelector = 2;
			HAL_Delay(500);
		}

		if(adcDistanceCount < adcCountReq && forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)){
			/* If all the criteria for the if statement are met, the next
			 * segment will be excited by writing the digital output pin
			 * high as follows:
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
			colourSelector = 1;
			HAL_Delay(500);
		}

		if(adcDistanceCount < adcCountReq && forward && !__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)){
			/* If all the criteria for the if statement are met, the final
			 * segment of the motor will be excited by writing the digital
			 * output pin high as follows:
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//White
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
			colourSelector = 0;
			HAL_Delay(500);
		}
	}
}

/*******************************/
/*   Stop Operation of Motor   */
/*******************************/

void stop_Motor(){
	/* The objective of this function is to excite a single segment of the motor
	 * that was last excited to stop the motor from moving. Essentially, I
	 * need to know which segment was excited last and then hold the excitation
	 * on that segment until  a new distance command is called or provided.*/

	while (colourSelector == 0 && !stop_reset){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//White
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
		HAL_Delay(1000);
	}

	while (colourSelector == 1 && !stop_reset){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//Brown
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
		HAL_Delay(1000);
	}

	while (colourSelector == 2 && !stop_reset){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
		HAL_Delay(1000);
	}



	/* I need to figure out an exit strategy for this function. I am not yet sure how I
	 * can leave the stop_Motor function. Maybe the best way is to use the on-board
	 * push-button on the board as my exit strategy. Once the button is pushed, exit
	 * the function.
	 */

	/* I think using a delay should work well enough just to "pause" the motor for long
	 * enough until the motor needs to move again.
	 */
}

/**********************************/
/* Return Home Operation of Motor */
/**********************************/

void motor_ReturnHome(){
	/* The objective of this function is to move the motor from any position on the
	 * motor rail back to the left-most position on the rail and only stop once
	 * the left-most limit switch has been triggered.
	 */

	if (!__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_11)){
		run_Reverse_Sequence(1000);
	}
}

/*****************************/
/* Interrupt Service Routine */
/*****************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/* The first interrupt with the highest priority that I want to set up is the
	 * E-stop. This interrupt is slightly different because the button will stay
	 * depressed for a longer period of time and needs to be manually disengaged:
	 */

	while (GPIO_Pin == GPIO_PIN_11){
		//Turn off all motor outputs
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//White
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//Brown
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
	}

	/* The second interrupt that I want to do is set the right limit switch interrupt
	 * with a low priority that only gets depressed once.
	 */

	if (GPIO_Pin == GPIO_PIN_13){
		/* If the right-most limit switch is hit, the motor needs to immediately stop
		 * moving and change to the distances that need to be run in the opposite
		 * direction (change from forward to reverse).
		 */
		stop_Motor();
		adcDistanceCount = 0;
		forward = 0;
		reverse = 1;
	}

	/* The last interrupt that I want to do is set the left limit switch interrupt
	 * with a low priority that only gets depressed once.
	 */

	if (GPIO_Pin == GPIO_PIN_15){
		/* If the left-most limit switch is hit, the motor needs to immediately stop
		 * moving and change to the distances that need to be run in the opposite
		 * direction (change from reverse to forward).
		 */
		stop_Motor();
		adcDistanceCount = 0;
		reverse = 0;
		forward = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM13 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13) {
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
