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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define sampleSize 4096
#define adcBufferLength (sampleSize * 2)
#define bufferLength 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

uint16_t adcBuffer[adcBufferLength];
uint16_t adcBufferTemp[adcBufferLength];
uint32_t adcDistanceCount = 0;

//Conversion ratio variable:
float MM_PPR_Forward = (6300.0f / 765.0f);
float MM_PPR_Reverse = (11100.0f / 765.0f);

//Time delay variable
const int forwardDelay = 21;
const int reverseDelay = 21;

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
int forwardFinal = 0;
int reverse = 0;
int returnHome = 0;

/* Directional distance variables used to tell the motor how
 * far to travel in either of the two directions. Because the
 * motor moves in incremental steps, there are minimum travel
 * distances that the motor can move, which are approximately
 * a minimum of 6mm increments.
 */
int forwardDistance = 1500;
int reverseDistance = 1500;
int forwardFinalDistance = 500;

//Temporary placeholder variables
int stop_reset = 0;

//Bluetooth GUI module variables
uint8_t RxBuffer[bufferLength] = {0};
uint8_t mydata[] = "";
uint8_t mydata_bt[] = "";
uint8_t store_data[2] = "";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void run_Forward_Sequence(int);
void run_Reverse_Sequence(int);
void stop_Motor();
void motor_ReturnHome();

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialise the connection between the ADC Buffer and DMA. This
   * allows me to store all ADC readings directly into memory without
   * having to pass through the processor. This will prevent the
   * processor from slowing down during other computation.
   */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, adcBufferLength);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Before the motor receives commands, it must always move to the
   * left most position on the rail so that the first forward command
   * can execute.
   */
  while(!HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	//Brown
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);	//White
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	//Yellow
  }
  motor_ReturnHome();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* The first check-point or criterion that needs to be satisfied
	   * is that the E-Stop switch must not be depressed. When the E-Stop
	   * is depressed, it breaks the circuit and sends a logic low level to
	   * the board GPIO.
	   */
	  while(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){

		  /* The criterion for the forward movement for the motor is that the
		   * forward flag must be set and the right-most limit switch must not
		   * be depressed. If depressed, it implies the motor has reached the
		   * end of the rail. Once finished moving, the forward flag is reset
		   * and the reverse flag is set.
		   */
		  if (forward && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13)){
			  run_Forward_Sequence(forwardDistance);
			  stop_Motor();
			  adcDistanceCount = 0;
			  forward = 0;
			  forwardFinal = 0;
			  reverse = 1;
		  }

		  /* The criterion for the reverse movement for the motor is that the
		   * reverse flag must be set and the left-most limit switch must not
		   * be depressed. If depressed, it implies the motor has reached the
		   * end of the rail. Once finished moving, the forwardFinal flag is
		   * set and the reverse flag is reset.
		   */
		  else if(reverse && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15)){
			  run_Reverse_Sequence(reverseDistance);
			  stop_Motor();
			  adcDistanceCount = 0;
			  reverse = 0;
			  forward = 0;
			  forwardFinal = 1;
		  }

		  /* The criterion for this final forward movement is identical to
		   * that of the forward movement described above. When finished moving,
		   * all directional flags are reset and the motor must return back to
		   * its starting position.
		   */
		  else if(forwardFinal && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13)){
			  run_Forward_Sequence(forwardFinalDistance);
			  stop_Motor();
			  adcDistanceCount = 0;
			  motor_ReturnHome();
			  adcDistanceCount = 0;
			  forwardFinal = 0;
			  reverse = 0;
			  forward = 0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	//Brown
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);	//White
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	//Yellow
		  }
	  }

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	//Brown
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);	//White
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	//Yellow
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, White_Phase_Pin|Brown_Phase_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Yellow_Phase_GPIO_Port, Yellow_Phase_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : Stop_Motor_Exit_Pin */
  GPIO_InitStruct.Pin = Stop_Motor_Exit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Stop_Motor_Exit_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : E_Stop_Detection_Pin Right_Limit_Detection_Pin Left_Limit_Detection_Pin */
  GPIO_InitStruct.Pin = E_Stop_Detection_Pin|Right_Limit_Detection_Pin|Left_Limit_Detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*******************************/
/* Setting up code for the ADC */
/*******************************/

/* This half callback function provides the calculations required
 * on the first half of the DMA register being used.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){

	for(int i = 1; (i < sampleSize + 1); i++){
			adcBufferTemp[i-1] = adcBuffer[i-1];
			if((adcBufferTemp[i] > 2900) && (adcBufferTemp[i-1] < 500)){
				adcDistanceCount++;
			}
		}
}

/* This complete callback function provides the calculations
 * required on the second half of the DMA register being used.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	for(int i = sampleSize+1; (i < 2*sampleSize+1); i++){
			adcBufferTemp[i-1] = adcBuffer[i-1];
			if((adcBufferTemp[i] > 2900) && (adcBufferTemp[i-1] < 500)){
				adcDistanceCount++;
			}
		}
}

/*******************************/
/* Forward Operation of Motor  */
/*******************************/
void run_Forward_Sequence(int distance) {
	/* Inside this function, the goal is to cycle through the output pins
	 * associated to the motor segments to successfully move in the
	 * forward direction. The pins excitation or pin HIGH/LOW allocation
	 * is as follows:
	 * 1. Brown (Pin A4)
	 * 2. White (Pin A6)
	 * 3. Yellow (Pin C4)
	 * Exciting these three cable colours in this order will move the
	 * motor forward successfully (left to right on the rail).
	 *
	 * Before this can be done, I first need to convert the ADC reading to
	 * a valid distance measurement to tell the forward sequence to know
	 * when to stop cycling. */

	int adcCountReq = (int)(distance * MM_PPR_Forward);

	/* First, I check whether or not the E-Stop button is depressed before
	 * allowing the remainer of the code to run.
	 */
	while (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11) && adcDistanceCount < adcCountReq && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13)){
		/* Inside this while loop, I want to cycle through the output pins
		 * connected to the relays controlling which motor segment to excite.
		 */
		if (adcDistanceCount < adcCountReq && (forward || forwardFinal) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
			/* If all of the criteria for the if statement are met, the first
			 * of the three segments will be excited by writing the digital
			 * output pin high as follows:
			 *
			 * On a side note, because a P-channel transistor is implemented
			 * on the switching circuit, the logic level for the relays need to
			 * be inversed to switch. Therefore, whenever a pin is reset, the
			 * relay coil conducts.
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 	//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
			colourSelector = 0;
			HAL_Delay(forwardDelay);
		}

		if(adcDistanceCount < adcCountReq && (forward || forwardFinal) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
			/* If all the criteria for the if statement are met, the next
			 * segment will be excited by writing the digital output pin
			 * high as follows:
			 *
			 * On a side note, because a P-channel transistor is implemented
			 * on the switching circuit, the logic level for the relays need to
			 * be inversed to switch. Therefore, whenever a pin is reset, the
			 * relay coil conducts.
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
			colourSelector = 1;
			HAL_Delay(forwardDelay);
		}

		if(adcDistanceCount < adcCountReq && (forward || forwardFinal) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
			/* If all the criteria for the if statement are met, the final
			 * segment of the motor will be excited by writing the digital
			 * output pin high as follows:
			 *
			 * On a side note, because a P-channel transistor is implemented
			 * on the switching circuit, the logic level for the relays need to
			 * be inversed to switch. Therefore, whenever a pin is reset, the
			 * relay coil conducts.
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
			colourSelector = 2;
			HAL_Delay(forwardDelay);
		}
	}
	return;
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
	 * 2. White
	 * 3. Brown
	 * Exciting in this order will cause for the motor to run in the reverse
	 * direction (right to left on the rail)
	 *
	 * Again, the ADC PPR value needs to be determined first to allow the
	 * function to determine when the reverse sequence needs to stop running:*/

	int adcCountReq = (int)(distance * MM_PPR_Reverse);

	/* First I check to see if the E-Stop button has been depressed before
	 * running the remainder of the code.
	 */
	while (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11) && adcDistanceCount < adcCountReq && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15)) {
		/* Inside this while loop, I want to cycle through the output pins
		 * connected to the relays controlling which motor segment to excite.
		 * I first need to do the timing considerations before I can program
		 * the cycling sequence. */

		if (adcDistanceCount < adcCountReq && (reverse || returnHome) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
			/* If all of the criteria for the if statement are met, the first
			 * of the three segments will be excited by writing the digital
			 * output pin high as follows:
			 *
			 * On a side note, because a P-channel transistor is implemented
			 * on the switching circuit, the logic level for the relays need to
			 * be inversed to switch. Therefore, whenever a pin is reset, the
			 * relay coil conducts.
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  //Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);	   //White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	   //Yellow
			colourSelector = 0;
			HAL_Delay(reverseDelay);
		}

		if(adcDistanceCount < adcCountReq && (reverse || returnHome) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
			/* If all the criteria for the if statement are met, the next
			 * segment will be excited by writing the digital output pin
			 * high as follows:
			 *
			 * On a side note, because a P-channel transistor is implemented
			 * on the switching circuit, the logic level for the relays need to
			 * be inversed to switch. Therefore, whenever a pin is reset, the
			 * relay coil conducts.
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
			colourSelector = 2;
			HAL_Delay(reverseDelay);
		}

		if(adcDistanceCount < adcCountReq && (reverse || returnHome) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
			/* If all the criteria for the if statement are met, the final
			 * segment of the motor will be excited by writing the digital
			 * output pin high as follows:
			 *
			 * On a side note, because a P-channel transistor is implemented
			 * on the switching circuit, the logic level for the relays need to
			 * be inversed to switch. Therefore, whenever a pin is reset, the
			 * relay coil conducts.
			 */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
			colourSelector = 1;
			HAL_Delay(reverseDelay);
		}
	}
	return;
}

/*******************************/
/*   Stop Operation of Motor   */
/*******************************/

void stop_Motor(){
	/* The objective of this function is to excite a single segment of the motor
	 * that was last excited to stop the motor from moving. Essentially, I
	 * need to know which segment was excited last and then hold the excitation
	 * on that segment until  a new distance command is called or provided.*/

	while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)){
		if (colourSelector == 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
		}

		if (colourSelector == 1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);	//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Yellow
		}

		if (colourSelector == 2){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		//Brown
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//White
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//Yellow
		}
	}
	return;



	/* I need to figure out an exit strategy for this function. I am not yet sure how I
	 * can leave the stop_Motor function. Maybe the best way is to use the on-board
	 * push-button as my exit strategy. Once the button is pushed, exit
	 * the function.
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

	/* First I check to see if the E-Stop button is depressed before
	 * running the remainder of the code.
	 */
	returnHome = 1;
	if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11) && !HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15)){
		run_Reverse_Sequence(2000);
		stop_Motor();
		returnHome = 0;
		adcDistanceCount = 0;
	}
	return;
}

/*******************************/
/* 	 Blue tooth Module GUI     */
/*******************************/

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//
//	int temp = strlen((char*)mydata_bt);
//
//	for (int i = 0; i < temp; i++){
//		store_data[i] = mydata_bt[i];
//	}
//
//	//Create variables to provide on/off functionality
////	char on[1] = 'n';
////	char off[1] = 'f';
//
//	if (store_data[0] == 'n'){
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
//		store_data[0] = '0';
//		store_data[1] = '0';
//	}else if (store_data[0] == 'f'){
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
//		store_data[0] = '0';
//		store_data[1] = '0';
//	}
//
//}

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
