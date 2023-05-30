/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define  float32_t			float

#define TOTAL_IC			2
#define TOTAL_VOLTAGES		12
#define TOTAL_TEMPERATURES	8
#define CELL_UV				2.85
#define CELL_OV				3.65
#define ADG_728_ADDRESS		0b1001100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
cell_asic cellASIC[TOTAL_IC];
float32_t voltages[TOTAL_IC][TOTAL_VOLTAGES];
float32_t temperatures[TOTAL_IC][TOTAL_TEMPERATURES];

//LTC CONFIGURATION VARIABLES
bool REFON = true; //!< Reference Powered Up Bit (true means Vref remains powered on between conversions)
bool ADCOPT = true; //!< ADC Mode option bit	(true chooses the second set of ADC frequencies)
bool gpioBits_a[5] = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5 (false -> pull-down on)
bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12 (all false -> no discharge enabled)
bool dctoBits[4] = {false, false, false, false}; //!< Discharge time value // Dcto 0,1,2,3	(all false -> discharge timer disabled)
uint16_t cellOV = (uint16_t) (CELL_OV*625.0f); // ovCount = U[microvolts]/(16*100)
uint16_t cellUV = (uint16_t) (CELL_UV*625.0f);


//ERROR VARIABLES
int8_t cvError = 0;
int8_t auxError = 0;	//hold if an error has occured while reading cell voltage and aux voltage values
int NV [TOTAL_IC][TOTAL_VOLTAGES];	//total number of voltage measurement errors per IC per cell
int NT [TOTAL_IC][TOTAL_TEMPERATURES];	//total number of temperature measurement errors per IC per cell
int NPEC_V = 0;	//number of communication errors in cell voltage measurement
int NPEC_T = 0;	//number of communication errors in temperature voltage measurement

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
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
	// local iterators for loops
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t k = 0;

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
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //TODO:error counter init

  LTC681x_initSPI(&hspi3);
  cs_high();
  LTC6811_init_cfg(TOTAL_IC, cellASIC);
  //This for loop initializes the configuration register variables
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
  {
	  LTC6811_set_cfgr(current_ic,cellASIC,REFON,ADCOPT,gpioBits_a,dccBits_a, dctoBits, cellUV, cellOV);
  }
  //sets the CRC count to 0
  LTC6811_reset_crc_count(TOTAL_IC, cellASIC);
  //Initializes the LTC's register limits for LTC6811 (because the generic LTC681x libraries can also be used for LTC6813 and others)
  LTC6811_init_reg_limits(TOTAL_IC, cellASIC);
  wakeup_sleep(TOTAL_IC);
  //writes the configuration variables in the configuration registers via SPI
  LTC6811_wrcfg(TOTAL_IC,cellASIC);

  //HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //Blink LED
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

	  //Voltage conversion
	  HAL_readCellVoltages();

	 //Temperature conversion
	  HAL_readCellTemperatures();

	 //Check for voltage differences and balancing logic
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEDR_Pin|LEDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDR_Pin LEDG_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin|LEDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void HAL_readCellVoltages()
{
	uint32_t i;
	uint32_t j;

	 wakeup_idle(TOTAL_IC);
	 LTC6811_adcv(MD_27KHZ_14KHZ, DCP_DISABLED, CELL_CH_ALL);

	 HAL_Delay(10);

	 wakeup_idle(TOTAL_IC);
	 cvError = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, cellASIC);
	 for(i=0; i<TOTAL_IC; i++)
	 {
		 for(j=0; j<12; j++)
		 {
			 voltages[i][j] = cellASIC[i].cells.c_codes[j] * 0.0001;
		 }
	 }
	 if(cvError)
	 {
		 //Message readback voltage failure. Reset error counter
		 cvError =0;
		 //TODO: CAN/UART message thet received voltage is faulty
	 }
}

static void HAL_readCellTemperatures()
{
	uint32_t i;
	uint32_t j;

	uint8_t dataH;
	uint8_t dataL;
	uint8_t temp;

	//Go over each sensor (per sensor conversion is slow so it is in an outer loop)
	for(i=0; i<TOTAL_TEMPERATURES; i++)
	{
		//Go over each ASIC
		for(j=0; j<TOTAL_IC; j++)
		{
			//Select the mux channel on ADG728: ch. read seq: 4,5,6,7,0,1,2,3
			temp = 0x01<<i;
			//Write to local MCU data structure to prepare data to be sent then send data
			cellASIC[j].com.tx_data[0] = 0b01101001; 			//ICOM0 = START,  ADG728AddrH
			cellASIC[j].com.tx_data[1] = 0b10000000; 			//ADG728AddrL,  FCOM0 = MasterACK
			cellASIC[j].com.tx_data[2] = (0b00000000 | temp);	//ICOM1 = BLANK, ADG728SelH
			cellASIC[j].com.tx_data[3] = (0b00000000 | temp);	//ADG728SelL, FCOM1 = MasterAck
			cellASIC[j].com.tx_data[4] = 0b00010000; 			//ICOM2 = STOP, 0000
			cellASIC[j].com.tx_data[5] = 0b00000000; 			//0000, FCOM2 = MasterAck
			//WRCOMM command
			LTC6811_wrcomm(TOTAL_IC, cellASIC);
			//STCOMM command (start I2C communication)
			LTC6811_stcomm(TOTAL_IC);
		 }
		 //Wait for mux to each ASIC to stabilize, read and parse MUX
		 HAL_Delay(10);

		 //MD = 0x02  - 7kHz mode
		 //CHG = 0x01 - measure on GPIO1 (MUXTEMP)
		 LTC6811_adax(0x02, 0x01);
		 //Read and parse each temperature for all asics
		 for(j=0; j<TOTAL_IC; j++)
		 {
			 auxError = LTC6811_rdaux(0, TOTAL_IC, cellASIC);
			 //TODO: interpolation function
			 temperatures[j][i] = cellASIC[j].aux.a_codes[i];
		 }
	 }
	 if(auxError)
	 {
		 //Message readback temperature failure. Reset error counter
		 auxError =0;
		 //TODO: CAN/UART message thet received temperature is faulty
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
