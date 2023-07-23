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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
cell_asic cellASIC[TOTAL_IC];
float32_t voltages[TOTAL_IC][TOTAL_VOLTAGES];
float32_t temperatures[TOTAL_IC][TOTAL_TEMPERATURES];

//LTC CONFIGURATION VARIABLES
bool REFON = true; //!< Reference Powered Up Bit (true means Vref remains powered on between conversions)
bool ADCOPT = true; //!< ADC Mode option bit	(true chooses the second set of ADC frequencies)
bool gpioBits_a[5] = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5 (false -> pull-down on)
bool dccBits_a[TOTAL_IC][TOTAL_VOLTAGES];//!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12 (all false -> no discharge enabled)
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

//BAL Variables
HAL_BALState_e balancingState = HAL_BALSTDBY;
uint32_t balancingCounter;
float32_t batteryPackCurrent;

//GLOBAL STATE VARIABLE
HAL_GlobalState_e globalState = HAL_GLOBALSTDBY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //Set DCCBits to false (balancing off)
  for(uint16_t currentIC=0; currentIC<TOTAL_IC; currentIC++)
  {
	  for(uint16_t currentCell=0; currentCell<TOTAL_VOLTAGES; currentCell++)
	  {
		  dccBits_a[currentIC][currentCell] = false;
	  }

  }

  //TODO:error counter init
  cs_high();
  //Clear array for initialization
  LTC6811_init_cfg(TOTAL_IC, cellASIC);
  //This for loop initializes the configuration register variables
  for (uint16_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
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

  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999999;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

	 //wakeup_idle(TOTAL_IC);
	 LTC6811_adcv(MD_27KHZ_14KHZ, DCP_DISABLED, CELL_CH_ALL);

	 delay_us(10000);

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
	 //TODO: check if there is overvoltage or undervoltage and enable/disable contactor
	 for(i=0; i<TOTAL_IC; i++)
	 {
		 for(j=0; j<12; j++)
		 {
			 if(voltages[i][j]>=CELL_OV)
			 {
				 //global state change
				 globalState=HAL_GLOBALCELLOV;
			 }
			 if(voltages[i][j]<=CELL_UV && voltages[i][j]>=CELL_NONZEROV)
			 {
				 globalState=HAL_GLOBALCELLUV;
			 }
		 }
	 }
}

static void HAL_readCellTemperatures()
{
	uint32_t i;
	uint32_t j;

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
			cellASIC[j].com.tx_data[1] = 0b10001000; 			//ADG728AddrL,  FCOM0 = MasterNACK
			cellASIC[j].com.tx_data[2] = (0b00000000 | (temp>>4));	//ICOM1 = BLANK, ADG728SelH
			cellASIC[j].com.tx_data[3] = (0b00001001 | (temp<<4));	//ADG728SelL, FCOM1 = STOP
			cellASIC[j].com.tx_data[4] = 0b01110000; 			//ICOM2 = No Transmit, 0000
			cellASIC[j].com.tx_data[5] = 0b00000000; 			//0000, FCOM2 = BLANK
			//WRCOMM command
			LTC6811_wrcomm(TOTAL_IC, cellASIC);
			delay_us(1000);
			//STCOMM command (start I2C communication), it will send stcomm+pec (4 bytes) + 3 bytes * the number sent
			LTC6811_stcomm(3*TOTAL_IC);
			delay_us(1000);
			LTC6811_rdcomm(TOTAL_IC,cellASIC);
			delay_us(1000);
		 }
		 //Wait for mux to each ASIC to stabilize, read and parse MUX
		 delay_us(5000);

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

void HAL_runBalancing()
{
	uint16_t i,j;
	bool balanceNeededFlag=false;
	float_t vmin=voltages[0][0];
	bool dccBitsSend[TOTAL_VOLTAGES];

	//STANDBY STATE
	if(balancingState==HAL_BALSTDBY)
	{
		//Find the minimum voltage HAL_readCellVoltages() has run before
		for(i=0; i<TOTAL_IC; i++)
		{
			for(j=0; j<TOTAL_VOLTAGES; j++)
			{
				if(voltages[i][j]<=vmin)
				{
					vmin=voltages[i][j];
				}
			}
		}
		//Find which of the cells satisfy threshold and write to array
		for(i=0; i<TOTAL_IC; i++)
		{
			for(j=0; j<TOTAL_VOLTAGES; j++)
			{
				if((voltages[i][j]-vmin)>HAL_BALVOLTAGETHRESHOLD)
				{
					//Do balance only existing/healthy cells
					if(voltages[i][j]>=CELL_UV)
					{
						dccBits_a[i][j]=true;
						balanceNeededFlag=true;
					}
				}
				else
				{
					dccBits_a[i][j]=false;
				}
			}
		}
		if(balanceNeededFlag==true)
		{
			balanceNeededFlag=false;
			balancingState=HAL_BALPREPARE;
		}
	}

	//PREPARE STATE
	if(balancingState==HAL_BALPREPARE)
	{
		if(batteryPackCurrent<HAL_BALCURRENTTHRESHOLD)
		{
			balancingCounter++;
			if(balancingCounter>=HAL_BALMAXSTATECNT)
			{
				balancingCounter=0;
				balancingState=HAL_BALON;
			}
		}
		else
		{
			balancingCounter=0;
			balancingState=HAL_BALSTDBY;
		}

	}

	//BALANCE ON STATE
	if(balancingState==HAL_BALON)
	{
		balancingCounter++;
		if(balancingCounter>=HAL_BALMAXBALDURATION)
		{
			balancingCounter=0;
			balancingState=HAL_BALSTDBY;
			//Turn off balancing
			for(i=0; i<TOTAL_IC; i++)
			{
				for(j=0; j<TOTAL_VOLTAGES; j++)
				{
					dccBits_a[i][j]=false;
					dccBitsSend[j]=false;
				}
				LTC6811_set_cfgr(i,cellASIC,REFON,ADCOPT,gpioBits_a,dccBitsSend, dctoBits, cellUV, cellOV);
				LTC6811_wrcfg(TOTAL_IC, cellASIC);
				delay_us(1000);
			}

		}
		//TODO: add additional conditions (like temperature)
		if(batteryPackCurrent<=HAL_BALCURRENTTHRESHOLD)
		{
			//Turn on balancing
			for(i=0; i<TOTAL_IC; i++)
			{
				for(j=0; j<TOTAL_VOLTAGES; j++)
				{
					dccBitsSend[j]=dccBits_a[i][j];
				}
				LTC6811_set_cfgr(i,cellASIC,REFON,ADCOPT,gpioBits_a,dccBitsSend, dctoBits, cellUV, cellOV);
				LTC6811_wrcfg(TOTAL_IC, cellASIC);
				delay_us(1000);
			}
		}
		else
		{
			balancingState=HAL_BALSTDBY;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3)
	{
		  //Toggle LED
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

		  //TODO:Get current
		  batteryPackCurrent=-0.51f;

		  //Wakeup
		  wakeup_idle(TOTAL_IC);

		  //Turn on voltage reference
		  LTC6811_wrcfg(TOTAL_IC, cellASIC);

		  //Voltage conversion
		  HAL_readCellVoltages();

		  //TODO: Temperature conversion
		  HAL_readCellTemperatures();

		  //Run balancing algorithm and set appropriate bits
		  HAL_runBalancing();
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
