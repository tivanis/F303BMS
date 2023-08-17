/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LTC6811.h"
#include "LTC681x.h"
#include "delay.h"
#include "fmath.h"

#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
static void HAL_readCellVoltages();

static void HAL_readCellTemperatures();

static void HAL_delay_us();

static void HAL_runSOC();

void HAL_runContactorControl();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CONTACTOR_Pin GPIO_PIN_13
#define CONTACTOR_GPIO_Port GPIOC
#define LEDR_Pin GPIO_PIN_6
#define LEDR_GPIO_Port GPIOC
#define LEDG_Pin GPIO_PIN_7
#define LEDG_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

#define  float32_t			float

//#define fabsf(x) (((x)<0) ? -(x) : (x))

#define TOTAL_IC			(2U)
#define TOTAL_VOLTAGES		(12U)
#define TOTAL_TEMPERATURES	(8U)
#define CELL_UV				(2.75f)
#define CELL_OV				(3.55f)
#define CELL_NONZEROV		(0.50f)
#define ADG_728_ADDRESS		(0b1001100)
#define HAL_BALVOLTAGETHRESHOLD (0.05f)
#define HAL_BALCURRENTTHRESHOLD (-0.5f)
#define HAL_BALMAXSTATECNT	(10U)
#define HAL_BALMAXBALDURATION (60U)

typedef enum
{
	HAL_BALSTDBY=0,
	HAL_BALPREPARE=1,
	HAL_BALMSR=2,
	HAL_BALON=3

} HAL_BALState_e;

typedef enum
{
	HAL_GLOBALSTDBY=0,
	HAL_GLOBALCONTACTORON=1,
	HAL_GLOBALCONTACTOROFF=2,
	HAL_GLOBALCELLUV=3,
	HAL_GLOBALCELLOV=4

} HAL_GlobalState_e;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
