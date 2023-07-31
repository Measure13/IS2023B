/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define Q1 0
#define Q2 1
#define Q3 2
#define Q4 3
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define ADC_DATA_LENGTH 1024
#define ADC_SAMPLE_RATE 1000000

extern bool volatile conv_done;
extern uint16_t adc_values[ADC_DATA_LENGTH + 4];
extern bool volatile first_interrupt;
extern uint8_t volatile interrupt_times;
extern uint32_t quadrant_time_stamp[4];
extern bool volatile overtime;
void Configuration_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Q1_Pin GPIO_PIN_0
#define Q1_GPIO_Port GPIOA
#define Q1_EXTI_IRQn EXTI0_IRQn
#define Q2_Pin GPIO_PIN_1
#define Q2_GPIO_Port GPIOA
#define Q2_EXTI_IRQn EXTI1_IRQn
#define Q3_Pin GPIO_PIN_2
#define Q3_GPIO_Port GPIOA
#define Q3_EXTI_IRQn EXTI2_IRQn
#define Q4_Pin GPIO_PIN_3
#define Q4_GPIO_Port GPIOA
#define Q4_EXTI_IRQn EXTI3_IRQn
#define BEEP_Pin GPIO_PIN_4
#define BEEP_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define ALARM HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)
#define NO_BB HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)
#define REF_WEIGHT 0.75f
#define LENGTH 500
#define WIDTH 500
#define HALF_SQUARE (LENGTH / 2 + 50.0f)
#define UNIT 50
#define UARTHMI_LATTICE 12
#define UARTHMI_SQUARE 25
#define LATTICE_RATIO (UNIT / UARTHMI_SQUARE)
#define V_VOICE 340.0f
#define CLK_FREQ  128000000.0f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
