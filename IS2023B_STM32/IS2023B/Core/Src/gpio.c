/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
static uint32_t time_temp;
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = Q1_Pin|Q2_Pin|Q3_Pin|Q4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (first_interrupt)
  {
    htim2.Instance->CNT = 0;
    htim2.State = HAL_TIM_STATE_BUSY;
    htim2.Instance->DIER |= (0x1UL << (0U));
    htim2.Instance->CR1 |= (0x1UL << (0U));
    switch (GPIO_Pin)
    {
    case Q1_Pin:
      quadrant_time_stamp[Q1] = 0;
      quadrant_time_order[Q1] = 1;
      __NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    case Q2_Pin:
      quadrant_time_stamp[Q2] = 0;
      quadrant_time_order[Q2] = 1;
      __NVIC_DisableIRQ(EXTI1_IRQn);
      break;
    case Q3_Pin:
      quadrant_time_stamp[Q3] = 0;
      quadrant_time_order[Q3] = 1;
      __NVIC_DisableIRQ(EXTI2_IRQn);
      break;
    case Q4_Pin:
      quadrant_time_stamp[Q4] = 0;
      quadrant_time_order[Q4] = 1;
      __NVIC_DisableIRQ(EXTI3_IRQn);
      break;
    default:
      break;
    }
  }
  else
  {
    time_temp = htim2.Instance->CNT;
    switch (GPIO_Pin)
    {
    case Q1_Pin:
      quadrant_time_stamp[Q1] = time_temp;
      quadrant_time_order[Q1] = interrupt_times;
      __NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    case Q2_Pin:
      quadrant_time_stamp[Q2] = time_temp;
      quadrant_time_order[Q2] = interrupt_times;
      __NVIC_DisableIRQ(EXTI1_IRQn);
      break;
    case Q3_Pin:
      quadrant_time_stamp[Q3] = time_temp;
      quadrant_time_order[Q3] = interrupt_times;
      __NVIC_DisableIRQ(EXTI2_IRQn);
      break;
    case Q4_Pin:
      quadrant_time_stamp[Q4] = time_temp;
      quadrant_time_order[Q4] = interrupt_times;
      __NVIC_DisableIRQ(EXTI3_IRQn);
      break;
    default:
      break;
    }
  }
  ++interrupt_times;
}
/* USER CODE END 2 */
