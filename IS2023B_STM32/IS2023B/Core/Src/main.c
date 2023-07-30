/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "USART_HMI.h"
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

/* USER CODE BEGIN PV */
static uint16_t total_steps = 50;
static uint16_t steps = 15;
static uint16_t P_skip_num = 0;
static float MICROPHONE[4][2] = { {HALF_SQUARE, HALF_SQUARE}, 
                                  {-HALF_SQUARE, HALF_SQUARE}, 
                                  {-HALF_SQUARE, -HALF_SQUARE}, 
                                  {HALF_SQUARE, -HALF_SQUARE}};
static float POINT_DIST[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float CORRECT_POINT_DIST[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float G_VECTOR[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
bool volatile first_interrupt = true;
uint8_t volatile interrupt_times = 0;
uint32_t quadrant_time_stamp[5] = {0, 0, 0, 0, 0};
uint8_t quadrant_time_order[5] = {0, 0, 0, 0, 0};
static float pix, piy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Gradient_descent_wrapper(void);
static void Quadrant_Lattice_Indexing(void);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  UARTHMI_Forget_It();
  UARTHMI_Reset();
  HAL_Delay(150);
  Configuration_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (interrupt_times == 4)
    {
      __NVIC_DisableIRQ(TIM2_IRQn);
      Quadrant_Lattice_Indexing();
      __NVIC_EnableIRQ(TIM2_IRQn);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
static inline float dist(float p1x, float p1y, float p2x, float p2y)
{
  return sqrtf((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y));
}

static inline float norm(float px, float py)
{
  return sqrtf(px * px + py * py);
}

static void Gradient_descent(uint8_t step)
{
  float sum = 0.0f, temp = 0.0f;
  float gradients_list[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
  for (uint8_t i = 0; i < 4; ++i)
  {
    POINT_DIST[i] = (dist(pix, piy, MICROPHONE[i][0], MICROPHONE[i][1]) - dist(pix, piy, MICROPHONE[P_skip_num][0], MICROPHONE[P_skip_num][1])) - quadrant_time_stamp[i] / CLK_FREQ * V_VOICE;
    sum += fabsf(POINT_DIST[i]);
  }
  for (uint8_t i = 0; i < 4; ++i)
  {
    POINT_DIST[i] /= sum;
    temp = (steps + sum / 20.0f) * (1.0f - logf(1.0f + exp(step / 5.0f - 10.0f)) / 0.65f);
    gradients_list[i][0] = G_VECTOR[i][0] * POINT_DIST[i] * temp;
    gradients_list[i][1] = G_VECTOR[i][1] * POINT_DIST[i] * temp;
    pix += gradients_list[i][0];
    piy += gradients_list[i][1];
  }
  
}

static void Gradient_descent_wrapper(void)
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    G_VECTOR[i][0] = MICROPHONE[i][0] - MICROPHONE[P_skip_num][0];
    G_VECTOR[i][1] = MICROPHONE[i][1] - MICROPHONE[P_skip_num][1];
    if (i != P_skip_num)
    {
      G_VECTOR[i][0] /= norm(G_VECTOR[i][0], G_VECTOR[i][1]) * sqrtf(3.0f);
      G_VECTOR[i][1] /= norm(G_VECTOR[i][0], G_VECTOR[i][1]) * sqrtf(3.0f);
    }
  }
  for (uint8_t i = 0; i < total_steps; ++i)
  {
    Gradient_descent(i);
  }
}

static void Quadrant_Lattice_Indexing(void)
{
  int8_t x_index, y_index;
  for (uint8_t i = 0; i < 4; ++i)
  {
    if (quadrant_time_order[i + 1] == 1)
    {
      P_skip_num = i;
      pix = MICROPHONE[i][0] * 5.0f / UARTHMI_LATTICE;
      piy = MICROPHONE[i][1] * 5.0f / UARTHMI_LATTICE;
      break;
    }
  }
  UARTHMI_Send_Number(0, P_skip_num + 1);
  Gradient_descent_wrapper();
  x_index = (int8_t)(fabsf(pix) / UNIT) + 1;
  y_index = (int8_t)(fabsf(piy) / UNIT) + 1;
  if (pix < 0.0f)
  {
    x_index = -x_index;
  }
  if (piy < 0.0f)
  {
    y_index = -y_index;
  }
  UARTHMI_Send_Number(1, x_index);
  UARTHMI_Send_Number(2, y_index);
  printf("page 0\xff\xff\xff");
  printf("fill %d,%d,%d,%d,RED\xff\xff\xff", (uint16_t)((pix + 250.0f) / 500.0f * 240.0f) / (UARTHMI_LATTICE * LATTICE_RATIO) * (UARTHMI_LATTICE * LATTICE_RATIO), (uint16_t)((250.0f - piy) / 500.0f * 240.0f) / (UARTHMI_LATTICE * LATTICE_RATIO) * (UARTHMI_LATTICE * LATTICE_RATIO), LATTICE_RATIO * UARTHMI_LATTICE, LATTICE_RATIO * UARTHMI_LATTICE);
}

void Configuration_Init(void)
{
  HAL_TIM_Base_Stop(&htim2);
  __NVIC_EnableIRQ(EXTI0_IRQn);
  __NVIC_EnableIRQ(EXTI1_IRQn);
  __NVIC_EnableIRQ(EXTI2_IRQn);
  __NVIC_EnableIRQ(EXTI3_IRQn);
  memset(quadrant_time_stamp, 0x00000000, sizeof(uint32_t) * 5);
  memset(quadrant_time_order, 0x00, 5);
  interrupt_times = 0;
  first_interrupt = true;
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
    printf("Error_Handler\r\n");
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
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
