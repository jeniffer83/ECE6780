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
#include "stm32f072xb.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  
	/* 3.1 — Using Timer Interrupts. */
	// Enable the timer 2 peripheral (TIM2) in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Enable GPIOC in the RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // Configure the timer to trigger an update event (UEV) at 4 Hz
  TIM2->PSC = 7999;
  TIM2->ARR = 250;
  // Configure the timer to generate an interrupt on the UEV event.
  TIM2->DIER = 0x1;
  // Configure and enable/start the timer
  TIM2->CR1 |= TIM_CR1_CEN;
  // Set up the timer’s interrupt handler, and enable in the NVIC.
  NVIC_EnableIRQ(TIM2_IRQn);
	
	/* Toggle between the green (PC8) and orange (PC9) LEDs in the interrupt handler. */
  // Reset the mode of PC8, PC9 to input
  GPIOC->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
  // Configure PC8, PC9 as an output
  GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
  // Set the output type of PC8, PC9 to push-pull
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
  // Set the output speed of PC8, PC9 to high
  GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR9;
  // Set the pull-up/down of PC8, PC9 to none
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
	
	// Set PC9 to high
	GPIOC->ODR |= GPIO_ODR_9;
	
	/* 3.2 — Configuring Timer Channels to PWM Mode. */
	// Enable the timer 3 peripheral (TIM3) in the RCC.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Configure the timer to trigger an update event (UEV) at 800 Hz
	TIM3->PSC = 79;
  TIM3->ARR = 125;
	// Configure and enable/start the timer
  TIM3->CR1 |= TIM_CR1_CEN;
	// Set CC1S to 00 as CC1 channel is configured as output
  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S_0;
  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S_1;
  // Set CC2S to 00 as CC2 channel is configured as output
  TIM3->CCMR1 &= ~TIM_CCMR1_CC2S_0;
  TIM3->CCMR1 &= ~TIM_CCMR1_CC2S_1;
  // Set output channel 1 to PWM Mode 2 ==> 111
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
  // Use the OC2M[2:0] bit field to set channel 2 to PWM Mode 1 ==> 110
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
  TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_0;
  // Enable the output compare preload for both channels
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
  // Set the output enable bits for channels 1 & 2 in the CCER register.
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  // Set the capture/compare registers (CCRx) for both channels to 20% of your ARR value.
  TIM3->CCR1 = 25; // 125 * 20
  TIM3->CCR2 = 25; // 125 * 20
	
	/* 3.3 — Configuring Pin Alternate Functions. */
	// Set up PC6 and PC7 as alternate function mode 10
  GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
  GPIOC->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
  // Configure all LEDs as push-pull output type
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 );
  // Configure all LEDs to low speed
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk);
  // Configure all LEDs with no pull-up/down resistors
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk);

  while (1)
  {

  }
}

void TIM2_IRQHandler(void)
{
  // Clear the interrupt pending bit
  TIM2->SR &= ~TIM_SR_UIF;
	
	GPIOC->ODR ^=(GPIO_ODR_8 | GPIO_ODR_9);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  //__disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
