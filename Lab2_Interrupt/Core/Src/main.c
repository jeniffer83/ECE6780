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
  // Enable the GPIOC clock in the RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // Enable the GPIOA clock in the RCC to use the user button B1 (PA0)
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	/* 2.4 — Enable and Set Priority of the EXTI Interrupt. */
  // Enable the selected EXTI interrupt by passing its defined name to the NVIC_EnableIRQ() function.
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  // Set the priority for the interrupt to 1 (high-priority) with the NVIC_SetPriority() function.
	// Change the EXTI interrupt to have priority 3. (lowest priority)
  NVIC_SetPriority(EXTI0_1_IRQn, 1);

	/* 2.5 — Writing the EXTI Interrupt Handler. */
	// Toggle both the green and orange LEDs(PC8 & PC9)in the EXTI interrupt handler.
  // Clear mode bits for PC6, 7, 8, 9
  GPIOC->MODER &= ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk |
                    GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
  // Set the mode bit to 01 for PC6, 7, 8, 9
  GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 |
                   GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
  // Config all LEDs as push-pull output type
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 |
                     GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
  // Config all LEDs to low speed
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk |
                      GPIO_OSPEEDR_OSPEEDR8_Msk | GPIO_OSPEEDR_OSPEEDR9_Msk);
  // Config all LEDs with no pull-up/down resistors
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk | GPIO_PUPDR_PUPDR7_Msk |
                    GPIO_PUPDR_PUPDR8_Msk | GPIO_PUPDR_PUPDR9_Msk);
	// Set the green LED (PC9) high
  GPIOC->BSRR = GPIO_BSRR_BS_9;
	
  // Configure the button pin (PA0) to input-mode at low-speed, with the internal pull-down resistor enabled.
  // Set PA0 to input mode
  GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk);
  // Set PA0 to low speed
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk);
  // Clear pull-up/pull-down bits for PA0
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk);
  // Set PA0 to pull-down mode
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;

	// Clear the appropriate flag for input line 0 in the EXTI pending register with in the handler.
  // Enable/unmask interrupt generation on EXTI input line 0 (EXTI0).
  EXTI->IMR = 0x0001;
  // Configure the EXTI input line 0 to have a rising-edge trigger
  EXTI->RTSR = 0x0001;
  // Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
  SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PA;
	
	/* 2.7 — Exploring Interrupt Priorities. */
	// Changes the SysTick interrupt priority to 2 (medium priority)
  NVIC_SetPriority(SysTick_IRQn, 2);
	
  while (1)
  {
    // Toggle the red LED (PC6) with a moderately-slow delay (400-600ms) in the infinite loop
    GPIOC->ODR ^= GPIO_ODR_6; // Toggle PC6
    HAL_Delay(500);           // Delay for 500 milliseconds
  }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/* 2.6 — Long-Running Interrupts. */
// Use the handler name to declare the handler function
void EXTI0_1_IRQHandler(void)
{
  // Check if the interrupt was triggered by EXTI0
  EXTI->PR |= EXTI_PR_PR0;

  GPIOC->ODR ^= (GPIO_ODR_8 | GPIO_ODR_9);

	// A delay loop of roughly 1-2 seconds to the EXTI interrupt handler.
  for (volatile uint32_t i = 0; i < 1500000; ++i)
  {
  }

  GPIOC->ODR ^= (GPIO_ODR_8 | GPIO_ODR_9);
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