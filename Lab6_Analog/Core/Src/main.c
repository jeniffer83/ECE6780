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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void _Error_Handler(char *file, int line);

/* USER CODE BEGIN PFP */
// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127, 151, 175, 197, 216, 232, 244, 251, 254, 251, 244,
								232, 216, 197, 175, 151, 127, 102, 78, 56, 37, 21, 9, 2, 0, 2, 9, 21, 37, 56, 78, 102};
// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0, 15, 31, 47, 63, 79, 95, 111, 127, 142, 158, 174,
									190, 206, 222, 238, 254, 238, 222, 206, 190, 174, 158, 142, 127, 111, 95, 79, 63, 47, 31, 15};
// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0, 7, 15, 23, 31, 39, 47, 55, 63, 71, 79, 87, 95, 103,
									111, 119, 127, 134, 142, 150, 158, 166, 174, 182, 190, 198, 206, 214, 222, 230, 238, 246};
// Square Wave: 8-bit, 32 samples/cycle
const uint8_t square_table[32] = {254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
								  254, 254, 254, 254, 254, 254, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
	HAL_Init();			  // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); // Configure the system clock

	/*  6.1 Measuring a Potentiometer With the ADC */
	/* This example uses HAL library calls to control
	the GPIOC peripheral. You�ll be redoing this code
	with hardware register access. */
	// Enable GPIOB and GPIOC in the RCC
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	// Enable the ADC1 in the RCC peripheral
	__HAL_RCC_ADC1_CLK_ENABLE();

	// Enable DAC in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	// Initialize the LED pins to output
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
								GPIO_MODE_OUTPUT_PP,
								GPIO_SPEED_FREQ_LOW,
								GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr);

	// Select a GPIO pin to use as the ADC input
	GPIOC->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0;
	// Configure the pin to analog mode, no pull-up/down resistors
	GPIOC->PUPDR &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0);

	// Configure the ADC to 8-bit resolution
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	ADC1->CFGR1 &= ~(ADC_CFGR1_RES_0);
	// Continuous conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	// Hardware triggers disabled (software trigger only)
	ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTEN_1);
	// Select/enable the input pin’s channel for ADC conversion
	ADC1->CHSELR = ADC_CHSELR_CHSEL10;

	// Perform a self-calibration, enable, and start the ADC
	// Use sections 13.4.1 (Calibration) and 13.4.2 (ADC on-off control) in the
	// peripheral reference manual.
	/* A.7.1 ADC Calibration code example
			(1) Ensure that ADEN=0
			(2) Clear ADEN by setting ADDIS
			(3) Clear DMAEN
			(4) Launch the calibration by setting ADCAL
			(5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		;
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL;		 /* (4) */
	// Wait until ADCAL = 0
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
		;
	}

	ADC1->CR |= ADC_CR_ADEN;

	/* In the main application loop, read the ADC data register and
	turn on/off LEDs depending on the value.
			- Use four increasing threshold values, each LED should have a
	minimum ADC value/voltage to turn on.
			- As the voltage on the input pin increases, the LEDs should light
	one-by-one.
			- If the pin voltage decreases below the threshold for a LED, it
	should turn off*/
	// Threshold from 0-255
	// int thres = 0;
	// int thres1 = 64;
	// int thres2 = 128;
	// int thres3 = 255;

	/* 6.2 Generating Waveforms with the DAC */
	// Select a GPIO pin to use as the DAC output
	// The “DAC_OUTx” additional/analog function indicates which DAC
	// output channel the pin connects to.
	// Configure the pin to analog mode
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0;
	// No pull-up/down resistors
	GPIOA->PUPDR &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0);

	// Set the used DAC channel to software trigger mode
	// Enable the used DAC channel
	DAC1->CR |= DAC_CR_TEN1;
	DAC1->CR |= DAC_CR_EN1;
	DAC->CR |= (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0);

	while (1)
	{
		/*  6.1 Measuring a Potentiometer With the ADC */
		// ADC1->CR = ADC_CR_ADSTART;
		// while (!(ADC1->ISR & ADC_ISR_EOC))
		// {
		// 	;
		// }
		// if (ADC1->DR == thres)
		// {
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		// }
		// else if (ADC1->DR > thres && ADC1->DR < thres1)
		// {
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		// }
		// else if (ADC1->DR > thres1 && ADC1->DR < thres2)
		// {
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		// }
		// else if (ADC1->DR > thres2 && ADC1->DR < thres3)
		// {
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		// }
		// else if (ADC1->DR == thres3)
		// {
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		// 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		// }

		/* 6.2 Generating Waveforms with the DAC */
		// Use an index variable to write the next value in the wave-table
		// (array) to the appropriate DAC data register
		for (int i = 0; i < 32; i++)
		{
			// The wave tables are 32-element arrays of unsigned 8-bit values
			DAC1->SWTRIGR = DAC_SWTRIGR_SWTRIG1;
			// DAC1->DHR8R1 = sine_table[i];
			// DAC1->DHR8R1 = triangle_table[i];
			DAC1->DHR8R1 = sawtooth_table[i];
			// DAC1->DHR8R1 = square_table[i];

			// Use a 1ms delay between updating the DAC to new values
			HAL_Delay(1);
		}
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
