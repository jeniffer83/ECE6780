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
#include "stdlib.h"

int32_t ReadX();
int32_t ReadY();
void _Error_Handler(char *file, int line);
void Write(volatile uint32_t addr);

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
	HAL_Init();			  // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); // Configure the system clock

	/* 5.2 Setting the GPIO Modes */
	/* This example uses HAL library calls to control
	the GPIOC peripheral. You�ll be redoing this code
	with hardware register access. */
	// Enable GPIOB and GPIOC in the RCC
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Enable I2C in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	// Initialize LED pins
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
								GPIO_MODE_OUTPUT_PP,
								GPIO_SPEED_FREQ_LOW,
								GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr);

	// Set PB11 and PB13 to alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER13_0);
	GPIOB->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1);

	// Set up PB11 and PB13 to open-drain output type
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;

	// Select I2C2_SDA as its alternate function.
	GPIOB->AFR[1] |= (1 << 12);

	// Select I2C2_SCL as its alternate function.
	GPIOB->AFR[1] |= (5 << 20);

	// Set PB14 to output mode
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER14_1);

	// Push-pull output type
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);

	// Initialize/set the pin high
	GPIOB->ODR |= GPIO_ODR_14;

	// Set PC0 to output mode
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_1);

	// Push-pull output type
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);

	// Initialize/set the pin high
	GPIOC->ODR |= GPIO_ODR_0;

	GPIOB->MODER &= ~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER15_1);

	/* 5.3 Initializing the I2C Peripheral */
	// Enable the I2C2 peripheral in the RCC.
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	// Set the parameters in the TIMINGR register to use 100kHz standard-mode I2C.
	I2C2->TIMINGR |= (1 << 28) | 0x13 | (0xF << 8) | (0x2 << 16) | (0x4 << 20);

	// Enable the I2C peripheral using the PE bit in the CR1 register.
	I2C2->CR1 |= I2C_CR1_PE;

	/* 5.4 Reading the Register */
	// Set the transaction parameters in the CR2 register.
	// I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	// Set the number of bytes to transmit=1.
	// I2C2->CR2 |= (1 << 16) | (0x69 << 1);

	// Set the RD_WRN bit to indicate a write operation.
	// I2C2->CR2 &= ~(I2C_CR2_RD_WRN);

	// Set the START bit
	// I2C2->CR2 |= I2C_CR2_START;

	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not
	// Acknowledge) flags a reset.
	// while (!(I2C2->ISR & I2C_ISR_TXIS))
	//	;
	// if (I2C2->ISR & I2C_ISR_NACKF)
	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// Write the address of the “WHO_AM_I” register into the I2C transmit register. (TXDR)
	// I2C2->TXDR |= 0x0F;
	// while (!(I2C2->ISR & I2C_ISR_TC))
	//	; /* loop waiting for TC */
	// Reload the CR2 register with the same parameters as before
	// I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// I2C2->CR2 |= (1 << 16) | (0x69 << 1);

	// Set the RD_WRN bit to indicate a read operation
	// I2C2->CR2 |= I2C_CR2_RD_WRN;

	// Set the START bit again to perform a I2C restart condition
	// I2C2->CR2 |= I2C_CR2_START;

	// Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not
	// Acknowledge) flags a reset
	// while (!(I2C2->ISR & I2C_ISR_RXNE))
	//	;
	// Wait until the TC (Transfer Complete) flag is set.
	// if (I2C2->ISR & I2C_ISR_NACKF)
	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// while (!(I2C2->ISR & I2C_ISR_TC))
	//	; /* loop waiting for TC */

	// Check the contents of the RXDR register to see if it matches 0xD3. (expected value of the
	// “WHO_AM_I” register)
	// if (I2C2->RXDR == 0xD3)
	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	// Set the STOP bit in the CR2 register to release the I2C bus
	// I2C2->CR2 |= I2C_CR2_STOP;

	/* 5.5 Initializing the Gyroscope */
	// Enable the X and Y sensing axes in the CTRL_REG1 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	// Set the sensor in to “normal or sleep mode” using the PD bit in the CTRL_REG1 register
	I2C2->CR2 |= (2 << 16) | (0x69 << 1);

	// All other bits in the CTRL_REG1 register should be set to 0. These place the device in the
	// default low-speed mode
	I2C2->CR2 |= I2C_CR2_START;

	/* 5.6 Exercise Specifications */
	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not
	// Acknowledge) flags a reset.
	while (!(I2C2->ISR & I2C_ISR_TXIS))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Error State
	// Write CTRL_REG1 into I2C transmit register
	I2C2->TXDR |= 0x20;

	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not
	// Acknowledge) flags a reset.
	while (!(I2C2->ISR & I2C_ISR_TXIS))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// Write the address of the “WHO_AM_I” register into the I2C transmit register. (TXDR)
	I2C2->TXDR |= 0xB;
	while (!(I2C2->ISR & I2C_ISR_TC))
		; /* loop waiting for TC */

	// Initialize the L3GD20 gyroscope sensor to read the X and Y axes
	int x_axes, y_axes;
	while (1)
	{
		x_axes = ReadX(); // Read and write
		y_axes = ReadY(); // Read and write

		int16_t threshold = 0x01FF; // This will turn LED on

		if (abs(x_axes) > threshold | abs(y_axes) > threshold)
		{
			if (abs(x_axes) > abs(y_axes))
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (x_axes > threshold) ? GPIO_PIN_SET : GPIO_PIN_RESET);	 // Positive X
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (x_axes < -threshold) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Negative X
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (y_axes > threshold) ? GPIO_PIN_SET : GPIO_PIN_RESET);	 // Positive Y
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (y_axes < -threshold) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Negative Y
			}
		}

		HAL_Delay(100);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

void Write(volatile uint32_t addr)
{
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	// Set the number of bytes to transmit=1.
	I2C2->CR2 |= (1 << 16) | (0x69 << 1);

	// Set the RD_WRN bit to indicate a write operation
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN);

	// Set the START bit
	I2C2->CR2 |= I2C_CR2_START;

	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not
	// Acknowledge) flags a reset.
	while (!(I2C2->ISR & I2C_ISR_TXIS))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// write CTRL_REG1 into I2C transmit register
	I2C2->TXDR |= addr;
	while (!(I2C2->ISR & I2C_ISR_TC))
		; /* loop waiting for TC */
		  // Set the STOP bit in the CR2 register to release the I2C bus
	I2C2->CR2 |= I2C_CR2_STOP;
}

int32_t ReadX()
{
	int16_t x_axis;
	Write(0xA8);

	// Enable the X and Y sensing axes in the CTRL_REG1 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	// Set the sensor in to “normal or sleep mode” using the PD bit in the CTRL_REG1 register
	I2C2->CR2 |= (2 << 16) | (0x69 << 1);

	// Set the RD_WRN bit to indicate a read operation// reset RD_WRN to read
	I2C2->CR2 |= I2C_CR2_RD_WRN;

	// Set the START bit
	I2C2->CR2 |= I2C_CR2_START;

	// Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not
	// Acknowledge) flags a reset
	while (!(I2C2->ISR & I2C_ISR_RXNE))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	x_axis = I2C2->RXDR;
	while (!(I2C2->ISR & I2C_ISR_RXNE))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	x_axis |= (I2C2->RXDR << 8);
	while (!(I2C2->ISR & I2C_ISR_TC))
		; /* loop waiting for TC */
	return x_axis;
}

int32_t ReadY()
{
	int16_t y_axis;
	Write(0xAA);

	// Reload the CR2 register with the same parameters as before
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	// Set the sensor in to “normal or sleep mode” using the PD bit in the CTRL_REG1 register
	I2C2->CR2 |= (2 << 16) | (0x69 << 1);

	// Set the RD_WRN bit to indicate a write operation.
	I2C2->CR2 |= I2C_CR2_RD_WRN;

	// Set the START bit again to perform a I2C restart condition
	I2C2->CR2 |= I2C_CR2_START;

	// Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not
	// Acknowledge) flags a reset
	while (!(I2C2->ISR & I2C_ISR_RXNE))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	y_axis = I2C2->RXDR;
	while (!(I2C2->ISR & I2C_ISR_RXNE))
		;
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	y_axis |= (I2C2->RXDR << 8);
	while (!(I2C2->ISR & I2C_ISR_TC))
		; /* loop waiting for TC */
	return y_axis;
}

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
