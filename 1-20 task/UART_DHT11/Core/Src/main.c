/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // for printf functions
#include <string.h> // for strlen functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// DHT 11 timing definitions (in microseconds)
#define DHT11_RESPONSE_TIMEOUT_US 100 // Max time to wait for sensor response
#define DHT11_BIT_READ_TIMEOUT_US 100 // Max time to wait for a bit to be transmitted
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RHR_Byte1, RHR_Byte2, TEMP_Byte1, TEMP_Byte2, Sum_Check_Byte; // Raw data from DHT11
uint8_t CheckSum; // Calculated checksum

volatile float Temperature = 0.0f; // Stores temperature in Celsius
volatile float Humidity = 0.0f;    // Stores humidity in percentage

// Flag to indicate if data was successfully read
volatile uint8_t DHT11_Data_OK = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Set_DHT11_GPIO_Output(void);
void Set_DHT11_GPIO_Input(void);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read_Byte(void);
void DHT11_Read_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf to USART2
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// Function to set DHT11 Data pin as Output
void Set_DHT11_GPIO_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_DATA_Pin; // Assuming DHT11_DATA_Pin is defined for PA8
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed is fine for output
    HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_InitStruct);
}

// Function to set DHT11 Data pin as Input
void Set_DHT11_GPIO_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // Pull-up should be external 10k resistor
    HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_InitStruct);
}

// Send start signal to DHT11
void DHT11_Start(void)
{
    Set_DHT11_GPIO_Output(); // Set pin as output
    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_RESET); // Pull low
    HAL_Delay(18); // Wait for at least 18ms (DHT11 datasheet min 18ms)
    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET);  // Pull high
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset timer
    while (__HAL_TIM_GET_COUNTER(&htim2) < 20); // Wait 20-40us (we'll use 20us here)
    Set_DHT11_GPIO_Input(); // Set pin as input to listen for sensor response
}

// Check DHT11 response signal
uint8_t DHT11_Check_Response(void)
{
    uint32_t Timeout_Start = __HAL_TIM_GET_COUNTER(&htim2); // Start timeout for response

    // Wait for DHT11 to pull low (80us approx)
    while (HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_RESET)
    {
        if ((__HAL_TIM_GET_COUNTER(&htim2) - Timeout_Start) > DHT11_RESPONSE_TIMEOUT_US) return 0; // Timeout
    }

    Timeout_Start = __HAL_TIM_GET_COUNTER(&htim2); // Reset timeout for next phase
    // Wait for DHT11 to pull high (80us approx)
    while (HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_SET)
    {
        if ((__HAL_TIM_GET_COUNTER(&htim2) - Timeout_Start) > DHT11_RESPONSE_TIMEOUT_US) return 0; // Timeout
    }
    return 1; // Response received
}

// Read a single byte (8 bits) from DHT11
uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++)
    {
        uint32_t Timeout_Start = __HAL_TIM_GET_COUNTER(&htim2);
        // Wait for low pulse (50us approx)
        while (HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_RESET)
        {
            if ((__HAL_TIM_GET_COUNTER(&htim2) - Timeout_Start) > DHT11_BIT_READ_TIMEOUT_US) return 0; // Timeout
        }

        Timeout_Start = __HAL_TIM_GET_COUNTER(&htim2); // Start timer to measure high pulse duration
        // Wait for high pulse (26-28us for '0', 70us for '1')
        while (HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_SET)
        {
            if ((__HAL_TIM_GET_COUNTER(&htim2) - Timeout_Start) > DHT11_BIT_READ_TIMEOUT_US) return 0; // Timeout
        }

        // Determine if bit is 0 or 1 based on high pulse duration
        data <<= 1; // Shift left to make space for the new bit
        if ((__HAL_TIM_GET_COUNTER(&htim2) - Timeout_Start) > 40) // If high pulse > 40us, it's a '1'
        {
            data |= 1; // Set the last bit to 1
        }
    }
    return data;
}

// Read all 40 bits of data from DHT11
void DHT11_Read_Data(void)
{
    DHT11_Data_OK = 0; // Reset success flag

    // Start communication sequence
    DHT11_Start();

    // Check for sensor response
    if (DHT11_Check_Response())
    {
        // Read 5 bytes of data
        RHR_Byte1 = DHT11_Read_Byte();  // Humidity integer part
        RHR_Byte2 = DHT11_Read_Byte();  // Humidity decimal part (often 0 for DHT11)
        TEMP_Byte1 = DHT11_Read_Byte(); // Temperature integer part
        TEMP_Byte2 = DHT11_Read_Byte(); // Temperature decimal part (often 0 for DHT11)
        Sum_Check_Byte = DHT11_Read_Byte(); // Checksum byte

        printf("Raw: H1=%d, H2=%d, T1=%d, T2=%d, Sum=%d, Check=%d\r\n", RHR_Byte1, RHR_Byte2, TEMP_Byte1, TEMP_Byte2, Sum_Check_Byte, CheckSum);
        // Calculate checksum
        CheckSum = RHR_Byte1 + RHR_Byte2 + TEMP_Byte1 + TEMP_Byte2;

        // Verify checksum
        if (CheckSum == Sum_Check_Byte)
        {
            // Data is valid!
            Humidity = RHR_Byte1 + (float)RHR_Byte2 / 10.0f; // DHT11 typically only integer for DHT11
            Temperature = TEMP_Byte1 + (float)TEMP_Byte2 / 10.0f; // DHT11 typically only integer for DHT11

            // Handle negative temperature for DHT11 (bit 7 of TEMP_Byte1 is 1)
            // For DHT11, Temp_Byte1 is usually the integer part, TEMP_Byte2 is decimal (often 0)
            // If the highest bit (bit 7) of TEMP_Byte1 is 1, it indicates negative temp.
            // The actual temp is (TEMP_Byte1 & 0x7F).
            // This is specific to DHT11. DHT22 has 16-bit signed temp.
            if (TEMP_Byte1 & 0x80) // Check if the MSB is set for negative temperature
            {
                 Temperature = -((TEMP_Byte1 & 0x7F) + (float)TEMP_Byte2 / 10.0f);
            }
            else
            {
                Temperature = TEMP_Byte1 + (float)TEMP_Byte2 / 10.0f;
            }

            DHT11_Data_OK = 1; // Set success flag
        }
    }
}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Start the TIM2 base counter. This is essential for the timer to actually count!
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	DHT11_Read_Data(); // Perform the DHT11 reading

	if (DHT11_Data_OK)
	{
		// Print the temperature and humidity to the serial monitor
		printf("Temperature: %.1f C, Humidity: %.1f %%\r\n", Temperature, Humidity);

	}
	else
	{
		printf("DHT11 Error: No response or Checksum mismatch!\r\n");
	}

	HAL_Delay(2000); // Wait 2 seconds before next reading (DHT11 min 1-2 seconds between reads)


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DHT11_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_DATA_Pin */
  GPIO_InitStruct.Pin = DHT11_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
