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
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// CAN Message Structure
typedef struct {
    uint32_t id;        // Standard ID (11-bit)
    uint8_t dlc;        // Data Length Code (0-8)
    uint8_t data[8];    // Data bytes
} CAN_Message;

// MCP2515 Handle Structure
typedef struct {
    SPI_HandleTypeDef *hspi;    // SPI handle
    GPIO_TypeDef *cs_port;      // CS GPIO Port
    uint16_t cs_pin;            // CS GPIO Pin
} MCP2515_Handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// MCP2515 SPI Commands
#define MCP2515_RESET       0xC0
#define MCP2515_READ        0x03
#define MCP2515_WRITE       0x02
#define MCP2515_RTS_TX0     0x81
#define MCP2515_READ_STATUS 0xA0
#define MCP2515_RX_STATUS   0xB0
#define MCP2515_BIT_MODIFY  0x05

// MCP2515 Registers
#define MCP2515_RXF0SIDH    0x00
#define MCP2515_RXF0SIDL    0x01
#define MCP2515_BFPCTRL     0x0C
#define MCP2515_TXRTSCTRL   0x0D
#define MCP2515_CANSTAT     0x0E
#define MCP2515_CANCTRL     0x0F
#define MCP2515_CNF3        0x28
#define MCP2515_CNF2        0x29
#define MCP2515_CNF1        0x2A
#define MCP2515_CANINTE     0x2B
#define MCP2515_CANINTF     0x2C
#define MCP2515_TXB0CTRL    0x30
#define MCP2515_TXB0SIDH    0x31
#define MCP2515_TXB0SIDL    0x32
#define MCP2515_TXB0DLC     0x35
#define MCP2515_TXB0D0      0x36
#define MCP2515_RXB0CTRL    0x60
#define MCP2515_RXB0SIDH    0x61
#define MCP2515_RXB0SIDL    0x62
#define MCP2515_RXB0DLC     0x65
#define MCP2515_RXB0D0      0x66

// MCP2515 Configuration
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_CONFIG     0x80
#define MCP2515_INT_RX0IF       0x01
#define MCP2515_INT_TX0IF       0x04
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// MCP2515 Handles
MCP2515_Handle hcan_tx = {
    .hspi = &hspi3,
    .cs_port = GPIOA,
    .cs_pin = CS_MSP2515_TX_Pin // Note: Typo in IOC (MSP vs MCP)
};

MCP2515_Handle hcan_rx = {
    .hspi = &hspi1,
    .cs_port = GPIOB,
    .cs_pin = CS_MSP2515_RX_Pin
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void MCP2515_Select(MCP2515_Handle *hcan);
void MCP2515_Deselect(MCP2515_Handle *hcan);
void MCP2515_WriteReg(MCP2515_Handle *hcan, uint8_t reg, uint8_t value);
uint8_t MCP2515_ReadReg(MCP2515_Handle *hcan, uint8_t reg);
void MCP2515_Reset(MCP2515_Handle *hcan);
void MCP2515_SetMode(MCP2515_Handle *hcan, uint8_t mode);
void MCP2515_Init(MCP2515_Handle *hcan, uint32_t bitrate);
void MCP2515_Transmit(MCP2515_Handle *hcan, CAN_Message *msg);
uint8_t MCP2515_Receive(MCP2515_Handle *hcan, CAN_Message *msg);
void UART_Print(const char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* MCP2515 Driver Functions */
void MCP2515_Select(MCP2515_Handle *hcan) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
}

void MCP2515_Deselect(MCP2515_Handle *hcan) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

void MCP2515_WriteReg(MCP2515_Handle *hcan, uint8_t reg, uint8_t value) {
    uint8_t tx_buf[3] = {MCP2515_WRITE, reg, value};
    MCP2515_Select(hcan);
    HAL_SPI_Transmit(hcan->hspi, tx_buf, 3, HAL_MAX_DELAY);
    MCP2515_Deselect(hcan);
}

uint8_t MCP2515_ReadReg(MCP2515_Handle *hcan, uint8_t reg) {
    uint8_t tx_buf[2] = {MCP2515_READ, reg};
    uint8_t rx_buf = 0;
    MCP2515_Select(hcan);
    HAL_SPI_Transmit(hcan->hspi, tx_buf, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(hcan->hspi, &rx_buf, 1, HAL_MAX_DELAY);
    MCP2515_Deselect(hcan);
    return rx_buf;
}

void MCP2515_Reset(MCP2515_Handle *hcan) {
    uint8_t tx_buf = MCP2515_RESET;
    MCP2515_Select(hcan);
    HAL_SPI_Transmit(hcan->hspi, &tx_buf, 1, HAL_MAX_DELAY);
    MCP2515_Deselect(hcan);
    HAL_Delay(10);
}

void MCP2515_SetMode(MCP2515_Handle *hcan, uint8_t mode) {
    MCP2515_WriteReg(hcan, MCP2515_CANCTRL, mode);
    HAL_Delay(1);
    while ((MCP2515_ReadReg(hcan, MCP2515_CANSTAT) & 0xE0) != mode) {
        HAL_Delay(1);
    }
}

void MCP2515_Init(MCP2515_Handle *hcan, uint32_t bitrate) {
    MCP2515_Reset(hcan);
    MCP2515_SetMode(hcan, MCP2515_MODE_CONFIG);
    // 500 kbps @ 16 MHz MCP2515 clock
    if (bitrate == 500000) {
        MCP2515_WriteReg(hcan, MCP2515_CNF1, 0x01); // SJW=1, BRP=1
        MCP2515_WriteReg(hcan, MCP2515_CNF2, 0xB1); // BTLMODE=1, PHSEG1=6, PRSEG=1
        MCP2515_WriteReg(hcan, MCP2515_CNF3, 0x05); // PHSEG2=6
    }
    MCP2515_WriteReg(hcan, MCP2515_RXB0CTRL, 0x60); // Receive all
    MCP2515_WriteReg(hcan, MCP2515_RXB0SIDH, 0x00); // Filter ID 0
    MCP2515_WriteReg(hcan, MCP2515_RXB0SIDL, 0x00);
    MCP2515_WriteReg(hcan, MCP2515_CANINTE, 0x00); // No interrupts
    MCP2515_SetMode(hcan, MCP2515_MODE_NORMAL);
}

void MCP2515_Transmit(MCP2515_Handle *hcan, CAN_Message *msg) {
    MCP2515_WriteReg(hcan, MCP2515_TXB0SIDH, (msg->id >> 3));
    MCP2515_WriteReg(hcan, MCP2515_TXB0SIDL, ((msg->id & 0x07) << 5));
    MCP2515_WriteReg(hcan, MCP2515_TXB0DLC, msg->dlc & 0x0F);
    for (uint8_t i = 0; i < msg->dlc; i++) {
        MCP2515_WriteReg(hcan, MCP2515_TXB0D0 + i, msg->data[i]);
    }
    MCP2515_WriteReg(hcan, MCP2515_TXB0CTRL, 0x08); // Set TXREQ
}

uint8_t MCP2515_Receive(MCP2515_Handle *hcan, CAN_Message *msg) {
    uint8_t status = MCP2515_ReadReg(hcan, MCP2515_CANINTF);
    if (!(status & MCP2515_INT_RX0IF)) {
        return 0; // No message
    }
    msg->id = (MCP2515_ReadReg(hcan, MCP2515_RXB0SIDH) << 3) |
              (MCP2515_ReadReg(hcan, MCP2515_RXB0SIDL) >> 5);
    msg->dlc = MCP2515_ReadReg(hcan, MCP2515_RXB0DLC) & 0x0F;
    for (uint8_t i = 0; i < msg->dlc; i++) {
        msg->data[i] = MCP2515_ReadReg(hcan, MCP2515_RXB0D0 + i);
    }
    MCP2515_WriteReg(hcan, MCP2515_CANINTF, status & ~MCP2515_INT_RX0IF);
    return 1; // Message received
}

void UART_Print(const char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  // Initialize MCP2515 modules
  UART_Print("Initializing MCP2515 Transmitter (SPI3)...\r\n");
  MCP2515_Init(&hcan_tx, 500000);
  UART_Print("Initializing MCP2515 Receiver (SPI1)...\r\n");
  MCP2515_Init(&hcan_rx, 500000);

  // Test message
  CAN_Message tx_msg = {
      .id = 0x123,
      .dlc = 4,
      .data = {0xAA, 0xBB, 0xCC, 0xDD}
  };
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buf[64];
  while (1)
  {
    UART_Print("Sending CAN message...\r\n");
    MCP2515_Transmit(&hcan_tx, &tx_msg);
    HAL_Delay(1000);

    CAN_Message rx_msg = {0};
    if (MCP2515_Receive(&hcan_rx, &rx_msg)) {
        sprintf(buf, "Received CAN ID: 0x%03X, DLC: %d, Data: \r\n", rx_msg.id, rx_msg.dlc);
        UART_Print(buf);
        for (uint8_t i = 0; i < rx_msg.dlc; i++) {
            sprintf(buf, "%02X ", rx_msg.data[i]);
            UART_Print(buf);
        }
        UART_Print("\r\n");
    }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
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
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  /* USER CODE END SPI3_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(CS_MSP2515_TX_GPIO_Port, CS_MSP2515_TX_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_MSP2515_RX_GPIO_Port, CS_MSP2515_RX_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = CS_MSP2515_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_MSP2515_TX_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = CS_MSP2515_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_MSP2515_RX_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */
