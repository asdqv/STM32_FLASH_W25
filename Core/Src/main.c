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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define w25_ENABLE_RESET	0x66
#define w25_RESET			0x99
#define w25_READ			0x03
#define w25_GET_JDEC_ID		0x9f
//макросы для управления ножкой выбора NSS
#define cs_set()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET) //chip select, если ножка в низом уровне, отклик ведомого устройства разрешен, если в высоком, то запрещен
#define cs_reset()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct
{
  uint16_t  PageSize;
  uint32_t  PageCount;
  uint32_t  SectorSize;
  uint32_t  SectorCount;
  uint32_t  BlockSize;
  uint32_t  BlockCount;
  uint32_t  NumKB;
  uint8_t   SR1;
  uint8_t   SR2;
  uint8_t   SR3;
}w25_info_t;

w25_info_t  w25_info;

uint8_t rx_buf[1025];
uint8_t tx_buf[10];
char str[130];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SPI1_Send(uint8_t *dt, uint16_t cnt){
	HAL_SPI_Transmit(&hspi1, dt, cnt, 5000);
}
void SPI1_Recv(uint8_t *dt, uint16_t cnt){
	HAL_SPI_Receive(&hspi1, dt, cnt, 5000);
}
void w25_Reset(void){
	cs_set();
	tx_buf[0] = w25_ENABLE_RESET;
	tx_buf[1] = w25_RESET;
	SPI1_Send(tx_buf, 2);
	cs_reset();
}
//func для начала чтения из flash-ки записываем команду 03 и 24 битный аддрес
void w25_Read_Data(uint32_t addr, uint8_t *dat, uint32_t sz){
	cs_set();
	tx_buf[0] = w25_READ;
	tx_buf[1] = (addr >> 16) & 0xFF;
	tx_buf[2] = (addr >> 8) & 0xFF;
	tx_buf[3] = addr & 0xFF;
	SPI1_Send(tx_buf, 4);
	SPI1_Recv(rx_buf, sz);
	cs_reset();
}
uint32_t w25_Read_ID(void){
	uint8_t dt[3];
	tx_buf[0] = w25_GET_JDEC_ID;
	cs_set();
	SPI1_Send(tx_buf, 1);
	SPI1_Recv(dt, 3);
	cs_reset();
	return (dt[0] << 16 | dt[1] << 8) | dt[2];
}
void w25_Init(void){
	HAL_Delay(100);
	w25_Reset();
	HAL_Delay(100);
	unsigned int ID = w25_Read_ID();
	HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
	 sprintf(str,"ID:0x%X\r\n", ID);
	 HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	 ID &= 0x0000ffff; //маска отсечения manuf ID
	 switch(ID)
	  {
	    case 0x401A:
	      w25_info.BlockCount=1024;
	      sprintf(str,"w25qxx Chip: w25q512\r\n");
	      break;
	    case 0x4019:
	      w25_info.BlockCount=512;
	      sprintf(str,"w25qxx Chip: w25q256\r\n");
	      break;
	    case 0x4018:
	      w25_info.BlockCount=256;
	      sprintf(str,"w25qxx Chip: w25q128\r\n");
	      break;
	    case 0x4017:
	      w25_info.BlockCount=128;
	      sprintf(str,"w25qxx Chip: w25q64\r\n");
	      break;
	    case 0x4016:
	      w25_info.BlockCount=64;
	      sprintf(str,"w25qxx Chip: w25q32\r\n");
	      break;
	    case 0x4015:
	      w25_info.BlockCount=32;
	      sprintf(str,"w25qxx Chip: w25q16\r\n");
	      break;
	    case 0x4014:
	      w25_info.BlockCount=16;
	      sprintf(str,"w25qxx Chip: w25q80\r\n");
	      break;
	    case 0x4013:
	      w25_info.BlockCount=8;
	      sprintf(str,"w25qxx Chip: w25q40\r\n");
	      break;
	    case 0x4012:
	      w25_info.BlockCount=4;
	      sprintf(str,"w25qxx Chip: w25q20\r\n");
	      break;
	    case 0x4011:
	      w25_info.BlockCount=2;
	      sprintf(str,"w25qxx Chip: w25q10\r\n");
	      break;
	    default:
	      sprintf(str,"w25qxx Unknown ID\r\n");
	      HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	      return;
	  }
	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   w25_info.PageSize=256;
	   w25_info.SectorSize=0x1000;
	   w25_info.SectorCount=w25_info.BlockCount*16;
	   w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
	   w25_info.BlockSize=w25_info.SectorSize*16;
	   w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
	   sprintf(str,"Page Size: %d Bytes\r\n",(unsigned int)w25_info.PageSize);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Page Count: %u\r\n",(unsigned int)w25_info.PageCount);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Sector Size: %u Bytes\r\n",(unsigned int)w25_info.SectorSize);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Sector Count: %u\r\n",(unsigned int)w25_info.SectorCount);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Block Size: %u Bytes\r\n",(unsigned int)w25_info.BlockSize);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Block Count: %u\r\n",(unsigned int)w25_info.BlockCount);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Capacity: %u KB\r\n",(unsigned int)w25_info.NumKB);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  w25_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
