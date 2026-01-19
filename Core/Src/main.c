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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "Modbus.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ID_mine 1

#define ACTION OutputRegisters[0] // регистр операции
#define Num1_REGISTER 1
#define Num2_REGISTER 3
#define RESULT_REGISTER 5

#define answer_H OutputRegisters[RESULT_REGISTER+1] // 32 битная display data для ответа
#define answer_L OutputRegisters[RESULT_REGISTER]
#define num1_H OutputRegisters[Num1_REGISTER+1] // для числа 1
#define num1_L OutputRegisters[Num1_REGISTER]
#define num2_H OutputRegisters[Num2_REGISTER+1] // для числа 2
#define num2_L OutputRegisters[Num2_REGISTER]

#define ERROR OutputCoils[0] // для плашки ERROR, если операция недопустима

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t TxData[128];
uint8_t RxData[128];

uint32_t num1;
uint32_t num2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern uint16_t OutputRegisters[OutRegSize];
extern uint16_t InputRegisters[InRegSize];
extern uint8_t OutputCoils[OutCoilsSize];
extern uint8_t InputCoils[InCoilsSize];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void usart1_transmit_dma_config(uint8_t *data, uint16_t size){
	
	USART1 -> CR3 |= USART_CR3_DMAT;
	USART1 -> CR1 |= USART_CR1_TCIE; // прерывание по окончанию передачи ответа от slave
	
	DMA1_Channel1 -> CCR |= DMA_CCR_TCIE;
	
	DMA1_Channel1 -> CNDTR = size;
	DMA1_Channel1 -> CPAR = (uint32_t)&USART1 -> TDR;
	DMA1_Channel1 -> CMAR = (uint32_t)data;
	
}

void usart1_transmit_dma(){
	
	DMA1_Channel1 -> CCR &= ~(DMA_CCR_EN);
	
	GPIOC -> ODR |= (1 << 6); //включить передачу по конвертеру
	
	DMA1_Channel1 -> CCR |= DMA_CCR_EN;
}

void usart1_receive_dma_ch2_config(uint8_t *data, uint16_t size){
	
	USART1 -> CR3 |= USART_CR3_DMAR;
	
	USART1 -> CR1 |= USART_CR1_IDLEIE; // прерывание по IDLE для обработки запроса от master
	
	DMA1_Channel2 -> CNDTR = size;
	DMA1_Channel2 -> CPAR = (uint32_t)&USART1 -> RDR;
	DMA1_Channel2 -> CMAR = (uint32_t)data; 
	
	DMA1_Channel2 -> CCR |= DMA_CCR_EN;
	
}

void DMA1_Channel1_IRQHandler(void){
	
	if (DMA1 -> ISR & DMA_ISR_TCIF1){
		
		DMA1 -> IFCR |= DMA_ISR_TCIF1;
		
		DMA1_Channel1 -> CCR &= ~DMA_CCR_EN;
		
	}

}

void calc(char act){
		
	num1 = ((uint32_t)(num1_H) << 16) | num1_L;
	num2 = ((uint32_t)(num2_H) << 16) | num2_L;
	uint32_t answer = 0;
	
	if ((act == '/' && num2 == 0) || (act == '-' && num1 < num2)) {
    ERROR = 1;
	} else {
    ERROR = 0;
	}
	
	switch (act){
		case '*': 
			answer = num1 * num2;
			break;
		case '+':
			answer = num1 + num2;
			break;
		case '-':
			answer = num1 - num2;
			break;
		case '/':
			answer = num1 / num2;
			break;
	}
	
	answer_H = (uint16_t)(answer >> 16);
	answer_L = (uint16_t)(answer & 0xFFFF);

}
	


void TIM6_DAC_IRQHandler(void){
	
	TIM6 -> SR &= ~TIM_SR_UIF;
	
	calc(ACTION);
	
	
}

void USART1_IRQHandler(void){
	
	if (USART1 -> ISR & USART_ISR_IDLE){
		
		USART1 -> ICR |= USART_ICR_IDLECF; 
		
		DMA1_Channel2 -> CCR &= ~DMA_CCR_EN;
		
		uint16_t frameLength = 128 - (DMA1_Channel2 -> CNDTR);

		ModBusRTU_PR(RxData, frameLength, TxData, RS485_U0_send);
		
		DMA1_Channel2->CMAR = (uint32_t)RxData;
		DMA1_Channel2->CNDTR = 128;
		
		DMA1_Channel2 -> CCR |= DMA_CCR_EN;
		
	}
	
	if (USART1 -> ISR & USART_ISR_TC){
		
		USART1 -> ICR |= USART_ICR_TCCF;
		
		GPIOC -> ODR &= ~(1 << 6); // включить режим приёма конвертера
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
	
	num1 = 0;
	num2 = 0;

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
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	GPIOC -> ODR &= ~(1 << 6); //включить приёмник 
	
	usart1_transmit_dma_config(TxData, 128);
	usart1_receive_dma_ch2_config(RxData, 128);
	
	TIM6->CR1 |= TIM_CR1_CEN;
	TIM6->DIER |= TIM_DIER_UIE;
	
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
