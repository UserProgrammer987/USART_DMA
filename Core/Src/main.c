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
uint8_t nums[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

uint8_t cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void usart1_transmit_dma(uint8_t *data, uint8_t size){
		
	USART1 -> CR3 |= USART_CR3_DMAT;
	
	DMA1_Channel1 -> CCR |= DMA_CCR_TCIE;
	
	DMA1_Channel1 -> CNDTR = size;
	DMA1_Channel1 -> CPAR = (uint32_t)&USART1 -> TDR;
	DMA1_Channel3 -> CMAR = (uint32_t)data;
	
	DMA1_Channel3 -> CCR |= DMA_CCR_EN;
	
}

void usart1_receive_dma_ch2_config(uint8_t *data, uint8_t size){
	
	USART1 -> CR3 |= USART_CR3_DMAR;
	
	DMA1_Channel2 -> CNDTR = size;
	DMA1_Channel2 -> CPAR = (uint32_t)&USART1 -> RDR;
	DMA1_Channel2 -> CMAR = (uint32_t)data; 
	
	DMA1_Channel2 -> CCR |= DMA_CCR_EN;
	

}

void usart2_transmit_dma(uint8_t *data, uint8_t size){
	/*
	DMA1_Channel3 -> CNDTR = size;
	DMA1_Channel3 -> CPAR = (uint32_t)&USART2 -> TDR;
	DMA1_Channel3 -> CMAR = (uint32_t)data;
	
	//USART1 -> ICR |= USART_ICR_TCCF;
			
	DMA1_Channel3 -> CCR |= DMA_CCR_EN;
			
	while( !(DMA1 -> ISR & DMA_ISR_TCIF3) );
			
	DMA1 -> IFCR = DMA_IFCR_CTCIF3;
			
	DMA1_Channel3 -> CCR &= ~DMA_CCR_EN;
	
	//while (!(USART2->ISR & USART_ISR_TXE));
	
	*/
	
	//USART2 -> CR1 |= USART_CR1_TE;
	
	USART2 -> CR3 |= USART_CR3_DMAT; 
	
	DMA1_Channel3 -> CCR |= DMA_CCR_TCIE; 
	
	//DMA1_Channel3 -> CCR |= DMA_CCR_HTIE;
	//DMA1_Channel3 -> CCR |= DMA_CCR_TEIE;
	
	/*
	DMA1_Channel3 -> CCR |= DMA_CCR_TCIE;
	DMA1_Channel3 -> CCR |= DMA_CCR_DIR;
	DMA1_Channel3 -> CCR &= ~DMA_CCR_CIRC;
	DMA1_Channel3 -> CCR |= DMA_CCR_PINC;
	DMA1_Channel3 -> CCR &= ~(DMA_CCR_PSIZE | DMA_CCR_MSIZE);
	DMA1_Channel3 -> CCR |= DMA_CCR_PL_0;
	*/
	
	DMA1_Channel3 -> CNDTR = size;
	DMA1_Channel3 -> CPAR = (uint32_t)&USART2 -> TDR;
	DMA1_Channel3 -> CMAR = (uint32_t)data;
	
	//USART1 -> ICR |= USART_ICR_TCCF;
	
	DMA1_Channel3 -> CCR |= DMA_CCR_EN;
	
	//DMA1_Channel3 -> CCR &= ~DMA_CCR_EN;

}



void usart2_recieve_DMA_ch4_config(uint8_t *data, uint8_t size){
	
	//USART2 -> CR1 |= USART_CR1_RE;
	
	USART2 -> CR3 |= USART_CR3_DMAR; 
	
	//DMA1_Channel4 -> CCR |=  DMA_CCR_TCIE;
	//DMA1_Channel4 -> CCR &= ~(DMA_CCR_DIR);
	//DMA1_Channel4 -> CCR |= DMA_CCR_CIRC;
	//DMA1_Channel4 -> CCR |= DMA_CCR_MINC;
	//DMA1_Channel4 -> CCR &= ~(DMA_CCR_PSIZE);
	//DMA1_Channel4 -> CCR &= ~(DMA_CCR_MSIZE);
	//DMA1_Channel4 -> CCR |= DMA_CCR_PL_0 | DMA_CCR_PL_1;
	
	DMA1_Channel4 -> CNDTR = size;
	DMA1_Channel4 -> CPAR = (uint32_t)&USART2 -> RDR;
	DMA1_Channel4 -> CMAR = (uint32_t)data; 
	
	DMA1_Channel4 -> CCR |= DMA_CCR_EN;
	
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Transmit_DMA(&huart2, nums, 10);
	
	//NVIC_SetPriority(DMA1_Channel4_IRQn, 1);
	//NVIC_SetPriority(DMA1_Channel3_IRQn, 0);
	
	//NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	//NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	
	usart2_recieve_DMA_ch4_config(nums, 10);
	usart1_receive_dma_ch2_config(nums, 10);
	
	//HAL_TIM_Base_Start_IT(&htim6);
	
	TIM6 -> CR1 |= TIM_CR1_CEN;
	TIM6 -> DIER |= TIM_DIER_UIE;
	
	
	

	
	
	
	

	//HAL_UART_Receive_DMA(&huart2, nums, 10);
	
	
	
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
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
   //GPIOC -> ODR ^= (1 << 6); 

	 
	//HAL_UART_Transmit_DMA(&huart2, nums, 10);
	//HAL_UART_Transmit_DMA(&huart1, nums, 10);



		
		
}


/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	

		GPIOC -> ODR ^= (1 << 6);
	
		for (uint8_t i = 0; i<10; i++){
			
			nums[i]++;
			
		}	

}*/

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
