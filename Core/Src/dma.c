/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

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

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

