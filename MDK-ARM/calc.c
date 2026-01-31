#include "stdbool.h"
#include "Modbus.h"
#include <string.h>

extern uint16_t OutputRegisters[OutRegSize];
extern uint16_t InputRegisters[InRegSize];
extern uint8_t OutputCoils[OutCoilsSize];
extern uint8_t InputCoils[InCoilsSize];

#define NUMBER 9
#define DOT 10
#define SIGN 11
#define ACTION 12

// каждая кнопка имеет 2 функции - менять BUTTON_REGISTER на нужный символ и активировать PRESSED_FLAG
#define PRESSED_FLAG OutputCoils[0] // флаг нажатой кнопки
#define BUTTON_REGISTER OutputRegisters[0] //

#define num1_L OutputRegisters[1] 
#define num1_H OutputRegisters[2]

#define num2_L OutputRegisters[3]
#define num2_H OutputRegisters[4]


uint32_t intNum1 = 0;
uint32_t intNum2 = 0;

uint32_t fracNum1 = 0;
float fracPart1;

uint32_t fracNum2 = 0;
float fracPart2;

float num1;
float num2;

uint8_t numbers[10];
uint8_t ind = 0;

uint8_t fracNumbers[10];
uint8_t fracInd = 0;

uint8_t fracFlag = 0;
uint8_t signFlag = 0;

void floatToBits(float num, uint16_t* float_H, uint16_t* float_L){
	
	uint32_t floatBuf;
	
	memcpy(&floatBuf, &num, sizeof(float));
	
	*float_H = (uint16_t)(floatBuf >> 16);
	*float_L = (uint16_t)(floatBuf & 0xFFFF);
	
}

void assemble_num(uint8_t nums[], uint32_t* number, uint8_t fracIndicator){

	uint8_t indx;
	if (fracIndicator){
		indx = fracInd;
	} else {
		indx = ind;
	}

	*number = 0;
	
	uint8_t i = 0;
	for (i = 0; i < indx; i++){
		*number = (*number * 10) + nums[i];
	}

}

void input_numbers_intNum1(){
	
	numbers[ind] = BUTTON_REGISTER;
	BUTTON_REGISTER = 0;
	ind++;
	
	assemble_num(numbers, &intNum1, 0);
	num1 = (float)(intNum1);
	if (signFlag) num1 = -num1;
	floatToBits(num1, &num1_H, &num1_L);

	
}

void input_numbers_fracNum1(){
	
	fracNumbers[fracInd] = BUTTON_REGISTER;
	BUTTON_REGISTER = 0;
	fracInd++;
		
	assemble_num(fracNumbers, &fracNum1, 1);

	float div = 1.0f;
	for (uint8_t i = 0; i < fracInd; i++){
		div *= 10.0f;
	}
	
	fracPart1 = (float)(fracNum1) / div;
	num1 = (float)intNum1 + fracPart1;
	if (signFlag) num1 = -num1;
	
	floatToBits(num1, &num1_H, &num1_L);	
	
}

void changeSign(){
	
	signFlag = ~signFlag;
	num1 *= -1;
	floatToBits(num1, &num1_H, &num1_L);	
}

void button_PR(){
	
	if (PRESSED_FLAG) {
		PRESSED_FLAG = 0;
		
		if (BUTTON_REGISTER == SIGN){
			changeSign();
		}
		
		if (BUTTON_REGISTER <= NUMBER){
			switch (fracFlag){
				case 0: 
					input_numbers_intNum1();
					break;
				case 1: 
					input_numbers_fracNum1();
					break;
			}
			
		}
		
		if (BUTTON_REGISTER == 10){
			fracFlag = 1;
		}
		
		
	}
	
}

void TIM6_DAC_IRQHandler(void){
	
	TIM6 -> SR &= ~TIM_SR_UIF;
	
	button_PR();

}
