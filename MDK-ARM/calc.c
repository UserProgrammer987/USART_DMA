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
#define ENTER 13

// каждая кнопка имеет 2 функции - менять BUTTON_REGISTER на нужный символ и активировать PRESSED_FLAG
#define PRESSED_FLAG OutputCoils[0] // флаг нажатой кнопки
#define BUTTON_REGISTER OutputRegisters[0] // регистр кнопки
#define ACTION_REGISTER OutputRegisters[5] // регистр действия 
#define ANSWER_REGISTER OutputRegisters[6] // регистр ответа 
#define ANSWER(indx) OutputRegisters[(indx) + 6]


#define num_L(indx) OutputRegisters[(indx) * 2 - 1] 
#define num_H(indx) OutputRegisters[(indx) * 2]

uint8_t actual_number = 0;

uint32_t intNum[2] = {0};
uint32_t fracNum[2] = {0};
float fracPart[2];
float num[2];

float answer = 0;

uint8_t numbers[10];
uint8_t ind = 0;

uint8_t fracNumbers[10];
uint8_t fracInd = 0;

uint8_t fracFlag = 0;
uint8_t signFlag = 0;
uint8_t actFlag = 0;

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

void input_numbers_intNum(){
	
	numbers[ind] = BUTTON_REGISTER;
	BUTTON_REGISTER = 0;
	ind++;
	
	assemble_num(numbers, &intNum[actual_number], 0);
	num[actual_number] = (float)(intNum[actual_number]);
	if (signFlag) num[actual_number] = -num[actual_number];
	floatToBits(num[actual_number], &num_H(actual_number+1), &num_L(actual_number+1));

	
}

void input_numbers_fracNum(){
	
	fracNumbers[fracInd] = BUTTON_REGISTER;
	BUTTON_REGISTER = 0;
	fracInd++;
		
	assemble_num(fracNumbers, &fracNum[actual_number], 1);

	float div = 1.0f;
	for (uint8_t i = 0; i < fracInd; i++){
		div *= 10.0f;
	}
	
	fracPart[actual_number] = (float)(fracNum[actual_number]) / div;
	num[actual_number] = (float)intNum[actual_number] + fracPart[actual_number];
	if (signFlag) num[actual_number] = -num[actual_number];
	
	floatToBits(num[actual_number], &num_H(actual_number+1), &num_L(actual_number+1));	
	
}

void changeSign(){
	
	signFlag = ~signFlag;
	num[actual_number] *= -1;
	floatToBits(num[actual_number], &num_H((actual_number)+1), &num_L(actual_number+1));	
}

void act(){
	actFlag = 1;
	
	actual_number++;
	ind = 0;
	fracInd = 0;
	fracFlag = 0;
	signFlag = 0;
	
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
					input_numbers_intNum();
					break;
				case 1: 
					input_numbers_fracNum();
					break;
			}
			
		}
		
		if (BUTTON_REGISTER == DOT){
			fracFlag = 1;
		}
		
		if (BUTTON_REGISTER == ACTION){
			act();
		}
		
		if (BUTTON_REGISTER == ENTER){
			switch (ACTION_REGISTER) {
				case '+': 
					answer = num[0] + num[1];
					break;
				case '-':
					answer = num[0] - num[1];
					break;
				case '*':
					answer = num[0] * num[1];
					break;
				case '/':
					answer = num[0] / num[1];
					break;
			}
			
			floatToBits(answer, &ANSWER(1), &ANSWER(0));
			
		}
		
		
	}
	
}

void TIM6_DAC_IRQHandler(void){
	
	TIM6 -> SR &= ~TIM_SR_UIF;
	
	button_PR();

}
