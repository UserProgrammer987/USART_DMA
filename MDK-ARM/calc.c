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

#define num_L(indx) OutputRegisters[(indx) * 2 - 1]  //регистр LOW байта числа
#define num_H(indx) OutputRegisters[(indx) * 2] //регистр HIGH байта числа

#define HISTORY_num(id, numInd, HL) OutputRegisters[(id) * 7 + (numInd) * 2 + 8 + (HL)] 
#define HISTORY_action(index) OutputRegisters[(index) * 7 + 14]
#define HISTORY_answer(i, HL) OutputRegisters[(i) * 7 + 12 + (HL)]

#define LOW 0
#define HIGH 1


uint8_t actual_number = 0;

uint32_t intNum[2] = {0};
uint32_t fracNum[2] = {0};
float fracPart[2];
float num[2] = {0};

float answer = 0;

typedef struct {
	float num1;
	float num2;
	float ans;
	char action;
} history;

history equations[3];

uint8_t numbers[10];
uint8_t ind = 0;

uint8_t fracNumbers[10];
uint8_t fracInd = 0;

uint8_t fracFlag = 0;
uint8_t signFlag = 0;
uint8_t answerFlag = 0;

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

void input_numbers_intNum(uint8_t n){
	
	numbers[ind] = BUTTON_REGISTER;
	BUTTON_REGISTER = 0;
	ind++;
	
	assemble_num(numbers, &intNum[n], 0);
	num[n] = (float)(intNum[n]);
	if (signFlag) num[n] = -num[n];
	floatToBits(num[n], &num_H(n+1), &num_L(n+1));

	
}

void input_numbers_fracNum(uint8_t n){
	
	fracNumbers[fracInd] = BUTTON_REGISTER;
	BUTTON_REGISTER = 0;
	fracInd++;
		
	assemble_num(fracNumbers, &fracNum[n], 1);

	float div = 1.0f;
	for (uint8_t i = 0; i < fracInd; i++){
		div *= 10.0f;
	}
	
	fracPart[n] = (float)(fracNum[n]) / div;
	num[n] = (float)intNum[n] + fracPart[n];
	if (signFlag) num[n] = -num[n];
	
	floatToBits(num[n], &num_H(n+1), &num_L(n+1));	
	
}

void changeSign(uint8_t n){
	
	signFlag = ~signFlag;
	num[n] *= -1;
	floatToBits(num[n], &num_H(n+1), &num_L(n+1));	
}

void act(){
	
	actual_number++;
	ind = 0;
	fracInd = 0;
	fracFlag = 0;
	signFlag = 0;
	
}

void clearCalc(){
	
	ind = 0;
	fracInd = 0;
	fracFlag = 0;
	signFlag = 0;
	
	actual_number = 0;
	num[0] = 0;
	num[1] = 0;
	answer = 0;
	memset(numbers, 0, sizeof(numbers));
	memset(fracNumbers, 0, sizeof(fracNumbers));
	
	floatToBits(num[0], &num_H(1), &num_L(1));
	floatToBits(num[1], &num_H(2), &num_L(2));
	floatToBits(answer, &ANSWER(1), &ANSWER(0));
	ACTION_REGISTER = 0;
	
}

void historyFill(uint8_t index){
	
	equations[index].num1 = num[0];
	equations[index].num2 = num[1];
	equations[index].ans = answer;
	equations[index].action = ACTION_REGISTER;
	
}

void historyShow(uint8_t id){
	
	float n[2] = {equations[id].num1, equations[id].num2};
	char ac = equations[id].action;
	HISTORY_action(id) = ac;
	float an = equations[id].ans;
	
	floatToBits(n[0], &HISTORY_num(id, 0, HIGH), &HISTORY_num(id, 0, LOW));
	floatToBits(n[1], &HISTORY_num(id, 1, HIGH), &HISTORY_num(id, 1, LOW));
	floatToBits(an, &HISTORY_answer(id, HIGH), &HISTORY_answer(id, LOW));
	
}

void history_PR(){
	
	history temp = equations[1];
	equations[1] = equations[0];
	equations[2] = temp;
	
	historyFill(0);	
	
	for (uint8_t i = 0; i < 3; i++){
		historyShow(i);
	}
	
}

void NUMBER_PR(){
	
	if (answerFlag){
		clearCalc();
		answerFlag = 0;
	}
			
	switch (fracFlag){
		case 0: 
			input_numbers_intNum(actual_number);
			break;
		case 1: 
			input_numbers_fracNum(actual_number);
			break;
	}

}

void SIGN_PR(){
	
	if (answerFlag){
		clearCalc();
		answerFlag = 0;
	}
	
	changeSign(actual_number);
			
}

void ENTER_PR(){
	
	if (answerFlag){
		num[0] = answer;	
	}
			
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
	answerFlag = 1;
	
	history_PR();
}

void button_PR(){
	
	if (PRESSED_FLAG) {
		PRESSED_FLAG = 0;
		
		if (BUTTON_REGISTER <= NUMBER){
			NUMBER_PR();
		}
		
		switch (BUTTON_REGISTER){
			case SIGN:
				SIGN_PR();
				break;
			case DOT:
				fracFlag = 1;
				break;
			case ACTION:
				act();
				break;
			case ENTER:
				ENTER_PR();
				break;
			default:
				break;
		}

	}
	
}

void TIM6_DAC_IRQHandler(void){
	
	TIM6 -> SR &= ~TIM_SR_UIF;
	
	button_PR();

}
