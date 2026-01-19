/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Modbus.c
  * @brief   This file provides code for the definition
  *          of all receiving and transmitting data over the Modbus protocol.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Modbus.h"

/* USER CODE BEGIN 0 */

uint8_t Ready_To_ModBus;


uint16_t InputRegisters[InRegSize] = {0};

uint16_t OutputRegisters[OutRegSize] = {0};

uint8_t InputCoils[InCoilsSize] = {0};
uint8_t OutputCoils[OutCoilsSize] = {0};

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/*----------------------------------------------------------------------------*/
/* Calculate CRC                                                              */
/*----------------------------------------------------------------------------*/
static uint16_t MODBUS_CRC16_v2( const uint8_t *buf, uint16_t len )
{
	static const uint16_t table[2] = { 0x0000, 0xA001 };
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;
	unsigned int xor = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			xor = crc & 0x01;
			crc >>= 1;
			crc ^= table[xor];
		}
	}

	return crc;
}



/*--------------------------------------------------------------------------*/
/*Errors forming                                                            */
/*--------------------------------------------------------------------------*/
void Error_modbusRTU (uint8_t *TxData, uint8_t *TxLength, uint8_t Temp_Error)
{
	TxData[1] |= 0x80;				/*Functional code with a modified bit*/
	TxData[2] = Temp_Error; 		/*Error Code*/

	*TxLength = 3;					/*Pointer on the end of array with Error*/
}

/*--------------------------------------------------------------------------*/
/*Read Coils and Discrete (commands 0x01, 0x02)                             */
/*--------------------------------------------------------------------------*/
void Read_Coils(uint8_t *RxData, uint8_t RxLength, uint8_t *TxData, uint8_t *TxLength, volatile uint8_t *Arr, uint16_t Arr_size)
{
	uint16_t StartAdr 	 = (((uint16_t)RxData[2]<<8)|(RxData[3]));
	uint16_t CoilCounter = (((uint16_t)RxData[4]<<8)|(RxData[5]));

	uint8_t StartAdrOf    = StartAdr % 8;		/*Смещение начального адреса относительно начала байта*/
	uint8_t CoilCounterAd = CoilCounter % 8;	/*Добавка битов поверх целого количества байт*/

				/*(CoilSize * 8)*/
	if(StartAdr < (Arr_size << 3))
	{														/*(CoilSize * 8)*/
		if((CoilCounter >= 1) && ((StartAdr + CoilCounter) <= (Arr_size << 3)))
		{
			/*точное кол-во передаваемых байт*/
			if(((CoilCounter + 7) >> 3) <= (TRANSCEIVER_SIZE-5))
			{
				StartAdr >>= 3;			// деление на 8
				CoilCounter >>= 3;		// деление на 8

				TxData[2] = CoilCounter;

				uint8_t i;
				for(i = 0; i < CoilCounter; i++)
				{
					TxData[3 + i] = (uint8_t)(((Arr[StartAdr + 1 + i] << 8)|(Arr[StartAdr + i])) >> StartAdrOf);	/*Получение байта по смещению*/
				}

				if(CoilCounterAd)
				{
					TxData[2]++;
					TxData[3 + i] = ((uint8_t)(((Arr[StartAdr + 1 + i] << 8)|(Arr[StartAdr + i])) >> StartAdrOf)) & ~(0xFF << CoilCounterAd); /*Последний байт маскируется согласно добавке*/
					i++;
				}
				*TxLength = 3 + i;
			}
			else
			{
				Error_modbusRTU (TxData, TxLength, 0x04); /*Ошибка обработки данных, ответ не влезет массив*/
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); /*Ошибка команды, невозможно передать запрошенное кол-во бит*/
		}

	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); /*Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон*/
	}
}

/*--------------------------------------------------------------------------*/
/*Force Single Coil (command 0x05)                                          */
/*--------------------------------------------------------------------------*/
void Force_Single_Coil(uint8_t *RxData, uint8_t RxLength, uint8_t *TxData, uint8_t *TxLength)
{
	uint16_t StartAdr = (((uint16_t)RxData[2]<<8)|(RxData[3]));
	uint16_t Comand   = (((uint16_t)RxData[4]<<8)|(RxData[5]));

	uint8_t StartAdrOf  = StartAdr % 8;		/*Смещение начального адреса относительно начала байта*/

	StartAdr >>= 3;		/*Деление на 8*/	/*Получение указателя на байт*/

	if(StartAdr < OutCoilsSize)
	{
		if((Comand == 0xFF00) || (Comand == 0x0000))
		{
			switch(Comand)
			{
				case(0xFF00):
					OutputCoils[StartAdr] |= (1<<StartAdrOf);
				break;

				case(0x0000):
					OutputCoils[StartAdr] &=~ (1<<StartAdrOf);
				break;
			}
			for(*TxLength = 2; *TxLength < 6; (*TxLength)++)		
			{
				TxData[*TxLength] = RxData[*TxLength];
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); /*Ошибка команды, полученная команда не соответствует ожидаемой*/
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); /*Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон*/
	}
}

/*--------------------------------------------------------------------------*/
/*Force Multiple Coils (command 0x0F)                                       */
/*--------------------------------------------------------------------------*/
void Force_Multiple_Coils(uint8_t *RxData, uint8_t RxLength, uint8_t *TxData, uint8_t *TxLength)
{
	uint16_t StartAdr 	 = (((uint16_t)RxData[2]<<8)|(RxData[3]));
	uint16_t CoilCounter = (((uint16_t)RxData[4]<<8)|(RxData[5]));
	uint16_t ByteCounter = RxData[6];

	uint8_t StartAdrOf    = StartAdr % 8;		/*Смещение начального адреса относительно начала байта*/
	uint8_t CoilCounterAd = CoilCounter % 8;	/*Добавка битов поверх целого количества байт*/

				/*(CoilSize * 8)*/
	if(StartAdr < (OutCoilsSize << 3))
	{														/*(CoilSize * 8)*/
		if((CoilCounter >= 1) && ((StartAdr + CoilCounter) <= (OutCoilsSize << 3)))
		{
			/*точное кол-во полученных байт*/
			if((((CoilCounter + 7) >> 3) == ByteCounter) && ((ByteCounter + 7) == RxLength))
			{
				StartAdr >>= 3;			// деление на 8
				CoilCounter >>= 3;		// деление на 8

				uint8_t i;
				for(i = 0; i < CoilCounter; i++)
				{
					OutputCoils[StartAdr+i] &=~ (0xFF << StartAdrOf);				/*Очистка битов перед записью*/
					OutputCoils[StartAdr+i] |= (RxData[7+i] << StartAdrOf); 		/*Первая часть байта помещается в нужный адрес со смещением*/

					OutputCoils[StartAdr+1+i] &=~ (0xFF >> (8-StartAdrOf));			/*Очистка битов перед записью*/
					OutputCoils[StartAdr+1+i] |= (RxData[7+i] >> (8-StartAdrOf));		/*Следующая часть байта помещается в следующий байт со смещением*/
				}

				if(CoilCounterAd)	/*Если кол-во битов не кратно 8*/
				{
					RxData[7+i] &= ~(0xFF << CoilCounterAd);		/*маскируем последний байт, на случай если там не нули*/

					OutputCoils[StartAdr+i] &=~ ((0xFF >> (8 - CoilCounterAd)) << StartAdrOf); /*очистка битов перед записью*/
					OutputCoils[StartAdr+i] |= (RxData[7+i] << StartAdrOf);	/*Запись последнего байта с маскированием ненужных бит (на случай если там не ноль)*/

					if((CoilCounterAd + StartAdrOf) > 8)
					{
						OutputCoils[StartAdr+1+i] &=~ (0xFF >> ((8 - CoilCounterAd) + (8 - StartAdrOf)));			/*очистка битов перед записью*/
						OutputCoils[StartAdr+1+i] |= (RxData[7+i] >> (8-StartAdrOf));
					}
				}

				RxLength -= ByteCounter;
				//Здесь добавил RxLength - 1, потому что иначе скорее всего вызывало бы ошибку на HMI панели(хотя и выполняло бы функцию)
				for(*TxLength = 2; *TxLength < RxLength-1; (*TxLength)++)		/*Формируем ответ как Эхо из запроса*/
				{
					TxData[*TxLength] = RxData[*TxLength];
				}
				
			}
			else
			{
				Error_modbusRTU (TxData, TxLength, 0x04); /*Ошибка обработки данных, ответ не влезет массив*/
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); /*Ошибка команды, невозможно записать запрошенное кол-во бит*/
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); /*Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон*/
	}
}

/*--------------------------------------------------------------------------*/
/*Read Registers (commands 0x03, 0x04)                                      */
/*--------------------------------------------------------------------------*/
void Read_Registers(uint8_t *RxData, uint8_t RxLength, uint8_t *TxData, uint8_t *TxLength, volatile uint16_t *Arr, uint16_t Arr_size)
{
	uint16_t StartAdr   = (((uint16_t)RxData[2]<<8)|(RxData[3]));
	uint16_t RegCounter = (((uint16_t)RxData[4]<<8)|(RxData[5]));

	if(StartAdr < Arr_size)
	{
		if((RegCounter >= 1) && ((StartAdr + RegCounter) <= Arr_size))
		{
			if((RegCounter*2) <= (TRANSCEIVER_SIZE-5))
			{
				TxData[2] = (RegCounter) * 2; //count

				uint8_t i;
				for(i = 0; i < RegCounter; i++)
				{
					TxData[3 + 2*i] = (uint8_t)(Arr[StartAdr+i] >> 8);	/*Запись 2-х байтных слов в два байта массива*/
					TxData[4 + 2*i] = (uint8_t)(Arr[StartAdr+i]);
				}

				*TxLength = 3 + 2*RegCounter;
			}
			else
			{
				Error_modbusRTU (TxData, TxLength, 0x04); /*Ошибка обработки данных, ответ не влезет массив*/
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); /*Ошибка команды, невозможно передать запрошенное кол-во регистров*/
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); /*Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон*/
	}
}

/*--------------------------------------------------------------------------*/
/*Preset Single Register (command 0x06)                                     */
/*--------------------------------------------------------------------------*/
void Preset_Single_Register(uint8_t *RxData, uint8_t RxLength, uint8_t *TxData, uint8_t *TxLength)
{
	uint16_t StartAdr = (((uint16_t)RxData[2]<<8)|(RxData[3]));
	uint16_t RegValue = (((uint16_t)RxData[4]<<8)|(RxData[5]));

	if(StartAdr < OutRegSize)
	{
		if((RegValue >= 0x0) && (RegValue <= 0xFFFF))		//(Псевдо проверка, возможно в будущем понадобится установить границы)/
		{
			OutputRegisters[StartAdr] = RegValue;

			for(*TxLength = 2; *TxLength < 6; (*TxLength)++)		//Формируем Эхо запроса
			{
				TxData[*TxLength] = RxData[*TxLength];
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); 	//Ошибка команды, невозможно записать полученное число
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); //Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон
	}
	
}

/*--------------------------------------------------------------------------*/
/*Preset Multiple Registers (command 0x10)                                     */
/*--------------------------------------------------------------------------*/
void Preset_Multiple_Registers(uint8_t *RxData, uint8_t RxLength, uint8_t *TxData, uint8_t *TxLength)
{
	uint16_t StartAdr    = (((uint16_t)RxData[2]<<8)|(RxData[3]));
	uint16_t RegCounter  = (((uint16_t)RxData[4]<<8)|(RxData[5]));
	uint16_t ByteCounter = RxData[6];

	if(StartAdr < OutRegSize)
	{
		if((RegCounter >= 1) && ((StartAdr + RegCounter) <= OutRegSize))
		{
			if(((RegCounter << 1) == ByteCounter) && ((ByteCounter + 7) == RxLength))
			{
				for(uint8_t i = 0; i < ByteCounter; i += 2)
				{
					OutputRegisters[StartAdr+(i>>1)] = (((uint16_t)RxData[7+i]<<8)|(RxData[8+i]));
				}

				RxLength -= ByteCounter;
				for(*TxLength = 2; *TxLength < (RxLength-1); (*TxLength)++)		//Формируем ответ как Эхо из запроса/
				{
					TxData[*TxLength] = RxData[*TxLength];
				}
			}
			else
			{
				Error_modbusRTU (TxData, TxLength, 0x04); //Ошибка обработки данных, кол-во регистров для записи и кол-во байт в запросе не совпадают
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); //Ошибка команды, невозможно записать запрошенное кол-во регистров
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); //Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон
	}
	
}

/*-----------------------------------------------------------------------------*/
/* Receiver handler                                                           */
/*----------------------------------------------------------------------------*/
void ModBusRTU_PR(uint8_t *Receiver_arr, uint8_t c_Receiver_arr, uint8_t *Transceiver_arr, void (*ModBusRTU_CallBack)(uint8_t)) /*Modbus RTU Processing and Responding*/
{
	uint16_t CRC_MB = 0;
	uint16_t CRC16 = 0;

	uint8_t c_Transceiver_arr = 0;

	if((Receiver_arr[0] == SLAVE_ID) || (Receiver_arr[0] == 0x00))
	{
		CRC_MB 	= (uint16_t)Receiver_arr[--c_Receiver_arr]<<8;
		CRC_MB |= (uint16_t)Receiver_arr[--c_Receiver_arr];

		CRC16 = MODBUS_CRC16_v2(Receiver_arr, c_Receiver_arr);
		
		if(CRC_MB == CRC16)
		{
			Transceiver_arr[0] = SLAVE_ID;
			Transceiver_arr[1] = Receiver_arr[1];

			switch(Receiver_arr[1])
			{
				case(READ_COIL_STATUS): 	/*0x01*/
					Read_Coils(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr, OutputCoils, OutCoilsSize); /*Чтение битов OutputCoils*/
				break;

				case(READ_DISCRETE_INPUTS): /*0x02*/
					Read_Coils(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr, InputCoils, InCoilsSize); /*Чтение битов InputCoils*/
				break;

				case(READ_HOLDING_REGISTERS):	/*0x03*/
					Read_Registers(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr, OutputRegisters, OutRegSize); /*Чтение регистров OutputRegisters*/
				break;

				case(READ_INPUT_REGISTERS):		/*0x04*/
					Read_Registers(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr, InputRegisters, InRegSize); /*Чтение регистров InputRegisters*/
				break;

				case(FORCE_SINGLE_COIL): /*0x05*/
					Force_Single_Coil(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr); /*Запись одного бита*/
				break;

				case(PRESET_SINGLE_REGISTER): /*0x06*/
					Preset_Single_Register(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr); /*Запись одного регистра*/
				break;

				case(FORCE_MULTIPLE_COIL): /*0x0F*/
					Force_Multiple_Coils(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr);	/*Запись нескольких битов*/
				break;

				case(PRESET_MULTIPLE_REGISTERS): /*0x10*/
					Preset_Multiple_Registers(Receiver_arr, c_Receiver_arr, Transceiver_arr, &c_Transceiver_arr); /*Запись нескольких регистров*/
				break;
				
				default:
					Error_modbusRTU (Transceiver_arr, &c_Transceiver_arr, 0x01);	/*Ошибка команды, Полученная команда не поддерживается*/
				break;
			}

			if(Receiver_arr[0] != 0x00) /*Не широковещательная команда*/
			{
				CRC16 = MODBUS_CRC16_v2(Transceiver_arr, c_Transceiver_arr);

				Transceiver_arr[c_Transceiver_arr++] = (uint8_t)CRC16;
				Transceiver_arr[c_Transceiver_arr++] = (uint8_t)(CRC16 >> 8);

				ModBusRTU_CallBack(c_Transceiver_arr);		/*CallBack функция отправки массива на интерфейс*/
			}
		}
	}
}

/*----------------------------------------------------------------------------*/
/* CallBack Functions for Responding                                          */
/*----------------------------------------------------------------------------*/
void RS485_U0_send(uint8_t c_Transceiver_arr)
{

	DMA1_Channel1 -> CNDTR = c_Transceiver_arr;
	
	GPIOC -> ODR |= (1 << 6); //включить передачу по конвертеру
	
	DMA1_Channel1 -> CCR |= DMA_CCR_EN;
	
}

/* USER CODE END 1 */

