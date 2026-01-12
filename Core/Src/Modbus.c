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

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


uint8_t Ready_To_ModBus;


uint16_t InputRegisters[InRegSize] = {
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   30001-30010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 30011-30020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 30021-30030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 30031-30040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 30041-30050
};

uint16_t OutputRegisters[OutRegSize] = {
		1235,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   40001-40010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 40011-40020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 40021-40030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 40031-40040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 40041-40050
};

uint8_t InputCoils[InCoilsSize] = {1};
uint8_t OutputCoils[OutCoilsSize] = {1};

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/*----------------------------------------------------------------------------*/
/* Calculate CRC                                                              */
/*----------------------------------------------------------------------------*/
static uint16_t MODBUS_CRC16_v2( uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_lo ^ *buffer++; /* calculate the CRC  */
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
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

			
			for(*TxLength = 2; *TxLength < RxLength; (*TxLength)++)		
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
				for(*TxLength = 2; *TxLength < RxLength; (*TxLength)++)		/*Формируем ответ как Эхо из запроса*/
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
		if((RegValue >= 0x0) && (RegValue <= 0xFFFF))		/*(Псевдо проверка, возможно в будущем понадобится установить границы)*/
		{
			OutputRegisters[StartAdr] = RegValue;

			for(*TxLength = 2; *TxLength < RxLength; (*TxLength)++)		/*Формируем Эхо запроса*/
			{
				TxData[*TxLength] = RxData[*TxLength];
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); 	/*Ошибка команды, невозможно записать полученное число*/
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); /*Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон*/
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
				for(*TxLength = 2; *TxLength < RxLength; (*TxLength)++)		/*Формируем ответ как Эхо из запроса*/
				{
					TxData[*TxLength] = RxData[*TxLength];
				}
			}
			else
			{
				Error_modbusRTU (TxData, TxLength, 0x04); /*Ошибка обработки данных, кол-во регистров для записи и кол-во байт в запросе не совпадают*/
			}
		}
		else
		{
			Error_modbusRTU (TxData, TxLength, 0x03); /*Ошибка команды, невозможно записать запрошенное кол-во регистров*/
		}
	}
	else
	{
		Error_modbusRTU (TxData, TxLength, 0x02); /*Ошибка адреса, принятый стартовый адрес выходит за допустимый диапазон*/
	}
}

/*-----------------------------------------------------------------------------*/
/* Receiver handler                                                           */
/*----------------------------------------------------------------------------*/
void ModBusRTU_PR(uint8_t *Receiver_arr, uint16_t c_Receiver_arr, uint8_t *Transceiver_arr, void (*ModBusRTU_CallBack)(uint8_t)) /*Modbus RTU Processing and Responding*/
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

				//case(READ_WRITE_MULTIPLE_REGISTERS):

				//break;

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
	
	/*
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

	DMA1_Channel1 -> CMAR = (uint32_t)Tx_Modbus_RS485;
	DMA1_Channel1 -> CNDTR = c_Transceiver_arr;
	DMA1_Channel1 -> CCR |= DMA_CCR_EN;
	*/
	
	DMA1_Channel1 -> CNDTR = c_Transceiver_arr;
	DMA1_Channel1 -> CCR &= ~(DMA_CCR_EN);
	
	GPIOC -> ODR |= (1 << 6); //включить передачу по конвертеру
	
	DMA1_Channel1 -> CCR |= DMA_CCR_EN;
	
}

/* USER CODE END 1 */

