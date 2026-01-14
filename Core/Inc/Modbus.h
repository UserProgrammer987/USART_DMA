/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_H__
#define __MODBUS_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Settings for buffurs size */
/*****************************/
#define InRegSize			50
#define OutRegSize		50

#define InCoilsSize		10
#define OutCoilsSize	10

#define RECEIVER_SIZE 		128
#define TRANSCEIVER_SIZE 	128

/* Modbus Function Codes -----------------------------------------------------*/
/******************************************************************************/
#define	SLAVE_ID 													0x01

#define READ_COIL_STATUS 									0x01
#define READ_DISCRETE_INPUTS 							0x02
#define READ_HOLDING_REGISTERS						0x03
#define READ_INPUT_REGISTERS							0x04

#define FORCE_SINGLE_COIL									0x05
#define PRESET_SINGLE_REGISTER						0x06

#define FORCE_MULTIPLE_COIL								0x0F
#define PRESET_MULTIPLE_REGISTERS					0x10

#define MASK_WRITE_REGISTER								0x16
#define READ_WRITE_MULTIPLE_REGISTERS			0x17

#define READ_FIFO_QUEUE										0x18

#define READ_EXCEPTION_STATUS							0x07
#define DIAGNOSTIC												0x08
#define GET_CORN_EVENT_COUNTER						0x0B
#define GET_CORN_EVENT_LOG								0x0C

#define REPORT_SERVER_ID									0x11

/* Defines for values in Input Register (RO) -------------------------------------------------------*/
/****************************************************************************************************/




/* Defines for values in Output Register (RW) ------------------------------------------------------*/
/****************************************************************************************************/



/* Defines for values in Input Coils (RO) ----------------------------------------------------------*/
/****************************************************************************************************/



/* Defines for values in Output Coils (RW) ---------------------------------------------------------*/
/****************************************************************************************************/


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Function prototypes ---------------------------------------------------------*/
/********************************************************************************/
void ModBusRTU_PR(uint8_t *Receiver_arr, uint8_t c_Receiver_arr, uint8_t *Transceiver_arr, void (*ModBusRTU_CallBack)(uint8_t));
void RS485_U0_send(uint8_t c_Transceiver_arr);
void StartModbusTask(void const * argument);


#endif /* __MODBUS_H__ */

