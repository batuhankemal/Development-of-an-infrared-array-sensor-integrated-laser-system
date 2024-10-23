#ifndef __SERIALHANDLER_H
#define __SERIALHANDLER_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"
#include "gpio.h"
#include "typedefs.h"

	 
//======================= define ===============================

//======================= typedef ==============================

extern volatile tdsCommunicationBuffer gUSB_CommStruct;
extern volatile tdsCommunicationBuffer gRS485_CommStruct;
extern uint8_t gUserID[4];
extern uint8_t receivedID[4];
extern uint8_t tState;
	 
	 
extern volatile uint8_t gRS485_DMABuffer;
extern void UART_RxAgain(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern void Init_UartHandler(tdeUartInitType pUart,UART_HandleTypeDef* pUartInstance);
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
extern void USB_WriteChar(uint8_t pCh);
extern void USB_Send(uint8_t *pBuff,uint16_t pSize);
extern void RS485_WriteChar(uint8_t pCh);
extern void RS485_Send(uint8_t *pBuff,uint16_t pSize);
extern void RS232_WriteChar(uint8_t pCh);
extern void RS232_Send_String(uint8_t *pBuff);
extern void hexDump (char *desc, void *addr, int len);
extern void UartHandler_ClearAndSetSwitchBuffer(uint8_t pState,uint8_t *pPtr,uint16_t pBuffLen);
	 
extern void UartHandler_Add2Buffer(tdsCommunicationBuffer* pSource,uint8_t pVal);
extern void UartHandler_Add2GPSBuffer(tdsCommunicationBuffer* pSource,uint8_t pVal);
extern void UartHandler_Print(tdeUartHandler pUart);
extern void UartHandler_MovePointer(tdsCommunicationBuffer* pSource,uint16_t pPointerIndx);
extern void UartHandler_HandleMessages(tdeUartHandler pUart);
extern void UartHandler_ClearBuffer(tdeUartHandler pUart);
extern void UartCrossHandler(void);
extern void setBaudRate(uint16_t baudrate);
int8_t UartHandler_GetModbusData(volatile tdsCommunicationBuffer * pBuffer, uint16_t pSize, uint8_t * pData);
int8_t UartHandler_GetFromBuffer(volatile tdsCommunicationBuffer* pSource,uint8_t * pVal);
#ifdef __cplusplus
}
#endif
#endif /*__SERIALHANDLER_H */
