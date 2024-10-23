#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H
#include "stm32f4xx_hal.h"

//======================= Defines ==============================
#define COMMUNICATION_BUFFER_MAX_SIZE				256
#define COMMUNICATION_DMA_BUFFER_MAX_SIZE		    1

#define APPLICATION_OFFSET												0x8000

typedef enum
{
	UART_ReceiveWith_IT=1,
	UART_ReceiveWith_DMA=2
}tdeUartReceiveOption;
typedef enum
{
	Init_USB_Uart=1,
	Init_RS485_Uart,
	Init_RS232_Uart
}tdeUartInitType;
typedef enum
{
	USB_Uart_Handler=1,
	RS485_Uart_Handler,
	RS232_Uart_Handler
}tdeUartHandler;
typedef struct
{
	volatile uint8_t 	buffer[COMMUNICATION_BUFFER_MAX_SIZE];
	volatile uint16_t	bufferSize;
	volatile uint16_t	writePtr;
	volatile uint16_t	readPtr;
	volatile uint32_t	lastUpdateTime;
}tdsCommunicationBuffer;

#endif /* __TYPEDEFS_H */
