#include "SerialHandler.h"
#include "typedefs.h"
#include "string.h"
#include "stdlib.h"

UART_HandleTypeDef* gUSB_Uart;
UART_HandleTypeDef* gRS485_Uart;
UART_HandleTypeDef* gRS232_Uart;

volatile uint8_t gUSB_DMABuffer;
volatile uint8_t gRS485_DMABuffer;
volatile uint8_t gRS232_DMABuffer;

volatile tdsCommunicationBuffer gUSB_CommStruct;
volatile tdsCommunicationBuffer gRS485_CommStruct;
volatile tdsCommunicationBuffer gRS232_CommStruct;

uint8_t tState = 0;

void Init_UartHandler(tdeUartInitType pUart,UART_HandleTypeDef* pUartInstance)
{
	
	if(pUart==Init_USB_Uart) //RS485
	{
		gUSB_Uart=pUartInstance;
		gUSB_CommStruct.writePtr=0;
		gUSB_CommStruct.readPtr=0;
		gUSB_CommStruct.bufferSize=COMMUNICATION_BUFFER_MAX_SIZE;
		gUSB_CommStruct.lastUpdateTime=HAL_GetTick();
		UartHandler_ClearBuffer(USB_Uart_Handler);
		HAL_UART_Receive_IT(gUSB_Uart,(void *)&gUSB_DMABuffer,1);
	}
    else if(pUart==Init_RS485_Uart) //RS485
	{
		gRS485_Uart=pUartInstance;
		gRS485_CommStruct.writePtr=0;
		gRS485_CommStruct.readPtr=0;
		gRS485_CommStruct.bufferSize=COMMUNICATION_BUFFER_MAX_SIZE;
		gRS485_CommStruct.lastUpdateTime=HAL_GetTick();
		UartHandler_ClearBuffer(RS485_Uart_Handler);
		HAL_UART_Receive_IT(gRS485_Uart,(void *)&gRS485_DMABuffer,1);
	}
    else if(pUart==Init_RS232_Uart) //RS232
	{
		gRS232_Uart=pUartInstance;
		gRS232_CommStruct.writePtr=0;
		gRS232_CommStruct.readPtr=0;
		gRS232_CommStruct.bufferSize=COMMUNICATION_BUFFER_MAX_SIZE;
		gRS232_CommStruct.lastUpdateTime=HAL_GetTick();
		UartHandler_ClearBuffer(RS232_Uart_Handler);
		HAL_UART_Receive_IT(gRS232_Uart,(void *)&gRS232_DMABuffer,1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(HAL_UART_ERROR_FE == huart->ErrorCode)
	{
		HAL_UART_MspDeInit(huart);
		HAL_UART_MspInit(huart);
	}
	if(huart->Instance == USART6)
	{
		HAL_UART_Receive_IT(gUSB_Uart,(void *)&gUSB_DMABuffer,1);
		gUSB_CommStruct.lastUpdateTime = HAL_GetTick();
	}

    else if(huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(gRS485_Uart,(void *)&gRS485_DMABuffer,1);
		gRS485_CommStruct.lastUpdateTime = HAL_GetTick();
	}
    
    else if(huart->Instance == USART2)
	{
		HAL_UART_Receive_IT(gRS232_Uart,(void *)&gRS232_DMABuffer,1);
		gRS232_CommStruct.lastUpdateTime = HAL_GetTick();
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART6)
	{
		HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_SET);
		UartHandler_Add2Buffer((void *)&gUSB_CommStruct,gUSB_DMABuffer);
		HAL_UART_Receive_IT(gUSB_Uart,(void *)&gUSB_DMABuffer,1);
		gUSB_CommStruct.lastUpdateTime = HAL_GetTick();
	}
    else if(huart->Instance == USART1)
	{
		UartHandler_Add2Buffer((void *)&gRS485_CommStruct,gRS485_DMABuffer);
		HAL_UART_Receive_IT(gRS485_Uart,(void *)&gRS485_DMABuffer,1);
		gRS485_CommStruct.lastUpdateTime = HAL_GetTick();
	}
    else if(huart->Instance == USART2)
	{
		UartHandler_Add2Buffer((void *)&gRS232_CommStruct,gRS232_DMABuffer);
		HAL_UART_Receive_IT(gRS232_Uart,(void *)&gRS232_DMABuffer,1);
		gRS485_CommStruct.lastUpdateTime = HAL_GetTick();
	}
}
void UartHandler_Add2Buffer(tdsCommunicationBuffer* pSource,uint8_t pVal)
{
	pSource->buffer[pSource->writePtr] = pVal;
	pSource->writePtr++;
    
    if(pSource->writePtr >= COMMUNICATION_BUFFER_MAX_SIZE)
	{
		pSource->writePtr=0;
	}
}

int8_t UartHandler_GetFromBuffer(volatile tdsCommunicationBuffer* pSource,uint8_t * pVal)
{
	 uint16_t next;

   if (pSource->writePtr == pSource->readPtr)  // if the head == tail, we don't have any data
        return -1;

   next = pSource->readPtr + 1;  // next is where tail will point to after this read.
   if(next >= COMMUNICATION_BUFFER_MAX_SIZE)
       next = 0;

	 if(NULL != pVal)
	 {
		*pVal =pSource->buffer[pSource->readPtr];  // Read data and then move
   }
	 pSource->readPtr = next;              // tail to next offset.
   return 0;  // return success to indicate successful push.
}


void USB_WriteChar(uint8_t pCh)
{
    HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(gUSB_Uart,&pCh,1,1000);
    HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
}
void USB_Send(uint8_t *pBuff,uint16_t pSize)
{
    HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(gUSB_Uart,pBuff,pSize,1000);
    HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
}
void RS485_WriteChar(uint8_t pCh)
{
    HAL_GPIO_WritePin(RS_EN_GPIO_Port, RS_EN_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(gRS485_Uart,&pCh,1,1000);
    HAL_GPIO_WritePin(RS_EN_GPIO_Port, RS_EN_Pin, GPIO_PIN_RESET);
}
void RS485_Send(uint8_t *pBuff,uint16_t pSize)
{
    HAL_GPIO_WritePin(RS_EN_GPIO_Port, RS_EN_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(gRS485_Uart,pBuff,pSize,1000);
    HAL_GPIO_WritePin(RS_EN_GPIO_Port, RS_EN_Pin, GPIO_PIN_RESET);
}	
void RS232_WriteChar(uint8_t pCh)
{
    HAL_UART_Transmit(gRS232_Uart,&pCh,1,1000);
}
void RS232_Send(uint8_t *pBuff,uint16_t pSize)
{
    HAL_UART_Transmit(gRS232_Uart,pBuff,pSize,1000);
}

void UartHandler_ClearBuffer(tdeUartHandler pUart)
{
	switch(pUart)
	{
		case USB_Uart_Handler:
			memset((void *)gUSB_CommStruct.buffer,0x00,sizeof(gUSB_CommStruct.buffer));
			gUSB_CommStruct.writePtr=0;
			gUSB_CommStruct.readPtr=0;
			break;

        case RS485_Uart_Handler:
			memset((void *)gRS485_CommStruct.buffer,0x00,sizeof(gRS485_CommStruct.buffer));
			gRS485_CommStruct.writePtr=0;
			gRS485_CommStruct.readPtr=0;
			break;
            
        case RS232_Uart_Handler:
			memset((void *)gRS232_CommStruct.buffer,0x00,sizeof(gRS232_CommStruct.buffer));
			gRS232_CommStruct.writePtr=0;
			gRS232_CommStruct.readPtr=0;
			break;
            
		default:break;
	}
}

int8_t UartHandler_GetModbusData(volatile tdsCommunicationBuffer * pBuffer, uint16_t pSize, uint8_t * pData)
{
	uint8_t tVal;
	int8_t retVal = -1;
	
	if(pSize <= abs(pBuffer->writePtr - pBuffer->readPtr))
	{
		uint8_t tStatus = 0;
		for(uint16_t i = 0; i < pSize; i++)
        {
			tStatus |= UartHandler_GetFromBuffer(pBuffer, &tVal);
			pData[i] = tVal;
        }
		
		if(0 == tStatus)
		{
			retVal = 0;
		}
	}
	
	return retVal;
}


void hexDump(char *desc, void *addr, int len) {
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printf ("%s:\r\n", desc);

    // Process every byte in the data.
    for (i = 0; i < len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printf ("  %s\r\n", buff);

            // Output the offset.
            printf ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf (" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf ("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf ("  %s\r\n", buff);
}
