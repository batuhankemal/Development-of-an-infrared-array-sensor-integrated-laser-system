#ifndef __MESSAGEHANDLER_H
#define __MESSAGEHANDLER_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "SerialHandler.h"

extern void HandleMessage(volatile tdsCommunicationBuffer * pBuffer);

#ifdef __cplusplus
}
#endif
#endif /*__MESSAGEHANDLER_H */
