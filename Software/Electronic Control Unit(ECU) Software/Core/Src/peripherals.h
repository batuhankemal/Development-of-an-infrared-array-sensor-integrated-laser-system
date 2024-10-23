#ifndef __PERIPHERALS_H
#define __PERIPHERALS_H

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "spi.h"
#include "adc.h"

extern struct laser laser_time;

#define MEASURMENT_COUNT 100
typedef struct 
{
	uint32_t Values[MEASURMENT_COUNT];
	uint32_t  writePtr;
	uint32_t avgValue;
}tdsMeasurments;
typedef enum
{
	MUX_RS485=1,
	MUX_USB=2,
	MUX_ETHERNET=3,
	MUX_REPEATER=4
}tdeMuxSelect;

typedef enum
{
	COM_NONE=0,
	COM_USB=1,
	COM_RS232=2,
	COM_RPTR=3
}tdeComType;

typedef struct 
{
	
	uint16_t Data   :12;
	uint16_t Shdn   :1;
	uint16_t Gain   :1;
	uint16_t Ctrl   :1;	
	uint16_t Port		:1;
}tdsDAC_data;

typedef union
{
	tdsDAC_data Data;
	uint8_t buffer[sizeof(tdsDAC_data)];
}tduDAC_data;

struct laser
{
	uint32_t init_time;
	uint32_t timeout;
	float target_temp;
};

extern uint32_t tSearchIdTime;
extern uint8_t comType;
extern tdsMeasurments gMeasurments;
extern tduDAC_data gDAC_Data;
extern uint32_t tEchoTime;
extern uint32_t tLapTime;
extern uint32_t tAuthTime;
void Init_IOStates(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Laser_Target_Temp_Check();


extern uint8_t Power_Fail(void);
extern void Peripheral_Handle(void);
extern void DAC_sendCommand(uint8_t pCtrl, uint8_t pGain, uint8_t pShdn, uint16_t pData);
void Buzzer_Effects(uint8_t Status);

extern uint32_t gADC_Val[4];
extern volatile uint8_t gOverloadDetected;
extern volatile uint8_t gShortCircuitDetected;




void IO_control_laser(uint8_t command, uint32_t time, uint8_t percentage);
void IO_check_timeout(void);


#endif /*__PERIPHERALS_H*/
