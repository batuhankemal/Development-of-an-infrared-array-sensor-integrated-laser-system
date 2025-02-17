#include "peripherals.h"
#include "SerialHandler.h"

#define STATUS_LED_TIME 500
#define PMM100_DEVICE 1
//#define PMM30_DEVICE 	1

struct laser laser_time = {0};

volatile int percentage = 0;

tduDAC_data gDAC_Data;
tdsMeasurments gMeasurments;

uint32_t 	gMeasurmentLapTime=0;
uint32_t  tSum=0;
uint8_t		gInitFlag=0;
uint8_t		CNTRL=0;
uint16_t	gMeasurmentCount=0;

uint8_t		Short_Circuit=0;
uint8_t   Short_Circuit_Check_Counter=0;

uint32_t gADC_Val[4];
volatile uint8_t gOverloadDetected = 0;
volatile uint8_t gShortCircuitDetected = 0;

uint32_t tLapTime;

uint32_t tSearchIdTime;


void Init_IOStates()
{
	
	gMeasurmentLapTime=HAL_GetTick();
	gInitFlag=1;
	gMeasurmentCount=0;
	gMeasurments.writePtr=0;
	
	tLapTime = HAL_GetTick();
}


uint8_t Power_Fail(void)
{
	if(gADC_Val[2] <= 500)
	{
		return 1;
	}
	return 0;
}

void DAC_sendCommand(uint8_t pCtrl, uint8_t pGain, uint8_t pShdn, uint16_t pData)
{
	
	gDAC_Data.Data.Port = 0;
	gDAC_Data.Data.Ctrl = pCtrl;
	gDAC_Data.Data.Gain = pGain;
	gDAC_Data.Data.Shdn = pShdn;
	gDAC_Data.Data.Data = pData;

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1,&gDAC_Data.buffer[1],1,100);
	HAL_SPI_Transmit(&hspi1,&gDAC_Data.buffer[0],1,100);
	HAL_Delay(10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(LDAC_GPIO_Port,LDAC_Pin,GPIO_PIN_RESET);
}

void IO_check_timeout(void)
{

	if(laser_time.timeout > 0)
	{
		if(HAL_GetTick() - laser_time.init_time >= laser_time.timeout)
		{
			DAC_sendCommand(0, 1, 1, 0);
			HAL_Delay(100);
			HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
			Buzzer_Effects(2);
			laser_time.timeout = 0;
			laser_time.target_temp = 0;
		}
	}
}

void IO_control_laser(uint8_t command, uint32_t time, uint8_t percentage)
{

	if(time > 0)
	{
		if(time != 0xFFFFFFFF)
		{
			laser_time.init_time = HAL_GetTick();
			laser_time.timeout = time;
			Buzzer_Effects(1);
		}
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);


		DAC_sendCommand(0, 1, 1, (640+(percentage * 1575 / 100)));
		HAL_Delay(100);
		if(command == 0)
		{
			HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_RESET);
			Buzzer_Effects(2);
		}
		else
		{
			HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_SET);
		}
	}
	else if(command == 0)
	{
		HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_RESET);
	    Buzzer_Effects(2);
	    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
	}
}


void Laser_Target_Temp_Check()
{
	extern float gTemp;
	extern volatile uint8_t gCoefficient;
	extern uint8_t tPer;

	if(laser_time.timeout == 0)
	{
		return;
	}

	float temp_difference = (float)((gTemp - laser_time.target_temp));

	// Calculate proportional power adjustment
	percentage = (int)((50+gCoefficient)  - temp_difference * (50-gCoefficient));
	tPer=percentage;
	// Ensure the percentage is within the valid range [0, 100]
	percentage = (percentage > 100) ? 100 : percentage;
	percentage = (percentage < 20) ? 20 : percentage;


	IO_control_laser(1, 0xFFFFFFFF, percentage);


}

void Buzzer_Effects(uint8_t Status)
{
	if(Status==1)
	{
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}

	else if(Status==2)
	{

		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	      HAL_Delay(1000);
	      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	}

	else
	{
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}
}

void Alarm_Functions()
{


}
