/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SerialHandler.h"
#include "peripherals.h"
#include "MessageHandler.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VREFIN_CAL ((uint16_t*)((uint32_t)0x1FFF7A2A))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ADC_Average_Measurement_Calculator();
float PT500_Real_Temperature_Value_Calculation();
float Voltage_Conversion();
float Current_Conversion();
void Temperature_Fail_Check();
void Short_Circuit_Check();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern  uint32_t gADC_Val[4];



uint16_t gMeasurments_Values[100][3];
uint8_t gMeasurments_counter=0;
uint32_t gMeasurments_avgValue[3];
uint8_t gTemp_State=0;
uint8_t gCurrent_State=0;

float OPAMP_Temperature;
float OPAMP_Voltage;
float OPAMP_Current;

float gTemp;
uint8_t Mode_Flag;
volatile uint8_t gSlope[250];
uint32_t iCounter=0;
uint8_t gCoefficient=0;
uint8_t tPer;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  Init_UartHandler(Init_USB_Uart, &huart6);
  Init_UartHandler(Init_RS485_Uart, &huart1);
  Init_UartHandler(Init_RS232_Uart, &huart2);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tLapTime = 0;
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_RESET);


  HAL_ADC_Start_DMA(&hadc1, gADC_Val, 4);

  HAL_Delay(100);

  uint8_t j;

  for(j=0;j<=100;j++)
  {
	  ADC_Average_Measurement_Calculator();
  }

  while (1)
  {
	  ADC_Average_Measurement_Calculator();
	  OPAMP_Temperature=PT500_Real_Temperature_Value_Calculation();
	  OPAMP_Voltage=Voltage_Conversion();
	  OPAMP_Current=Current_Conversion();

	  IO_check_timeout();
	  HandleMessage(&gUSB_CommStruct);
	  char * ptr = strstr(gRS232_CommStruct.buffer, "Temp:");

	  if(NULL != ptr)
	  	  {
	  		  HAL_Delay(50);
	  		  uint32_t temp = strtol(ptr + 6, NULL, 10);
	  		  gTemp = (float)(((float)temp - 2732.00) / 10.00);
	  		  printf("IR_Temp: %.02f\r\n", gTemp);
	  		  UartHandler_ClearBuffer(RS232_Uart_Handler);
	  		  Laser_Target_Temp_Check();

	  	  }

	  if(HAL_GetTick() - tLapTime > 1000)
	  {
		  printf("Current: %.3f,Output Voltage: %.3f,OPAMP Temp: %.3f Celsius\r\n", OPAMP_Current, OPAMP_Voltage, OPAMP_Temperature);
		  tLapTime = HAL_GetTick();
	  }

	  Temperature_Fail_Check();
	  Short_Circuit_Check();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim6)
  {
	  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_RESET);

	  if(Mode_Flag==1)
      {
		  gSlope[iCounter]=gTemp;

		  iCounter++;

		  if(iCounter>250)
		  {
              if((gSlope[249]-gSlope[0])>=5)
              {
            	  gCoefficient=0;
              }
              else
              {
            	  gCoefficient=30;
              }
			  Mode_Flag=0;
		      iCounter=0;
		  }
      }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // HAL_GPIO_TogglePin(RX_LED_GPIO_Port, RX_LED_Pin);
}

void ADC_Average_Measurement_Calculator()
{
	uint8_t i; uint8_t ADC_Counter; uint32_t New_tSum[3]={0};

	if(gMeasurments_counter>=100)
	{

	gMeasurments_counter=0;

    for(ADC_Counter=0;ADC_Counter<3;ADC_Counter++)
    {
		for(i=0;i<100;i++)
		{

			New_tSum[ADC_Counter]=New_tSum[ADC_Counter]+gMeasurments_Values[i][ADC_Counter];

			gMeasurments_avgValue[ADC_Counter]=New_tSum[ADC_Counter]/(i+1);
		}
    }

	}

	gMeasurments_Values[gMeasurments_counter][0]=gADC_Val[0]; // Current
	gMeasurments_Values[gMeasurments_counter][1]=gADC_Val[1]; // Voltage
	gMeasurments_Values[gMeasurments_counter][2]=gADC_Val[2]; // PT500
	gMeasurments_counter++;

}

float PT500_Real_Temperature_Value_Calculation()
{
	float Temperature_Value=0; float Voltage_Expression=0; float Resistor_Value=0;

	Voltage_Expression=((float)(gMeasurments_avgValue[2])* 3.3320 / 4.0950)/1000.0000;

	Resistor_Value=((Voltage_Expression/71.6000)*110000.0000)/(3.3320-(Voltage_Expression/71.6000));

	Temperature_Value=(Resistor_Value-500.0000)/1.7;

	return Temperature_Value;

}


float Voltage_Conversion()
{
	float Voltage_Expression=0;

	Voltage_Expression=(((float)(gMeasurments_avgValue[1]) * 11.0 * 3.332) / 4095.0);

	return Voltage_Expression;

}

float Current_Conversion()
{
	float Current_Expression=0;

	//VDDA = (float)3.300*( (float)*VREFIN_CAL) /  (float)gADC_Val[3];

	Current_Expression=(((float)gMeasurments_avgValue[0]* 3.332 / 4095.000)/25.000)/0.005;

	return Current_Expression;

}

void Temperature_Fail_Check()

{
	if(PT500_Real_Temperature_Value_Calculation()>=75&&gTemp_State==0)
	{
		DAC_sendCommand(0, 1, 1, 0);
		HAL_Delay(100);
		HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_RESET);
		Buzzer_Effects(2);
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
		printf("Laser shut down, temperature is higher then 110 celsius\r\n");
		gTemp_State=1;
	}
	else if(PT500_Real_Temperature_Value_Calculation()<75)
	{
		gTemp_State=0;
	}

}

void Short_Circuit_Check()
{
	if(gADC_Val[0]>=925&&gCurrent_State==0)
	{
		DAC_sendCommand(0, 1, 1, 0);
		HAL_Delay(100);
		HAL_GPIO_WritePin(ES_MCU_GPIO_Port, ES_MCU_Pin, GPIO_PIN_RESET);
		Buzzer_Effects(2);
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
		printf("Laser shut down, excessive current detected\r\n");
		gCurrent_State=1;
	}
	else if(gADC_Val[0]<925)
	{
		gCurrent_State=0;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
