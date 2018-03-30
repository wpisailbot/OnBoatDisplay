
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "epd2in9b.h"
#include "epdif.h"
#include "epdpaint.h"
#include "imagedata.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define COLORED      1
#define UNCOLORED    0

uint8_t ubKeyNumber = 0x0;
CAN_HandleTypeDef    CanHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CAN_Config(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile unsigned long ticks;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_CAN_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	CAN_Config();

	__HAL_CAN_CLEAR_FLAG(&CanHandle, CAN_FLAG_WKU);


	/* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
	unsigned char* frame_buffer_black = (unsigned char*)malloc(EPD_WIDTH * EPD_HEIGHT / 8);
	unsigned char* frame_buffer_red = (unsigned char*)malloc(EPD_WIDTH * EPD_HEIGHT / 8);
	EPD epd;
	if (EPD_Init(&epd) != 0) {
		//printf("e-Paper init failed\n");
		return -1;
	}

	Paint paint_black;
	Paint paint_red;
	Paint_Init(&paint_black, frame_buffer_black, epd.width, epd.height);
	Paint_Init(&paint_red, frame_buffer_red, epd.width, epd.height);
	Paint_Clear(&paint_black, UNCOLORED);
	Paint_Clear(&paint_red, UNCOLORED);

	/* Draw something to the frame buffer */
	/* For simplicity, the arguments are explicit numerical coordinates */
	Paint_SetRotate(&paint_black, ROTATE_90);

	/*Write strings to the buffer */
	Paint_DrawStringAt(&paint_black, 36, 30, "Boat booting", &Font24, COLORED);
	Paint_DrawStringAt(&paint_black, 50, 64, "Please wait", &Font24, COLORED);

	/* Display the frame_buffer */
	EPD_DisplayFrame(&epd, frame_buffer_black, frame_buffer_red);




	HAL_Delay(5000); // Will replace with wait for rx of specific CAN message or some other signal from BBB




	Paint_Clear(&paint_black, UNCOLORED);
	Paint_DrawStringAt(&paint_black, 40, 30, "Boat booted", &Font24, COLORED);
	EPD_DisplayFrame(&epd, frame_buffer_black, frame_buffer_red);

	CanHandle.pTxMsg->StdId = 0x123;
	CanHandle.pTxMsg->RTR = 0;
	CanHandle.pTxMsg->DLC = 3;
	CanHandle.pTxMsg->Data[0] = 0;
	CanHandle.pTxMsg->Data[1] = 0;
	CanHandle.pTxMsg->Data[2] = 0;

	HAL_ADC_Start(&hadc2);

	int idx = 0;
	int ADCValue = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_ADC_Start(&hadc2);
		HAL_StatusTypeDef adcStatus = HAL_ADC_PollForConversion(&hadc2, 1000);
		if (adcStatus == HAL_OK)
		{
			ADCValue = HAL_ADC_GetValue(&hadc2);
			CanHandle.pTxMsg->Data[0] = ADCValue>>8;
			CanHandle.pTxMsg->Data[1] = ADCValue;
			CanHandle.pTxMsg->Data[2] = (int)adcStatus;
			idx++;
		} else {
			CanHandle.pTxMsg->Data[0] = 0;
			CanHandle.pTxMsg->Data[1] = 0;
			CanHandle.pTxMsg->Data[2] = (int)adcStatus;
		}

		if (HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)
		{
			/* Transmition Error */
			Error_Handler();
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_Delay(100);
		}

		float voltage = (float)ADCValue/4096.0*13.1;

		char voltageBuf[6];
		sprintf(voltageBuf, "%d.%dv",  (int)voltage, (int)(voltage*10)%10);

		Paint_Clear(&paint_black, UNCOLORED);
		Paint_DrawStringAt(&paint_black, 40, 30, voltageBuf, &Font24, COLORED);
		EPD_DisplayFrame(&epd, frame_buffer_black, frame_buffer_red);
		HAL_Delay(10000);

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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback(void)
{
	ticks++;
}

static void CAN_Config(void)
{
	CAN_FilterConfTypeDef  sFilterConfig;
	static CanTxMsgTypeDef        TxMessage;
	static CanRxMsgTypeDef        RxMessage;

	//__HAL_CAN_DBG_FREEZE(&CanHandle, DISABLE); // allow CAN peripheral to continue to run while debugger is halted

	/*##-1- Configure the CAN peripheral #######################################*/
	CanHandle.Instance = CAN1;
	CanHandle.pTxMsg = &TxMessage;
	CanHandle.pRxMsg = &RxMessage;

	CanHandle.Init.TTCM = DISABLE;
	CanHandle.Init.ABOM = DISABLE;
	CanHandle.Init.AWUM = DISABLE;
	CanHandle.Init.NART = ENABLE; // change back
	CanHandle.Init.RFLM = DISABLE;
	CanHandle.Init.TXFP = DISABLE;
	CanHandle.Init.Mode = CAN_MODE_NORMAL;
	CanHandle.Init.SJW = CAN_SJW_1TQ;
	CanHandle.Init.BS1 = CAN_BS1_13TQ;
	CanHandle.Init.BS2 = CAN_BS2_2TQ;
	CanHandle.Init.Prescaler = 2;

	if (HAL_CAN_Init(&CanHandle) != HAL_OK)
	{

		/* Initiliazation Error */
		Error_Handler();
	}

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 0;

	if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
