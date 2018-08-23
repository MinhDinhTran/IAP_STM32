/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "menu.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle1;
UART_HandleTypeDef UartHandle;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern pFunction JumpToApplication;
extern uint32_t JumpAddress;
UART_4 uart4;
uint8_t cnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance==UartHandle.Instance)
		{
				if (uart4.Rx_indx==0) 
				{
						for (uint8_t i=0;i<100;i++) uart4.Rx_Buffer[i]=0;                                        //clear Rx_Buffer before receiving new data 
				}   

				if (uart4.Rx_data[0]!='\n')                                                                  //if received data different from ascii 13 (enter)
				{
						uart4.Rx_Buffer[uart4.Rx_indx++]=uart4.Rx_data[0];                            //add data to Rx_Buffer
				}
				else                                                                                             //if received data = 13
				{
						uart4.Rx_indx=0;
						HAL_UART_Transmit_IT(&UartHandle,(uint8_t *)uart4.Rx_Buffer,sizeof(uart4.Rx_Buffer));
				}   
				HAL_UART_Receive_IT(&UartHandle,(uint8_t *)uart4.Rx_data, 1);
		}
		
		if(huart->Instance==UartHandle1.Instance)
		{
				if (uart4.Rx_indx==0) 
				{
						for (uint8_t i=0;i<100;i++) uart4.Rx_Buffer[i]=0;                                        //clear Rx_Buffer before receiving new data 
				}   

				if (uart4.Rx_data[0]!='\n')                                                                  //if received data different from ascii 13 (enter)
				{
						uart4.Rx_Buffer[uart4.Rx_indx++]=uart4.Rx_data[0];                            //add data to Rx_Buffer
				}
				else                                                                                             //if received data = 13
				{
						uart4.Rx_indx=0;
						HAL_UART_Transmit_IT(&UartHandle1,(uint8_t *)uart4.Rx_Buffer,sizeof(uart4.Rx_Buffer));
				}   
				HAL_UART_Receive_IT(&UartHandle1,(uint8_t *)uart4.Rx_data, 1);
		}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&UartHandle,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&UartHandle1,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&UartHandle1,UART_IT_TC);
	
	HAL_UART_Receive_IT(&UartHandle,(uint8_t *)uart4.Rx_data, 1);
	HAL_UART_Transmit_IT(&UartHandle,(uint8_t *)uart4.Rx_Buffer,sizeof(uart4.Rx_Buffer));
	HAL_UART_Receive_IT(&UartHandle1,(uint8_t *)uart4.Rx_data, 1);
	HAL_UART_Transmit_IT(&UartHandle1,(uint8_t *)uart4.Rx_Buffer,sizeof(uart4.Rx_Buffer));
	
//	if(uart4.Rx_Buffer[0]=='7')
//	{
//			cnt++;
//			for (uint8_t i=0;i<100;i++) uart4.Rx_Buffer[i]=0;   
//	}
//	if(cnt==1)
//	{
//			cnt--;
			FLASH_If_Init();
			Main_Menu ();
//	}
//	else if(uart4.Rx_Buffer[0]=='8')
//	{
//			if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
//			{
//				/* Jump to user application */
//				JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
//				JumpToApplication = (pFunction) JumpAddress;
//				/* Initialize user application's Stack Pointer */
//				__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
//				JumpToApplication();
//			}
////	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//			if(uart4.Rx_Buffer[0]=='7')
//			{
//					cnt++;
//					for (uint8_t i=0;i<100;i++) uart4.Rx_Buffer[i]=0;   
//			}
//			if(cnt==1)
//			{
//					cnt--;
//					FLASH_If_Init();
//					Main_Menu ();
//			}
//			else if(uart4.Rx_Buffer[0]=='8')
//			{
//					if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
//					{
//						/* Jump to user application */
//						JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
//						JumpToApplication = (pFunction) JumpAddress;
//						/* Initialize user application's Stack Pointer */
//						__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
//						JumpToApplication();
//					}
//			}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* UART4 init function */
void MX_UART4_Init(void)
{

  UartHandle1.Instance = UART4;
  UartHandle1.Init.BaudRate = 115200;
  UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle1.Init.StopBits = UART_STOPBITS_1;
  UartHandle1.Init.Parity = UART_PARITY_NONE;
  UartHandle1.Init.Mode = UART_MODE_TX_RX;
  UartHandle1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartHandle1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  UartHandle.Instance = USART1;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartHandle);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
