/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "hwdrv.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t cmd[TXNUM] = { '\0' };
uint16_t ADCBuffer[ARRAYSIZE];
uint16_t x_coord, y_coord;
DATA_TypeDef data;
int16_t tmp_l, tmp_r;
__IO uint8_t drvl = 0x40;
__IO uint8_t drvr = 0x40;
__IO uint8_t tmp_drvl = 0;
__IO uint8_t tmp_drvr = 0;
__IO uint8_t flg = 0;
__IO uint8_t response_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *) &ADCBuffer, ARRAYSIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    HAL_Delay(50);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    
    data.currentStatus |= Status_OK;
    
    if (data.currentStatus & Status_AUTO)
    {
        if ((y_coord > 3000) && ((x_coord > 1900) && (x_coord < 2200))) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_FORWARD; 
        }
        else if ((y_coord < 1000) && ((x_coord > 1900) && (x_coord < 2100))) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_BACK;
        }
        else if ((x_coord > 3000)  && ((y_coord > 1900) && (y_coord < 2100))) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_RIGHT;
        }
        else if ((x_coord < 1000)  && ((y_coord > 1800) && (y_coord < 2100))) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_LEFT;
        }
        else if ((x_coord > 4000)  && (y_coord > 4000)) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_SLEFT;
        }
        else if ((x_coord < 50)  && (y_coord < 50)) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_SBRIGHT;
        }
        else if ((x_coord < 50)  && (y_coord > 4000)) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_SRIGHT;
        }
        else if ((x_coord > 4000)  && (y_coord < 50)) 
        { 
            data.currentStatus &= 0xF0; data.currentStatus |= Status_SBLEFT;
        }
        cmd[1] = data.currentStatus;
    }
    else
    {
        
        tmp_l = tmp_r = y_coord;
        
        if (x_coord >= MIDVAL)
        {
            tmp_l = y_coord + (x_coord - MIDVAL);
            tmp_r = y_coord - (x_coord - MIDVAL);
        }
        else if (x_coord < MIDVAL)
        {
            tmp_l = y_coord - (MIDVAL - x_coord);
            tmp_r = y_coord + (MIDVAL - x_coord);
        }
        if (tmp_l > MAXVAL) tmp_l = MAXVAL;
        if (tmp_r > MAXVAL) tmp_r = MAXVAL;

        if (tmp_l < 0) tmp_l = 0;
        if (tmp_r < 0) tmp_r = 0;
        drvl = tmp_l * MAXPWM / MAXVAL;
        drvr = tmp_r * MAXPWM / MAXVAL;

        data.currentStatus &= 0xF0;
        
    }
    
    /* Users buttons */
    if (!HAL_GPIO_ReadPin(BTN_D3_GPIO_Port, BTN_D3_Pin)) 
    {
      data.armStatus = ARM_CMD_RELEASE;
    } 
    else if (!HAL_GPIO_ReadPin(BTN_D4_GPIO_Port, BTN_D4_Pin))
    {
      data.armStatus = ARM_CMD_DOWN; 
    }
    else if (!HAL_GPIO_ReadPin(BTN_D5_GPIO_Port, BTN_D5_Pin))
    {
      data.armStatus = ARM_CMD_UP;
    }
    else if (!HAL_GPIO_ReadPin(BTN_D6_GPIO_Port, BTN_D6_Pin))
    {
      data.armStatus = ARM_CMD_GRAB;
    }
    else
    {
      data.armStatus = ARM_CMD_NO;
    }
    
    cmd[0] = STARTMARKER;
    cmd[1] = data.currentStatus;
    cmd[2] = drvl;
    cmd[3] = drvr;
    cmd[4] = data.armStatus;
    cmd[5] = STOPMARKER;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)cmd, TXNUM);
    HAL_ADC_Start_DMA(&hadc, (uint32_t *) &ADCBuffer, ARRAYSIZE);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_D6_Pin BTN_D3_Pin BTN_D5_Pin BTN_D4_Pin */
  GPIO_InitStruct.Pin = BTN_D6_Pin|BTN_D3_Pin|BTN_D5_Pin|BTN_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_AUTO_Pin */
  GPIO_InitStruct.Pin = BTN_AUTO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_AUTO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  x_coord = (ADCBuffer[0] + ADCBuffer[2] + ADCBuffer[4] + ADCBuffer[6] + ADCBuffer[8]) / 5;
  y_coord = (ADCBuffer[1] + ADCBuffer[3] + ADCBuffer[5] + ADCBuffer[7] + ADCBuffer[9]) / 5;
  if ((x_coord > 1800)&(x_coord < 2200)) x_coord = MIDVAL;
  if ((y_coord > 1800)&(y_coord < 2200)) y_coord = MIDVAL;
  HAL_ADC_Stop_DMA(hadc);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (data.currentStatus & Status_AUTO)
    data.currentStatus &= ~(Status_AUTO);
  else
    data.currentStatus |= Status_AUTO;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
