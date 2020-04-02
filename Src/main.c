/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <stdio.h>
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define WORKING 1
#define STOP 0
#define PAUSE 2
#define THUAN 1
#define NGHICH 0
#define HIGH 1
#define LOW 0

const float STOPSPEED = 5;
const float MAXSPEED = 6.06; //% or 6.06%
const float pi = 3.1416; 
const int16_t deltaTime = 20;
const float offsetA = 0; // calib here
const float offsetT = -10; // don't change this parameter
const uint16_t sample1 = 800;
const uint16_t sample2 = 1200;
const uint32_t samplePWM2 = 419999;
const uint16_t samplePWM3 = 20999;

uint32_t e, cou;
uint8_t gimbal = STOP;
uint8_t state = STOP;
uint8_t chieuQuay = THUAN;
int32_t suppliedVolt = 0; 
int8_t voltage = 0;
float thetaR = 0;
float alphaR = 0;
float thetaD = 0;
float alphaD = 0;
__IO uint32_t t;
__IO uint8_t m;
__IO uint16_t ADC_Value[4];

float x1 [5] = { 0,
          0,
          0,
          0,
          0};
/*		                                                              
float Ak [5][5] = {-0025.8,    0001.0,    0001.7,         .0,         0,
					0029.6,         .0,    0000.9,    0009.4,         0,
					0001.7,         .0,   -0025.7,    0001.0,         0,
					0084.7,   -3323.1,   -0012.5,   -0533.3,    5119.1,
				   -0000.3,         .0,   -0000.6,   -0000.0,   -0027.5};
          
float Bk [5][3] = {     0  , 25.8188 ,  -1.6601 ,
						 0 ,  14.6838,   -0.8896,
						 0 ,  -1.6601,   25.7339,
						 0 , -84.6924,   12.4950,
					1.4925 ,   0.2951,    0.6406};
          
float Kf [5] = {44.7, 2, -4.47, 0.019, 3.3};
*/

float Ak [5][5] = {   {-90,   1,    0.8,  0,    0},
                      {4.6,  0,    2.9,  6.7,  0},
                      {0.8,  0,    -90,  1,    0},
                      {140, -2373, -71, -533, 5119},
                      {-1.4,  0,    -5.4, 0,    -27.5}};
          
float Bk [5][3] = { {0,   90,  -0.8},
                    {0,   40,  -2.9},
                    {0,   -0.8,  90},
                    {0,   -141, 70.8},
                    {1.49,  1.4, 5.4}};
          
float Kf [5] = { 44.7, 2, -4.47, 0.019, 3.3};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_SYSTICK_Callback(void);
int8_t LQGcalculating(int16_t voltage_U);
void buttonSet(void);
void GetAnglePotentio (void);
void SendAngles(void);
void pauseGimbal(void);
void controlDC(int8_t voltage_U);
void calibrationForGimbal();
void startBLDC(void);
void stopBLDC(void);
void QuayNghich(int8_t voltage_U);
void QuayThuan(int8_t voltage_U);
void Delay (uint32_t count);

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  /***************************************************************************/
  // PWM configuration
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_4);
  
    // ESC 5% -> 10% PWM 50Hz
    // 140rps -> 6.0%
    TIM2->CCR1 = (int)(STOPSPEED*samplePWM2/100); // 5%
    TIM3->CCR2 = 0*samplePWM3/100; // 0%
    TIM3->CCR4 = 0*samplePWM3/100; // 0%

    // ADC configuration
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 4);

    // This Blue led will be used for comparison of ADC sampling process
    // It should be brighter than when MCU sampling ( right after 0.1s )
    // It should be checked by oxiloscope with duty cycle smaller than 70%
    HAL_GPIO_WritePin(GPIOD, LEDblue_Pin, GPIO_PIN_SET);
    Delay(300);
    HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_SET);
    Delay(300);
    HAL_GPIO_WritePin(GPIOD, LEDorange_Pin, GPIO_PIN_SET);
    Delay(300);
    HAL_GPIO_WritePin(GPIOD, LEDred_Pin, GPIO_PIN_SET);
    Delay(500);
    HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin | LEDorange_Pin | LEDred_Pin | LEDblue_Pin, GPIO_PIN_SET);
    Delay(100);
    /**************************************************/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /**************************************************/
    while (1){
        if ( state == STOP ){
            calibrationForGimbal();
            
            // Green Led will be used for adjustment of theta
            if ( thetaD < 0.2 && thetaD > -0.2 )
                HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_RESET);
        }
        buttonSet();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /***************************************************************************/
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 419999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin|LEDorange_Pin|LEDred_Pin|LEDblue_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GyroBT1_Pin */
  GPIO_InitStruct.Pin = GyroBT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GyroBT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDgreen_Pin LEDorange_Pin LEDred_Pin LEDblue_Pin */
  GPIO_InitStruct.Pin = LEDgreen_Pin|LEDorange_Pin|LEDred_Pin|LEDblue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

/*****************************************************************************/

void HAL_SYSTICK_Callback(void){
    if ( t++ > 1000){
        t = 0;
        // This Red Led will change every one second
        // It shows that this working frequency is right
        HAL_GPIO_TogglePin(GPIOD, LEDred_Pin);
    }
    if ( state == WORKING) {
      if (t == 500 || t == 0){
         HAL_GPIO_TogglePin(GPIOD, LEDgreen_Pin);
      }
    }
    
    // Send data via UART every 10ms
    if (++m > 10){
        SendAngles();
        m = 0;
    }

    // The blue led will change with frequency is 1kHz
    // If this led does not bright at maximum intensity,
    // It means the ADC sample quantity is acceptable
    // If this led does not bright, keep increase ADC sample
    // You should measure Blue LED signal on Oxiloscope, it should be < 70%
    HAL_GPIO_WritePin(GPIOD, LEDblue_Pin, GPIO_PIN_SET);
    
    // This code won't work when gimbal is paused
    if ( gimbal != PAUSE ) {
        // The MCU will read adc value every 1ms
        GetAnglePotentio();
        voltage = LQGcalculating(voltage);
    }
    if ( state == WORKING && gimbal == WORKING ){
        controlDC(voltage);
    }
    HAL_GPIO_WritePin(GPIOD, LEDblue_Pin, GPIO_PIN_RESET);
}

int8_t LQGcalculating(int16_t voltage_U){
  
  float u1 [3] ={voltage_U, thetaR, alphaR};
  float xDot [5] = {0,0,0,0,0};  
  float v1 [5] = {0,0,0,0,0};
  float z1 [5] = {0,0,0,0,0};  
  float k1 [5] = {0,0,0,0,0};
  int16_t CalculatedV = 0;

  for (int i = 0; i < 5; i++){
    for (int j = 0; j < 3; j++){
      z1[i] += Bk[i][j] * u1[j];
    }
    for (int j = 0; j < 5; j++){
      v1[i] += Ak[i][j] * x1[j];
    }
    xDot[i] = v1[i] + z1[i];
    k1[i] = xDot[i]/1000 + x1[i]; // delta time = 0.001s (=/1000)
    x1[i] = k1[i];
    CalculatedV += (int)((-1)*Kf[i] * k1[i]);
  }

  if ( CalculatedV > suppliedVolt ) return suppliedVolt;
  if ( CalculatedV < -suppliedVolt ) return -suppliedVolt;
  if ( CalculatedV > 1.5 && CalculatedV <= 3 ) return 3;
  if ( CalculatedV < -1.5 && CalculatedV >= -3 ) return -3;

  return CalculatedV;
}

void buttonSet(void){
  if (HAL_GPIO_ReadPin(GPIOC,GyroBT1_Pin) == HIGH) {
    while (HAL_GPIO_ReadPin(GPIOC,GyroBT1_Pin) == HIGH){
      Delay(10);
    }
    if (state == WORKING){
      state = STOP;
      stopBLDC();
      calibrationForGimbal();
    }
    else{
      calibrationForGimbal();
      startBLDC();
    }
  }
}

void GetAnglePotentio (void){
  // Measure supplied voltage and calirating theta
  float calibTheta = 0;
  for (int i = 0; i < sample1; i++){
    suppliedVolt += ADC_Value[2];
    calibTheta += ADC_Value[3];
  }
  suppliedVolt = (int)(suppliedVolt/sample1*6*3.3/4096);
  calibTheta = calibTheta/sample1/200;       // 4096/200 = 20; offset -10: -10+20=10 ; -10 to 10
  
  // 3240 = 90
  // 872 = -90
  // Adc90*a + 1 = 90
  // Adc-90*a + 1 = -90
  float alpha1 = 0;
  float theta1 = 0;
  for (int i = 0; i < sample2; i++){
    alpha1 += ADC_Value[0];
    theta1 += ADC_Value[1];
  }
  alphaD = alpha1/sample2*0.0835 - 157.5 + offsetA;
  thetaD = theta1/sample2*0.0835 - 157.5 + offsetT + calibTheta;
  alphaR = alphaD*pi/180;
  thetaR = thetaD*pi/180;
}

void SendAngles(){
    if (gimbal == STOP){
        thetaD = 0;
        alphaD = 0;
        voltage = 0;
    }
    printf("%4.1f\t%4.1f\t%4.1f\t%4.1f\t%d \n", thetaD, alphaD, x1[0]*180/pi, x1[2]*180/pi, voltage);
}

void pauseGimbal (){
  gimbal = PAUSE;
  if ( chieuQuay == NGHICH ){
      QuayThuan(suppliedVolt);
    }
    else{
      QuayNghich(suppliedVolt);
    }
  Delay(50);
  QuayNghich(0);
  gimbal = STOP;
}

// Input is voltage: negative or positive
void controlDC (int8_t voltage_U){
  if ( alphaD > -80 && alphaD < 80){
    if ( voltage_U > 0 ){
      QuayNghich(voltage_U);
    }
    else {
      QuayThuan(-voltage_U);
    }
  }
  else{
     gimbal = STOP;
     HAL_GPIO_WritePin(GPIOD, LEDorange_Pin, GPIO_PIN_SET);
     pauseGimbal();
     calibrationForGimbal();
     QuayNghich(0);
     Delay(1000);
     HAL_GPIO_WritePin(GPIOD, LEDorange_Pin, GPIO_PIN_RESET);
     gimbal = WORKING;
  } 
}

// This function uses for delaay with milisecond 
// The variable e will decrease every 10 clock cycles
void Delay (uint32_t count){
  while(--count > 1){
    uint32_t e = 16800;
    while(--e > 1);
  }
}

void calibrationForGimbal(){
  QuayThuan(0);
  while ( alphaD > 6 || alphaD < -6){
    HAL_GPIO_WritePin(GPIOD, LEDorange_Pin, GPIO_PIN_SET);
    GetAnglePotentio();
    if ( alphaD > 0 ){
      QuayThuan(4);
    }
    else{
      QuayNghich(4);
    }
    Delay(10);
    QuayNghich(0);
  }
  HAL_GPIO_WritePin(GPIOD, LEDorange_Pin, GPIO_PIN_RESET);
}

void startBLDC (void){
  TIM2->CCR1 = (int)(MAXSPEED*samplePWM2/100);
  
  HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_SET);
  Delay(1000);
  
  HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_RESET);
  Delay(29000);
  
  HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_SET);
  state = WORKING;
  gimbal = WORKING;
}

void stopBLDC (void){
    QuayNghich(0);
	gimbal = STOP;
    TIM2->CCR1 = (int)(STOPSPEED*samplePWM2/100); // 5%
    
    HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_RESET);
    Delay(4000);
    
    HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_SET);
    Delay(1000);
    HAL_GPIO_WritePin(GPIOD, LEDgreen_Pin, GPIO_PIN_RESET);
}

void QuayNghich (int8_t voltage_U){
  chieuQuay = NGHICH;
  TIM3->CCR4 = (unsigned int)samplePWM3*voltage_U/suppliedVolt;
  TIM3->CCR2 = 0;
}

void QuayThuan (int8_t voltage_U){
  chieuQuay = THUAN;
  TIM3->CCR2 = (unsigned int)samplePWM3*voltage_U/suppliedVolt;
  TIM3->CCR4 = 0;
}
/*****************************************************************************/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
