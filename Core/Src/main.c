/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * @authors        : Rodrigo De Lama Fernández @rodrigodelama
  *                   Manuel Morales Niño @ikaoseu
  *                   Jaime Mato Rodriguez @Pekeniojimi
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l152c_discovery.h"
#include "stm32l152c_discovery_glass_lcd.h"
#include "SDM_Utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;

LCD_HandleTypeDef hlcd;

/* USER CODE BEGIN PV */
//GLOBAL VARS
int game = 1;
int winner = 0; //Init to 0, if it never changes, it will generate an error
//long 3secs = 3000000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//INTERRUPTS
void EXTI0_IRQHandler(void)
{
  if ((EXTI->PR&0x01) != 0) // Is EXTI0 flag on? //0000000000000001 in the Pending Register
  {                         // USER BUTTON is pressed, a rising edge is detected in PA0
    game++; // Increase the count
    if (game > 2) game = 1; //Reset to 1 when game surpases 2
    EXTI->PR |= (1 << 6); // Clear EXTI0 flag (writes a 1 in PR0 pos)
  }
}
//TODO:
//how do we determine which button is pressed first in the main section
//the interrupt will obviously be executed first, but how do we discard the second player
void EXTI1_IRQHandler(void) //ISR for EXTI1 - Edge detection for BUTTON 1
{
  if ((EXTI->PR&0x02) != 0) //0000000000000010 in binary
  {                         // BUTTON 1 is pressed, a rising edge is detected in PA11

    winner = 1;
    EXTI->PR |= (1 << 7); // Clear the EXTI1 flag (writes a 1 in PR1)
  }
}
void EXTI2_IRQHandler(void) //ISR for EXTI2 - Edge detection for BUTTON 2
{
  if ((EXTI->PR&0x04) != 0) //0000000000000100 in binary
  {                         // BUTTON 2 is pressed, a rising edge is detected in PA12
    winner = 2;
    EXTI->PR |= (1 << 8); // Clears the EXTI2 flag (writes a 1 in PR2)
  }
}

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
  MX_ADC_Init();
  MX_LCD_Init();
  MX_TS_Init();
  /* USER CODE BEGIN 2 */
  //DECLARATION OF PERIPHERALS WE WILL USE

  //LCD Setup
  BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_BarLevelConfig(0);
  BSP_LCD_GLASS_Clear();

  //PAs + their EXTIs - I/O
  //PA0 (USER BUTTON) - digital input (00)
  GPIOA->MODER &= ~(1 << (0*2 + 1));
  GPIOA->MODER &= ~(1 << (0*2));
  //EXTI0
  EXTI->RTSR |= 0x01; // Enables rising edge in EXTI0
  EXTI->FTSR &= ~(0x01); // Disables falling edge in EXTI0
  SYSCFG->EXTICR[0] = 0; // EXTI0 is linked to GPIOA (USER BUTTON = PA0) - all zeros mean GPIOA
  EXTI->IMR |= 0x01; // Enables the Interrupt (i.e. the event) (IMR = Interrupt Mask Register)
  NVIC->ISER[0] |= (1 << 6); //Enables EXTI0 in NVIC (pos 6)

  //USING UNUSED I/O PINS 11 and 12
  //PA11 (BUTTON 1) - digital input (00)
  GPIOA->MODER &= ~(1 << (11*2 + 1));
  GPIOA->MODER &= ~(1 << (11*2));
  //WE NEED INTERNAL RESISTORS - pull-up OR pull-down ????
  //Pull-Up: should be a constant 0, unless we press, then it should change to a 1
  //Set up with pull-up resistor (01)
  GPIOA->PUPDR |= (1 << (11*2));
  GPIOA->PUPDR &= ~(1 << (11*2 + 1));
  //EXTI1
  EXTI->RTSR |= 0x02; // Enables rising edge in EXTI1
  EXTI->FTSR &= ~(0x02); // Disables falling edge in EXTI1
  SYSCFG->EXTICR[0] = 0; // EXTI2 is linked to GPIOA (BUTTON 1 = PA11) - TODO: DOUBLE CHECK
  EXTI->IMR |= 0x02; // Enables the interrupt
  NVIC->ISER[0] |= (1 << 7);

  //PA12 (BUTTON 2) - digital input (00)
  GPIOA->MODER &= ~(1 << (12*2 + 1));
  GPIOA->MODER &= ~(1 << (12*2));
  //Set up with pull-up resistor (01)
  GPIOA->PUPDR |= (1 << (12*2));
  GPIOA->PUPDR &= ~(1 << (12*2 + 1));
  //EXTI2
  EXTI->RTSR |= 0x04; // Enables rising edge in EXTI2
  EXTI->FTSR &= ~(0x04); // Disables falling edge in EXTI2
  SYSCFG->EXTICR[0] = 0; // EXTI2 is linked to GPIOA (BUTTON 2 = PA12) - TODO: DOUBLE CHECK
  EXTI->IMR |= 0x04; // Enables the interrupt
  NVIC->ISER[0] |= (1 << 8);

  //LEDs
  //PB7 (GREEN LED) - digital output (01)
  GPIOB->MODER &= ~(1 << (7*2 + 1));
  GPIOB->MODER |= (1 << (7*2));
  //PB6 (BLUE LED) - digital output (01) - ERROR LED
  GPIOB->MODER &= ~(1 << (6*2 + 1));
  GPIOB->MODER |= (1 << (6*2));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //display GAME 1 (initially) --> DONE IN GLOBAL VAR DECLARATIOn
    //if USER BUTTON is pressed, change to GAME 2 (at ANY time - use interrupts)
    //else wait predetermined time and start (use espera() function)

    switch(game)
    {
      case 1: // Game 1
        BSP_LCD_GLASS_Clear(); //FIXME: Not sure how important it is to clear before writing
        BSP_LCD_GLASS_DisplayString((uint8_t*)" GAME1");
        espera(10000000); //shall be random in milestone 2
        //Light up GREEN LED
        GPIOB->BSRR = (1 << 7);
        espera(3000000);
        //Interrupts will determine which winner it is
        switch(winner)
        {
          case 1:
            GPIOB->BSRR = (1 << 7) << 16; //GREEN LED OFF
            BSP_LCD_GLASS_Clear(); //FIXME: see comment in Game 1
            BSP_LCD_GLASS_DisplayString((uint8_t*)" P1 W");
            break;
          
          case 2:
            GPIOB->BSRR = (1 << 7) << 16; //GREEN LED OFF
            BSP_LCD_GLASS_Clear(); //FIXME:
            BSP_LCD_GLASS_DisplayString((uint8_t*)" P2 W");
            break;
          
          default:
            GPIOB->BSRR = (1 << 7) << 16; //GREEN LED OFF
            GPIOB->BSRR = (1 << 6); //Turn on the BLUE LED signalling an error
            BSP_LCD_GLASS_Clear(); //FIXME:
            BSP_LCD_GLASS_DisplayString((uint8_t*)" 2slow");
            espera(3000000);
            //BSP_LCD_GLASS_Clear(); //FIXME:
           // BSP_LCD_GLASS_DisplayString((uint8_t*)" RESET");
            break;
        }

        BSP_LCD_GLASS_Clear();
        break;
      case 2: // Game 2
        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_DisplayString((uint8_t*)" GAME2");
        //TODO:
        //Game 2 will be done at a later milestone

        BSP_LCD_GLASS_Clear();
        break;
      default:
        GPIOB->BSRR = (1 << 7) << 16; //GREEN LED OFF
        GPIOB->BSRR = (1 << 6); //Turn on the BLUE LED signalling an error
        BSP_LCD_GLASS_Clear(); //FIXME:
        BSP_LCD_GLASS_DisplayString((uint8_t*)" ERROR");
        espera(3000000);
        BSP_LCD_GLASS_Clear(); //FIXME:
        BSP_LCD_GLASS_DisplayString((uint8_t*)" RESET");
        break;
    }
    return 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LCD;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
