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
  * @repo           : https://github.com/rodrigodelama/LAB_1
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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
//GLOBAL VARS & DEFINITIONS
//game vars
unsigned int prev_game = 0;
unsigned int game = 1; //game 1 is the starting point
unsigned int winner = 0; //Init to 0, if it never changes, it will generate an error
unsigned int playing = 0; // to not activate the interrupts unless we are awaiting

//timer vars
int countdown[] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
//unsigned int game1_rand;
unsigned int diff;
unsigned int diff1;
unsigned int diff2;
char p1w[10];

/* ideas for timer variables
unsigned int led_timer = sec; //minimum 1 sec
unsigned short init_time = 0;
unsigned int time1;
unsigned int time_taken = 0;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//INTERRUPTS
void EXTI0_IRQHandler(void)
{
  // USER BUTTON is pressed, a rising edge is detected in PA0
  if ((EXTI->PR&BIT_0) != 0) // Is EXTI0 flag on? //0000000000000001 in the Pending Register of ISER[0]
  {
	  game++; //If at GAME 1, proceed to GAME 2
	  if (game > 2) game = 1; //Reset to 1 after requesting change from GAME 2
  }
  EXTI->PR |= (1 << 6); // Clear EXTI0 flag (writes a 1 in PR0 pos)
  //Clear flag must be out of condition so it never hangs if condition is not met
}

// NVIC->ISER[0] pin 23 for EXTIs 9 to 5
void EXTI9_5_IRQHandler(void) //ISR for EXTI7 & EXTI6
{
  if (((EXTI->PR&BIT_7) || (EXTI->PR&BIT_6)) != 0) //most general aproach --> (EXTI->PR != 0)
  {
    // BUTTON 1 is pressed, a rising edge is detected in PB7
    if (EXTI->PR & (1 << 7) && (playing == 1)) // 00000000010000000 in pending register of ISER[0]
    {
      winner = 1;
    }
    // BUTTON 2 is pressed, a rising edge is detected in PB6
    if (EXTI->PR & (1 << 6) && (playing == 1)) // 00000000001000000 in pending register
    {
      winner = 2;
    }
  }
  //Always clear flags to avoid hanging
  EXTI->PR |= (1 << 7); // Clear the EXTI7 flag (writes a 1 in PR7)
  EXTI->PR |= (1 << 6); // Clear the EXTI6 flag
}

//TIMERS
//timers TOC timer 3 ch4
void TIM3_IRQHandler(void)
{
/**
 * @brief 
 *             //here?
            TIM3->CCR = random_num(0, 10000);
 */
  //we possibly should not care, just use ch1 for game 1, ch2 for game 2
  /*
  if ((TIM3->SR & BIT_1) != 0)
  {
    switch (game)
    {
    case 1:
      //for led shutoff
      //game1_rand = random_num(0, 10000);
      //TIM3->CCR1 = game1_rand;
      TIM3->CCR1 = random_num(0, 1000000);
    break;

    case 2:
      //TIM3->CCR1 from 0 to 10000
    break;
    
    default:
      break;
    }
  }
*/
}
//timers TIC timer 4 ch1 and ch2
void TIM4_IRQHandler(void) //TIC
{
  //checks if it has been activated if so makes it work
//checks if the SR has been flagged or if its
//checks if the event has come
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
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //DECLARATION OF PERIPHERALS WE WILL USE

  //LCD Setup
  BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_BarLevelConfig(0);
  BSP_LCD_GLASS_Clear();

  //Pins + their EXTIs - I/O
  /* PB0 ---------------------------------------------------------------------*/
  //PA0 (USER BUTTON) - digital input (00)
  GPIOA->MODER &= ~(1 << (0*2 + 1));
  GPIOA->MODER &= ~(1 << (0*2));
  //EXTI0
  SYSCFG->EXTICR[0] = 0000; // Linking EXTI0 to GPIOA (PA0 = USER BUTTON) - all zeros mean GPIOA
  EXTI->RTSR |= BIT_0; // Enables rising edge in EXTI0
  EXTI->FTSR &= ~(BIT_0); // Disables falling edge in EXTI0
  EXTI->IMR |= BIT_0; // Enables the Interrupt (i.e. the event) (IMR = Interrupt Mask Register)
  NVIC->ISER[0] |= (1 << 6); //Enables EXTI0 in the NVICs ISER[0]s position 6
  
  //USING I/O PINS 7 & 6 FOR PLAYER BUTTONS 1 & 2 RESPECTIVELY
  /* PB7 ---------------------------------------------------------------------*/
  //PB7 (BUTTON 1) - digital input (00)
  //Set PB7 to 10 for Alternate Functions (AFs)
  GPIOB->MODER |= (1 << (7*2 + 1));
  GPIOB->MODER &= ~(1 << (7*2));
  //AF for TIM4_CH2
  GPIOB->AFR[0] |= (0x02 << (7*4)); // Writes 0010 in AFRL7
  //WE NEED INTERNAL RESISTORS - pull-up OR pull-down ?
  //We chose pull-up: a constant 1 unless we press, then its shorted to GND
  //Set up with pull-up resistor (01)
  GPIOB->PUPDR &= ~(1 << (7*2 + 1));
  GPIOB->PUPDR |= (1 << (7*2));
  //EXTI7
  SYSCFG->EXTICR[1] |= BIT_12; // Linking EXTI7 to GPIOB (PB7 = BUTTON 1)
                    // Sets a 1 in bit 12 see page 145 of the manual
  EXTI->RTSR &= ~(BIT_7); // Disables rising edge in EXTI7
  EXTI->FTSR |= BIT_7; // Enables falling edge in EXTI7
  EXTI->IMR |= BIT_7; // Enables the interrupt
  NVIC->ISER[0] |= (1 << 23); // EXTI7 has position 23 in ISER[0]

  /* PB6 ---------------------------------------------------------------------*/
  //PB6 (BUTTON 2) - digital input (00)
  //TIM4_CH1
  //Set PB6 to 10 for AFs
  GPIOB->MODER |= (1 << (6*2 + 1));
  GPIOB->MODER &= ~(1 << (6*2));
  //Set up with pull-up resistor (01)
  GPIOB->PUPDR &= ~(1 << (6*2 + 1));
  GPIOB->PUPDR |= (1 << (6*2));
  //AF for TIM4_CH1
  GPIOB->AFR[0] |= (0x02 << (6*4)); // Writes 0010 in AFRL6
  //EXTI6
  SYSCFG->EXTICR[1] |= BIT_8; // Linking EXTI6 to GPIOB (PB6 = BUTTON 2)
                    // Sets a 1 in bit 8 see page 145 of the manual
  EXTI->RTSR &= ~(BIT_6); // Disables rising edge in EXTI6
  EXTI->FTSR |= BIT_6; // Enables falling edge in EXTI6
  EXTI->IMR |= BIT_6; // Enables the interrupt
  NVIC->ISER[0] |= (1 << 23); // EXTI6 & 7 have position 23 in the NVIC, since

  //TIMERS
  /* TIM3 --------------------------------------------------------------------*/
  //No pin assignment, we just plainly use it for the TOC
  //SET-UP for TIMs 3, CH3 & CH4 - TOCs, for random LED off and TBD
  TIM3->CR1 = 0x0000; // ON IN CODE BELOW
  TIM3->CR2 = 0x0000; // Always set to 0
  TIM3->SMCR = 0x0000; // Always set to 0
  TIM3->PSC = 32000;
  TIM3->CNT = 0;
  TIM3->ARR = 0xFFFF; //USED IN PWN
  TIM3->CCMR1 = 0x0000;  //CCyS = 0 (TOC)
                         //OCyM = 000 (no external output) 
                         //OCyPE = 0 (no preload)
  TIM3->CCER = 0x0000;   //CCyP = 0 (always in TOC)
                         //CCyE = (external output disabled)
  TIM3->CCR1 = random_num(0, 10000);
  //TIM3->CC1S = 00;
  //TIM3->OC1M = 000; //TOC NO OUTPUT
  TIM3->DIER |= BIT_0; //activated IRQ for channel 1
  TIM3->CR1 |= 1; // Set BIT_0 to 1 to activate the timer

  /* TIM 4 -------------------------------------------------------------------*/
  //Assigned to PB7 and PB7
  //SET-UP for TIMs 4, CH1 & CH2 - TICs
  TIM4->CR1 = 0x0000; //Set to 0 for Counter OFF - ON IN CODE BELOW
                      //ARPE off because NOT PWM
  TIM4->CR2 = 0x0000; //Always set to 0
  TIM4->SMCR = 0x0000; //Always set to 0
  TIM4->PSC = 31999; //Means fclk/(PSC+1)
  TIM4->CNT = 0;
  TIM4->ARR = 0xFFFF; //USED IN PWN
  //TIM4->CC3S = 01;
  //TIM4->CC4S = 01;

  //FIXME:
  TIM4->DIER = 0X0000; //No IRQ after succesful comparison -> CCyIE = 0

  /* COMMENTS ----------------------------------------------------------------*/
  //CCMR1 for ch1 and ch2
  //CCMR2 for ch3 and ch4
  
  //activate I/O here 
  //TIC ojo con la entrada

  // CCR4 
  // TIM3->CCR4 = rand_value_where_we_stop; //It will always be lower than 0xFFFF (65,535 > 10,000)
                  //Our max time value will be 10secs

  /* LEDs ---------------------------------------------------------------------*/
  //LED1
  //PA12 (EXTERNAL LED1) - AF
  GPIOA->MODER &= ~(1 << (12*2 + 1));
  GPIOA->MODER |= (1 << (12*2));
  GPIOA->AFR[0] |= (0x02 << (12*4)); // Writes 0010 in AFRL12
  //Set up with pull-up resistor (01)
  GPIOA->PUPDR &= ~(1 << (12*2 + 1));
  GPIOA->PUPDR |= (1 << (12*2));
  //LED2
  //PD2 (EXTERNAL LED2) - digital output (01)
  GPIOD->MODER &= ~(1 << (2*2 + 1));
  GPIOD->MODER |= (1 << (2*2));
  //Set up with pull-up resistor (01)
  GPIOD->PUPDR &= ~(1 << (2*2 + 1));
  GPIOD->PUPDR |= (1 << (2*2));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Requirements
    //Display GAME 1 (initially) --> DONE IN GLOBAL VAR DECLARATION
    //If the USER BUTTON is pressed, change to GAME 2 (at ANY time) --> global if
    //In GAME 1, wait predetermined time and start (use espera() function)

    //Global IF condition, so we may immediately switch between games
    //(a WHILE would force us to finish the code execution inside)
    
    //All LEDs shall be initially off - when changing game modes, upon a restart of the main while loop
    GPIOA->BSRR = (1 << 12) << 16;
    GPIOD->BSRR = (1 << 2) << 16;

    //Clear all timer flags to be used
    TIM4->SR = 0; // CLEAR FLAGS
    TIM3->SR = 0;

    if (prev_game != game)
    {
      prev_game = game;
      switch(game)
      {
        case 1: // GAME 1 - REACTION TIME
          while (game == 1)
          {
            BSP_LCD_GLASS_Clear(); //Clear LCD
            BSP_LCD_GLASS_DisplayString((uint8_t*)" GAME1");
            espera(2*sec);
            //if (prev_game != game) break;
            //GAME STARTS HERE
            BSP_LCD_GLASS_Clear(); //Not strictly needed since we are printing the same No. of chars to display
            BSP_LCD_GLASS_DisplayString((uint8_t*)" READY");
            espera(2*sec);
            //if (prev_game != game) break;
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t*)"  GO");
            //if (prev_game != game) break;
            //Waiting for users to input

            //random num generator lower than 10000 (ten thousand) - 10 secs to 

            while ((game == 1) && (winner == 0)) //(game == 1) is also necessary in case we want to change games here
            {
              playing = 1;
              if (prev_game != game) break;  //Not sure if needed

              //FIXME: review - ask
              //Start counters
              TIM3->CR1 |= 0x0001;   //CEN = 1 Start counter
              TIM3->EGR |= 0x0001;   //UG = 1->Generate an update event to update all registers 
              //TIM3->SR = 0;          //clear counter flags
   

              

              //Before 10 secs at ANY time, LED1 ON
              //Random timer reaches zero - led
              while ((TIM3->CNT) != (TIM3->CCR1))
              {
                if (prev_game != game) break;
              }
              while (winner == 0)
              {
                while((TIM3->SR&0x0002)==0);
                TIM3->SR &= ~(0x0002);
                GPIOA->BSRR = (1 << 12); // LED ON while no player has pressed their button yet
                  //Start TICs counters
                TIM4->CR1 |= BIT_0;
                TIM4->EGR |= BIT_0;
                if (prev_game != game) break;
              }
              if (prev_game != game) break;
            }

            //WINNER is determined by the interrupts, they will change the var winner to 1 or 2 respectively
            if (winner == 1)
            {
              GPIOA->BSRR = (1 << 12) << 16; //Turn off the LED after a win
              BSP_LCD_GLASS_Clear();
              
              //FIXME: review
              //which is tim4->cnt count for ch3 or ch4 ???????
              diff = ((TIM4->CNT) - (TIM3->CCR1));

              diff1 = diff + 10000; //+10000 to always have P1 won (1XXXX
              //p1w = strcat(" 1", (char*) diff);
              //format shall be Y user followed by XXXX milliseconds
              BSP_LCD_GLASS_DisplayString((uint8_t*) diff1);

              espera(2*sec); //wait so the player acknowledges their win
              winner = 0; //reset winner for future match
              playing = 0;
              //reset timers? or do we reset at the start of each game?
            }
            else if (winner == 2) // We use an else if because we only want ONE winner
            {
              GPIOA->BSRR = (1 << 12) << 16;
              BSP_LCD_GLASS_Clear();

              diff = ((TIM4->CNT) - (TIM3->CCR1));
              diff2 = diff + 20000; //+20000 to always have P2 won (2XXXX)
              //char p2w[] = " 2" + diff;
              BSP_LCD_GLASS_DisplayString((uint8_t*) diff2);

              espera(2*sec);
              winner = 0;
              playing = 0;
            }
          }
        break;

        case 2: // GAME 2 - COUNTDOWN
          while (game == 2)
          {
            //TODO: COUNTDOWN
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t*)" GAME2");
            espera(2*sec);
            //if (prev_game != game) break;

            //COUNTDOWN GAME
            //users are displayed the countdown in real time from 10 in real time
            //at a random time (a while before 0)
            //the countdown STOPS being displayed
            //the users have to attempt to press the button when the countdown reaches 0

            //The player with the closest time to 0 will win
            //Pressing before the countdown ends will result in displaying -XXXX time left
            //Pressing after the countdown ends will result in displaying +XXXX time passed
            //The player to have the closest absolute value to 0, wins

            //REVIEW BELOW
            //Determine OVERTIME or UNDERTIME with TOC minus TIC
            //undertime if TIM3 hasnt reached 0 so do operation (time_to_zero - input_time)

            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t*)" READY");
            espera(2*sec);
            //if (prev_game != game) break;
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t*)" SET");
/*
            //NOT A WHILE - Not waiting, just counting
            //We want a continuously running game (3 seconds max of waiting for user input)
            if ((winner == 0) && (tim4_TICs << 3*sec)) //if wait to press is longer than 3 secs after countdown = 0, abort and restart game
            {
              //start countdown with tim3_ch3
              if (prev_game != game) break;
              playing = 1;
              //When random timer reaches zero STOP LCD DISPLAY
              //keep counting down though
              // if lcd_on != 0 keep on displaying, else kill lcd

              //send an array of ASCII code for the countdown

              if (winner == 1)
              {
                GPIOA->BSRR = (1 << 12); //Turn ON LED1 for P1 win
                BSP_LCD_GLASS_Clear();
                
                //format shall be Y user followed by XXXX milliseconds
                BSP_LCD_GLASS_DisplayString((uint8_t*)" P1 W"); //(" 1%d", time_taken)

                espera(2*sec); //wait so the player acknowledges their win
                winner = 0; //reset winner for future match
                playing = 0;
                //reset timers? or do we reset at the start of each game?
              }
              else if (winner == 2) // We use an else if because we only want ONE winner
              {
                GPIOD->BSRR = (1 << 2); //Turn ON LED2 for P2 win
                BSP_LCD_GLASS_Clear();
                BSP_LCD_GLASS_DisplayString((uint8_t*)" P2 W");

                espera(2*sec);
                winner = 0;
                playing = 0;
              }
              //Retry message for when players take more than 20 post countdown reaching 0 to react
              if (tim4_TICs << 30*sec) BSP_LCD_GLASS_DisplayString((uint8_t*)" RETRY");

              //USE TIMER to count how many secs and add to a variable
              //time_taken = timer_value at break
            }
*/
          }
        break;

        //This code below should be unreachable on purpose
        //Written to acknowledge a logical failure (of our code)
        default:
          GPIOA->BSRR = (1 << 12) << 16;
          BSP_LCD_GLASS_Clear();
          BSP_LCD_GLASS_DisplayString((uint8_t*)" ERROR");
          espera(2*sec);
          BSP_LCD_GLASS_Clear();
          BSP_LCD_GLASS_DisplayString((uint8_t*)" RESET");
        break;
      }
    }
  HAL_Delay(50); //to avoid button bouncing
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
