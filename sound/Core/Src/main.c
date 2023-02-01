/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
char* Notes[] = {"3Rn4","3Fn4","3Rn2","3Rn2","3Sn2","3Rn2","3Dn2","3Rn4","3Ln4","3Rn2","3Rn2","3L#2","3Ln2","3Fn2","3Rn2","3Ln2","4Rn2","3Rn2","3Dn3","3Dn3","2Ln3","3Mn3","3Rn3"};
uint8_t end_d = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void nota(char not[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  RCC->APB1ENR = (1 << 0)|(1<<3);//тактируем счетчик TIM2(включаем его), тактируем TIM5
   RCC->AHB1ENR = 1;//тактируем пин А0
   TIM2->CR1 = 0x1; //разрешили счёт таймера 2
   TIM5->CR1 = 0x0;// разрешили счёт таймера 5
   TIM2->CCMR1 = 0x6 << TIM_CCMR1_OC1M_Pos; //канал 1 активен пока CNT меньше CCR1 иначе неактивен
   TIM2->ARR = 1000; //когда значение в счётчике достигает величины, записанной в этом регистре,
   //следующий импульс сбрасывает счётчик в 0, при этом генерируется сигнал переполнения счётчика,
   //который используется как update event - событие обновления. Если в TIMx_ARR записано значение 0, то счётчик таймера останавливается.
   TIM5->ARR = 1000;
   TIM2->CCR1 = 500; //если меньше 500 нолик больше единичка выходной сигнал ????где
   TIM2->CCER = 1 << TIM_CCER_CC1E_Pos; //сигнал ОС1 является выходным и выходит на нулевом пине ???какая буква
   TIM5->DIER = (1 << 0);// включаем прерывания по первому каналу по сравнению
   //почему проскакивает 16
   //TIM2->PSC = 16; //прескейлер
   //	  for(int i = 0; i < 1600000; i++);
   //	  TIM2->PSC = 32; //прескейлер
   //	  for(int i = 0; i < 1600000; i++);

 	  //TIM2->PSC = 16; //прескейлер

   //GPIOC->MODER = 1 « 26; //настраиваем пин PINC13 на выход
   GPIOA->MODER |= 0x2; //настроили PINA0 на мод альтернативной функции т.к. таймер это альтернативная функция страница 44 датащита
   GPIOA->AFR[0] |= 1; //выбрали какую альтернативную функцию надо использовать исходя страница 44 датащите это AF1


   void noteLya();
   NVIC->ISER[1] |= 1<<18;
   __enable_irq();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	for(int i = 0; i<23;i++ )
	{
		nota(Notes[i]);
	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void nota(char not[])
{
	uint32_t Freq = 0,  k = 1, D = 0;
	//uint8_t Buf[4];
	//&Buf[0] = &not[0];
	switch((uint8_t)(not[0]))
	{
		case '1' : Freq = 65;  k = 1; break;
		case '2' : Freq = 131; k = 2; break;
		case '3' : Freq = 262; k = 4; break;
		case '4' : Freq = 523; k = 8; break;
		case '5' : Freq = 1026;k = 16;break;
		default: Freq = 131; break;
	}
	switch((uint8_t)(not[1]))
	{
		case 'D' : break;
		case 'R' : Freq += 8*k; break;
		case 'M' : Freq += 17*k; break;
		case 'F' : Freq += 22*k; break;
		case 'S' : Freq += 32*k;break;
		case 'L' : Freq += 45*k; break;
		case 'H' : Freq += 58*k;break;
		case 'P' : RCC->APB1ENR = 0; break;
		default: Freq = 111; break;
	}
	switch((uint8_t)(not[2]))
	{
		case '#' :
			if ((uint8_t)(not[1]) == 'D'|(uint8_t)(not[1]) =='R')
			{
					Freq += 4*k;
			}
			if ((uint8_t)(not[1]) == 'F')
			{
				Freq += 5*k;
			}
			if ((uint8_t)(not[1]) == 'S'|(uint8_t)(not[1]) =='L')
			{
				Freq += 6*k;
			}
			break;
			case 'b' :
			if ((uint8_t)(not[1]) =='R')
			{
				Freq -= 4*k;
			}
			if ((uint8_t)(not[1]) == 'M'|(uint8_t)(not[1]) == 'S')
			{
				Freq -= 5*k;
			}
			if ((uint8_t)(not[1]) == 'H'|(uint8_t)(not[1]) =='L')
			{
				Freq -= 6*k;
			}
			break;
		default: break;
	}

	D = (uint32_t)(not[3]) - 0x30 + 1;




	TIM2->PSC = (uint32_t) 16000/Freq;
//start
	TIM5->CNT = 0;
	TIM5->PSC  = D*2000;
	TIM5->CR1 |= 0x1;// разрешили счёт таймера 5
	end_d = 0;
    while(end_d == 0 )
    {}
    TIM5->CR1 &= ~(0x1);// разрешили счёт таймера 5
}
 	 void otzhimka()
 	 {
	 RCC->APB1ENR = 0;
	 return;
 	 }
     void do2()
     {
   	   RCC->APB1ENR = 1;
   	   TIM2->PSC = 122; //прескейлер
   	   return;
     }
     void dodi2()
     {
        RCC->APB1ENR = 1;
        TIM2->PSC = 115 ; //прескейлер
        return;
     }
     void re2()
     {
        RCC->APB1ENR = 1;
        TIM2->PSC = 109; //прескейлер
        return;
      }
     void mibe2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 103; //прескейлер
         return;
     }
     void mi2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 97; //прескейлер
         return;
     }
     void fa2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 92; //прескейлер
         return;
     }
     void fadi2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 87; //прескейлер
         return;
     }
     void sol2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 82; //прескейлер
         return;
     }
     void soldi2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 77; //прескейлер
         return;
     }
     void lya2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 73; //прескейлер
         return;
     }
     void sibe2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 69; //прескейлер
         return;
     }
     void si2()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 65; //прескейлер
         return;
     }
     void do3()
     {
   	   RCC->APB1ENR = 1;
   	   TIM2->PSC = 61; //прескейлер
   	   return;
     }
     void dodi3()
     {
        RCC->APB1ENR = 1;
        TIM2->PSC = 58 ; //прескейлер
        return;
     }
     void re3()
     {
        RCC->APB1ENR = 1;
        TIM2->PSC = 55; //прескейлер
        return;
      }
     void mibe3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 52; //прескейлер
         return;
     }
     void mi3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 49; //прескейлер
         return;
     }
     void fa3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 46; //прескейлер
         return;
     }
     void fadi3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 44; //прескейлер
         return;
     }
     void sol3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 41; //прескейлер
         return;
     }
     void soldi3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 39; //прескейлер
         return;
     }
     void lya3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 37; //прескейлер
         return;
     }
     void sibe3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 35; //прескейлер
         return;
     }
     void si3()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 33; //прескейлер
         return;
     }
     void do4()
     {
   	   RCC->APB1ENR = 1;
   	   TIM2->PSC = 31; //прескейлер
   	   return;
     }
     void dodi4()
     {
        RCC->APB1ENR = 1;
        TIM2->PSC = 29 ; //прескейлер
        return;
     }
     void re4()
     {
        RCC->APB1ENR = 1;
        TIM2->PSC = 27; //прескейлер
        return;
      }
     void mibe4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 26; //прескейлер
         return;
     }
     void mi4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 24; //прескейлер
         return;
     }
     void fa4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 23; //прескейлер
         return;
     }
     void fadi4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 22; //прескейлер
         return;
     }
     void sol4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 21; //прескейлер
         return;
     }
     void soldi4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 19; //прескейлер
         return;
     }
     void lya4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 18; //прескейлер
         return;
     }
     void sibe4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 17; //прескейлер
         return;
     }
     void si4()
     {
         RCC->APB1ENR = 1;
         TIM2->PSC = 16; //прескейлер
         return;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
