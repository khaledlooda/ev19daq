
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
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

#define pi 3.1415
#define raduis_of_wheel .5
/* USER CODE BEGIN Includes */

/* flages to increment every pulse to calculate the speed after 4 pulses */ 

uint16_t flage1=0 ;			  
uint16_t flage2=0 ;
uint16_t flage3=0 ;
uint16_t flage4=0 ;

uint16_t input_capture1=0 ;  // variable to store counter of timer 1 value
uint16_t input_capture2=0 ;	 // variable to store counter of timer 2 value
uint16_t input_capture3=0 ;	 // variable to store counter of timer 3 value
uint16_t input_capture4=0 ;	 // variable to store counter of timer 4 value

float first_wheel_speed =0 ;		// speed of wheel 1
float second_wheel_speed =0 ;  // speed of wheel 2
float third_wheel_speed =0 ;		// speed of wheel 3
float forth_wheel_speed =0 ;		// speed of wheel 4

float avarage_speed =0 ;   		// avarage speed of the car 


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  
{
 if (htim->Instance==TIM1)
  {
		flage1 ++ ;
}
	 if (htim->Instance==TIM2)
	 {
		flage2 ++ ;
	 }
	  if (htim->Instance==TIM3)
		{
		flage3 ++ ;
		}
			if (htim->Instance==TIM4)
		 {
		flage4 ++ ;		
		 }
	 } /* USER CODE END 0 */

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
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);  
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);  
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);  
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);  

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if (flage1 == 5){
	 input_capture1= __HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_1);    //read TIM1 channel 1 capture value
  __HAL_TIM_SetCounter(&htim1, 0);    //reset counter after input capture interrupt occurs	
		first_wheel_speed = (((60*1000 /input_capture1) *(2*pi/60)) * raduis_of_wheel ) ; // 60 to convert to min, 1000 for second, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		flage1 =0 ;
	}
	if (flage2 == 5){
	 input_capture2= __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);    //read TIM1 channel 1 capture value
  __HAL_TIM_SetCounter(&htim2, 0);    //reset counter after input capture interrupt occurs	
		second_wheel_speed = (((60*1000 /input_capture2) *(2*pi/60)) * raduis_of_wheel ) ; // 60 to convert to min, 1000 for second, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		flage2 =0 ;
	}
	if (flage3 == 5){
	 input_capture3= __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1);    //read TIM1 channel 1 capture value
  __HAL_TIM_SetCounter(&htim3, 0);    //reset counter after input capture interrupt occurs	
		third_wheel_speed = (((60*1000 /input_capture3) *(2*pi/60)) * raduis_of_wheel ) ;// 60 to convert to min, 1000 for second, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		flage3 =0 ; 
	}
	if (flage4 == 5){
	 input_capture4= __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_1);    //read TIM1 channel 1 capture value
  __HAL_TIM_SetCounter(&htim4, 0);    //reset counter after input capture interrupt occurs	
		forth_wheel_speed = (((60*1000 /input_capture1) *(2*pi/60)) * raduis_of_wheel ) ;// 60 to convert to min, 1000 for second, 2pi/60 to convert to rad/sec, *raduis to convert to m/sec
		flage4 =0 ; 
	}
	
	avarage_speed = ((first_wheel_speed+ second_wheel_speed+ third_wheel_speed+forth_wheel_speed)/4) ; // avarage speed of the car 
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
