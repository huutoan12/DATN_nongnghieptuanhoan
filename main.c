/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
float temp,light,humid,PH;
uint8_t den,bom;
int status=0;
char M[32];
uint32_t AD0,AD1,AD2,AD3;
uint32_t t_humid=50,t_light=50;
float Offset=0.0;
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void Lcd_Cauhinh(void);//KHOI TAO LCD
void Lcd_Ghi_Lenh(char lenh);//GHI LENH
void Lcd_Ghi_Dulieu(char data);//GHI DATA
void Lcd_Ghi_Lenh (char malenh)
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);//LCD_RW=0
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);//RS=0
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//LCD_EN= 1;
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, (malenh>>4)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, (malenh>>5)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, (malenh>>6)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, (malenh>>7)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//LCD_EN=0
HAL_Delay(5); //5ms
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//LCD_EN= 1;
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, (malenh&0x01));
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, (malenh>>1)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, (malenh>>2)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, (malenh>>3)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//LCD_EN=0
HAL_Delay(5); //5ms
}
void Lcd_Ghi_Dulieu (char dulieu)
{
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);//LCD_RW=0
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);//RS=1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//LCD_EN= 1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, (dulieu>>4)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, (dulieu>>5)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, (dulieu>>6)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, (dulieu>>7)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//LCD_EN=0
HAL_Delay(5); //5ms
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//LCD_EN= 1
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, (dulieu&0x01));
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, (dulieu>>1)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, (dulieu>>2)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, (dulieu>>3)&0x01);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//LCD_EN=0
HAL_Delay(5); //5ms
}
void Lcd_Ghi_chuoi (char *str)
{
while(*str)
{
Lcd_Ghi_Dulieu(*str);
str++;
}
}
void Lcd_Cauhinh (void)
{
Lcd_Ghi_Lenh(0x03); //Bat dau
Lcd_Ghi_Lenh(0x02); // tro ve dau dong
Lcd_Ghi_Lenh(0x28);// Giao ti?p v?i V K b?ng 4 ch n
Lcd_Ghi_Lenh(0x06);// T? d?ng dua con tr? d?n v? tr  ti?p theo
Lcd_Ghi_Lenh(0x0c);// B?t hi?n th?, t?t con tr?
Lcd_Ghi_Lenh(0x01); // X a m n h nh
}
void ADC_Kenh_0(){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void ADC_Kenh_1(){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void ADC_Kenh_2(){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void ADC_Kenh_3(){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  MX_ADC1_Init();
  Lcd_Cauhinh();

  while (1)
  {
     ADC_Kenh_0();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		AD0 = HAL_ADC_GetValue(&hadc1);
		temp = (float)((AD0 * 6.5 ) / 4095.0 )/2 *100 ;
		HAL_ADC_Stop(&hadc1);
		
    ADC_Kenh_1();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		AD1 = HAL_ADC_GetValue(&hadc1);
		light= 100-(float)(AD1)/4095.0*100;
		HAL_ADC_Stop(&hadc1);
		
		ADC_Kenh_2();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		AD2 = HAL_ADC_GetValue(&hadc1);
		humid= (100-(float)(AD2)/4095.0*100)*2;
 		HAL_ADC_Stop(&hadc1);
		
		ADC_Kenh_3();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		int bufpH[20];                
		for(int i=0;i<20;i++)       
		{ 
			bufpH[i]=HAL_ADC_GetValue(&hadc1);
			HAL_Delay(10);
		}
		for(int i=0;i<19;i++)        
		{
			for(int j=i+1;j<20;j++)
			{
				if(bufpH[i]>bufpH[j])
				{
					int t=bufpH[i];
					bufpH[i]=bufpH[j];
					bufpH[j]=t;
				}
			}
		}
		float avgValue=0;
		for(int i=12;i<18;i++)                      
			avgValue+=bufpH[i];
		float phValue=(float)avgValue*5.0/4095.0/6; 
		/*AD3=HAL_ADC_GetValue(&hadc1);
		float volt=(float)AD3*5.0/4096; */
		PH=2*phValue+Offset;
 		HAL_ADC_Stop(&hadc1);

		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0){
			while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0){}
			status+=1;
		}

		if(status%2==0){
			HAL_Delay(500);
			Lcd_Ghi_Lenh(0x80);
			sprintf(&M[0],"AUTO %.f\xDF""C P:%.2f  ",temp,PH);
			Lcd_Ghi_chuoi(&M[0]);
			Lcd_Ghi_Lenh(0xc0);
			sprintf(&M[0]," L:%.f%%    H:%.f%%   ",light,humid);
			Lcd_Ghi_chuoi(&M[0]);
			
			if(light<t_light){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
			}
			else{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
			}
			if(humid<t_humid){
				if (PH<5.8 || PH>8){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
				}
				else{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
				}
			}
			else{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
			}
			
			}
		else{
			HAL_Delay(500);
			Lcd_Ghi_Lenh(0x80);
			sprintf(&M[0],"MANU %.f\xDF""C P:%.2f  ",temp,PH);
			Lcd_Ghi_chuoi(&M[0]);
			Lcd_Ghi_Lenh(0xc0);
			sprintf(&M[0]," L:%.f%%    H:%.f%%   ",light,humid);
			Lcd_Ghi_chuoi(&M[0]);
			
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0){
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0){}
					den=~den;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,den);
			}
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==0){
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==0){}
					bom=~bom;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,bom);
			}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
