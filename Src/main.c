/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <main.h>
#include <i2c.h>
#include <gpio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <lcd.h>
#include <led.h>
#include <temp_sensor.h>
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_display_welcome(void);
void LCD_display_goodbye(void);
void LCD_sendTemperature(TemperatureSensor ts);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  temperature_sensor_init();

  LED_State(LED_ON);
  LCD_display_welcome();

  TemperatureSensor ts;
  do {
    ts = get_temperature_sensor_data(ts);
  } while (!ts.is_valid);
  LCD_sendTemperature(ts);
  HAL_Delay(10000);
  LCD_Clear();
  LCD_display_goodbye();
  LCD_Backlight(LCD_BACKLIGHT_OFF);

  LED_State(LED_OFF);

  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
  HAL_PWR_EnterSTANDBYMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */
void LCD_display_welcome(void) {
	uint_fast8_t delay = 100;
	LCD_SendCommand(0b10000000);
	LCD_SendString(" W");
	HAL_Delay(delay);
	LCD_SendString("e");
	HAL_Delay(delay);
	LCD_SendString("l");
	HAL_Delay(delay);
	LCD_SendString("c");
	HAL_Delay(delay);
	LCD_SendString("o");
	HAL_Delay(delay);
	LCD_SendString("m");
	HAL_Delay(delay);
	LCD_SendString("e");
	HAL_Delay(delay);
	LCD_SendString(",");
	HAL_Delay(delay);

	LCD_SendCommand(0b11000000);
	LCD_SendString("     %");
	HAL_Delay(delay);
	LCD_SendString("u");
	HAL_Delay(delay);
	LCD_SendString("s");
	HAL_Delay(delay);
	LCD_SendString("e");
	HAL_Delay(delay);
	LCD_SendString("r");
	HAL_Delay(delay);
	LCD_SendString("n");
	HAL_Delay(delay);
	LCD_SendString("a");
	HAL_Delay(delay);
	LCD_SendString("m");
	HAL_Delay(delay);
	LCD_SendString("e");
	HAL_Delay(delay);
	LCD_SendString("%");
	HAL_Delay(2500);
	LCD_Clear();
	LCD_SendCommand(0b1100);
}

void LCD_display_goodbye(void) {
	uint_fast8_t delay = 100;
	LCD_SendCommand(0b10000000);
	LCD_SendString("    G");
	HAL_Delay(delay);
	LCD_SendString("o");
	HAL_Delay(delay);
	LCD_SendString("o");
	HAL_Delay(delay);
	LCD_SendString("d");
	HAL_Delay(delay);
	LCD_SendString("b");
	HAL_Delay(delay);
	LCD_SendString("y");
	HAL_Delay(delay);
	LCD_SendString("e");
	HAL_Delay(delay);
	LCD_SendString("!");
	HAL_Delay(delay);
	HAL_Delay(2500);
	LCD_Clear();
	LCD_SendCommand(0b1100);
}

void LCD_sendTemperature(TemperatureSensor ts) {
	LCD_Clear();
	LCD_SendCommand(0b10000000);
	if (ts.is_valid) {
		int16_t t = get_temperature_sensor_temperature(ts);
		uint16_t rh = get_temperature_sensor_rel_humidity(ts);
		char string[20];
		sprintf(string, "Temp.: %d.%d C", t / 10, t % 10);
		LCD_SendString(string);
		memset(string, '\0', sizeof(string));
		sprintf(string, "RH: %d.%d%%", rh / 10, rh % 10);
		LCD_SendCommand(0b11000000);
		LCD_SendString(string);
	} else {
		LCD_SendString("  TEMP. SENSOR");
		LCD_SendCommand(0b11000000);
		LCD_SendString("     ERROR");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
