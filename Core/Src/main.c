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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Libs for LCD IPS 1.47"
#include "GUI_Paint.h"
#include "fonts.h"
#include "image.h"
#include "LCD_Test.h"
#include "DEV_Config.h"
#include "LCD_1in47.h"

// Pulse oximeter MAX30102 library
//#include "MAX30102_Oximeter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX30102_I2C_ADDRESS    0x57

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Read Buffer for oximeter MAX30102
uint8_t rbuf[6];

uint8_t spo2 = 				0; // Oxygen saturation level in red blood cells
uint32_t heartbeat = 		0; // Heart beats per minute

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void MAX30102_Init(void);
void MAX30102_StartCollect(void);
void MAX30102_GetHeartbeatSPO2(uint8_t *spo2, uint32_t *heartbeat);
float MAX30102_GetTemperature(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Initialize LCD IPS 1.47"
void initLCD()
{
	LCD_1IN47_SetBackLight(1000); // Not sure if we need it but of. docs has it
	LCD_1IN47_Init(VERTICAL);
	LCD_1IN47_Clear(WHITE);

	// Set screen perspective as VERTICAL
	Paint_NewImage(LCD_1IN47_HEIGHT,LCD_1IN47_WIDTH, ROTATE_90, WHITE);
	// Set screen cleaner function for LCD
	Paint_SetClearFuntion(LCD_1IN47_Clear);
	// Set pointer to draw (it helps to draw automatically)
	Paint_SetDisplayFuntion(LCD_1IN47_DrawPoint);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  initLCD();



  /* IIC MAX30102 Polling Method */
//  if(MAX30102_CheckConnection() == OXIMETER_OK)
//  {
//	  MAX30102_ResetSensor();
//
//	  MAX30102_LEDConfig();
//
//	  MAX30102_SetSpO2Mode();
//
//	  MAX30102_SlotCofig();
//
//	  MAX30102_ResetFIFOPointers();
//
//	  MAX30102_DataCollectTurnOn();
//  }

  uint8_t mode_reset = 0x40; // Reset bit
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x09, I2C_MEMADD_SIZE_8BIT, &mode_reset, 1, HAL_MAX_DELAY);
  HAL_Delay(100); // Wait for reset

  // Clear the reset bit
  uint8_t mode_config = 0x03; // Set to SpO2 mode
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x09, I2C_MEMADD_SIZE_8BIT, &mode_config, 1, HAL_MAX_DELAY);

  // Configure LED current (max for both LEDs)
  uint8_t led1_pa = 0x7F; // Red LED current
  uint8_t led2_pa = 0x7F; // IR LED current
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, &led1_pa, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x0D, I2C_MEMADD_SIZE_8BIT, &led2_pa, 1, HAL_MAX_DELAY);

  // Set SpO2 configuration (ADC range, sample rate, pulse width)
  uint8_t spo2_config = 0x27; // 0x27 sets sample rate to 100 Hz, pulse width to 411us
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x0A, I2C_MEMADD_SIZE_8BIT, &spo2_config, 1, HAL_MAX_DELAY);

  // Enable slots (Slot 1: Red, Slot 2: IR)
  uint8_t slot_config = 0x21; // Slot 1 for Red, Slot 2 for IR
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x11, I2C_MEMADD_SIZE_8BIT, &slot_config, 1, HAL_MAX_DELAY);

  // Reset FIFO pointers
  uint8_t fifo_config = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x04, I2C_MEMADD_SIZE_8BIT, &fifo_config, 1, HAL_MAX_DELAY); // FIFO Write Pointer
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x05, I2C_MEMADD_SIZE_8BIT, &fifo_config, 1, HAL_MAX_DELAY); // FIFO Overflow Counter
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &fifo_config, 1, HAL_MAX_DELAY); // FIFO Read Pointer



  uint8_t start_collect[2] =  {0x00, 0x01}; // Command to start data collection
  HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x20, I2C_MEMADD_SIZE_8BIT, &start_collect, 2, HAL_MAX_DELAY);



  void MAX30102_GetHeartbeatSPO2(uint8_t *spo2, uint32_t *heartbeat)
  {
	  uint8_t rbuf[8]; // Buffer to store the read data

	  // Read from the register where SPO2 and Heartbeat data are stored
	  HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, rbuf, 8, HAL_MAX_DELAY);

	  // Extract SPO2 and Heartbeat data
	  *spo2 = rbuf[0]; // SPO2 level
	  *heartbeat = (rbuf[2] << 24) | (rbuf[3] << 16) | (rbuf[4] << 8) | rbuf[5]; // Heartbeat rate
  }


  float MAX30102_GetTemperature(void)
  {
	  uint8_t temp_buf[2];
	  HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDRESS << 1, 0x14, I2C_MEMADD_SIZE_8BIT, temp_buf, 2, HAL_MAX_DELAY);

	  // Convert the temperature data
	  float temperature = temp_buf[0] + temp_buf[1] / 100.0;
	  return temperature;
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Get SPO2 and Heartbeat values */
	  void MAX30102_GetHeartbeatSPO2(uint8_t *spo2, uint32_t *heartbeat);

	  /* Prepare text for SPO2 and Heartbeat values */
	  char spo2Text[50];
	  sprintf(spo2Text, "SPO2: %d%%", spo2);\
	  Paint_DrawString_EN(0, 70, spo2Text, &Font16, BLACK, WHITE); // Display SPO2 on the screen

	  char heartbeatText[50];
	  sprintf(heartbeatText, "bpm: %lu", heartbeat);
	  Paint_DrawString_EN(0, 120, heartbeatText, &Font16, BLACK, WHITE); // Display Heartbeat on the screen


	  /* Get and format temperature */
	  float temperature = MAX30102_GetTemperature();

	  char temperatureText[50];
	  sprintf(temperatureText, "T: %.2f C", temperature); // Format Temperature string
	  Paint_DrawString_EN(0, 250, temperatureText, &Font16, BLACK, WHITE); // Display Temperature on the screen


	  // Draw text at different coordinates to test the LCD
	  Paint_DrawString_EN(0, 50, "dwd1", &Font16, BLACK, WHITE);


//	  Paint_DrawString_EN(125, 20, "dwd2", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(0, 30, "dwd3", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(65, 80, "dwd4", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(70, 120, "dwd5", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(70, 140, "dwd6", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(70, 170, "dwd7", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(70, 190, "dwd8", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(70, 200, "dwd9", &Font16, BLACK, WHITE);
//
//	  Paint_DrawString_EN(70, 240, "dwd10", &Font16, BLACK, WHITE);
//	  Paint_DrawString_EN(70, 260, "dwd11", &Font24, BLACK, WHITE);
//	  Paint_DrawString_EN(60, 290, "dwd12", &Font24, BLACK, WHITE);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
