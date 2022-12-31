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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mine.h"
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
// extern "C" void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t read_encoder_value(void){
    int16_t enc_buff = (int16_t)TIM2->CNT;
    TIM2->CNT = 0; 
    return (int16_t)enc_buff;
  }

uint8_t spi_gyro_read(uint8_t);
void spi_gyro_write(uint8_t address, uint8_t value);
float gyro_offset = 0.0f;
float spi_gyro_OUT_Z(void)
{
  /*HAL_Delay(1);*/

  uint16_t Z_H = spi_gyro_read(0x2D);
  uint16_t Z_L = spi_gyro_read(0x2C);
  //printf("Z = %d\r\n", (int16_t)((Z_H << 8) + Z_L));
  /*HAL_Delay(1);*/
  return (float)((int16_t)((Z_H << 8) + Z_L)) * 0.00875f;
}
void imu_calibulation()
{
  HAL_Delay(100);
  spi_gyro_write(0x20,0x00);
  HAL_Delay(100);
  spi_gyro_write(0x20,0x0F);
  HAL_Delay(100);
  float temp = 0.0f;
  int times = 2000;
  for(int i=0; i < times; i++){
    temp += spi_gyro_OUT_Z();
    HAL_Delay(1);
  }
  gyro_offset = temp / times;
  printf("%f",gyro_offset);
}
float read_gyro()
{
  float Z_vel = spi_gyro_OUT_Z() -gyro_offset;
  if(abs(Z_vel)<0.5)Z_vel=0;
  return Z_vel;
}
void spi_gyro_who_am_i(void)
{
  HAL_Delay(100);
  uint8_t report = spi_gyro_read(0x0F);
  printf("WHO_AM_I = %d\r\n", report);
  HAL_Delay(100);
}

void spi_gyro_write(uint8_t address, uint8_t value)
{
  uint8_t transmit[2] = {address, value};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CSピン立ち下げ
  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1, transmit, 2, 100);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CSピン立ち上げ
}

uint8_t spi_gyro_read(uint8_t address)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CSピン立ち下げ
  //HAL_Delay(1);
  uint8_t transmit;
  transmit = address | 0x80;
  uint8_t receive=0x00;
  HAL_SPI_Transmit(&hspi1,&transmit,1,100);
  HAL_SPI_Receive(&hspi1,&receive,1,100);
  //HAL_SPI_TransmitReceive(&hspi1, &transmit, &receive, 2, 100);
  //HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CSピン立ち上げ
  return receive;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // initialise_monitor_handles();
  
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout,NULL);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);


  /* USER CODE END 2 */

  /* Infinite loop */
  int32_t cnt_total=0;
  /* USER CODE BEGIN WHILE */
  // HAL_Delay(3000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  spi_gyro_who_am_i();
  spi_gyro_who_am_i();
  imu_calibulation();
  float sum = 0;
  while (1)
  {
    // printf("Hello World% f\n",t+=0.1);
    
    // HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
    // cnt_total+=read_encoder_value();
    // read_gyro();
    // printf("%ld\r\n",cnt_total);
    /* USER CODE END WHILE */ 
    /* USER CODE BEGIN 3 */
  
    sum += read_gyro()*0.001f;
    HAL_Delay(1);
    printf("%f\r\n", sum);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
extern "C" int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 100);
  return len;
}
// int _io_put_char(int ch)
// {
//   HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 100);
//   return ch;
// }
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
