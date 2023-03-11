/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "battery.hpp"
#include "encoder.hpp"
#include "gyro.hpp"
#include "mine.hpp"
#include "motor.hpp"
#include "parts.hpp"
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
parts::wheel<std::unique_ptr<pwm::Encoder<float, int32_t>>,
             std::unique_ptr<pwm::Encoder<float, int16_t>>>
    enc;
parts::wheel<std::unique_ptr<pwm::Motor>, std::unique_ptr<pwm::Motor>> motor;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {
  }
  if (htim == &htim1) {
    enc.right->read_encoder_value(1000);
    enc.left->read_encoder_value(1000);
  }
}

void HAL_SYSTICK_Callback(void)  // 1kHz
{
  // This is system clock timer
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  // initialise_monitor_handles();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, NULL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  spi::Gyro gyro;
  HAL_Delay(100);
  adc::Battery<float, uint16_t> batt(&hadc1);
  enc.right = std::make_unique<pwm::Encoder<float, int16_t>>(TIM8);
  enc.left = std::make_unique<pwm::Encoder<float, int32_t>>(TIM2);
  motor.left = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_1,
                                            TIM_CHANNEL_2);
  motor.right = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_3,
                                             TIM_CHANNEL_4);
  printf("stroberry\r\n");
  // uint8_t test;
  // uint8_t *flash_data = (uint8_t *)Flash_load(&test, sizeof(uint8_t));
  // printf("flash_data:%u\n", *flash_data);
  // (*flash_data)++;

  // if (!Flash_store(&test, sizeof(uint8_t))) {
  //   printf("Failed to write flash\n");
  //   Error_Handler();
  // }
  // Flash_clear();
  char d[100];
  sprintf(d, "USSR %d\r\n", (uint16_t)1991);
  uint32_t *flash_data = (uint32_t *)Flash_load();
  printf("%c\r\n", *flash_data);
  memcpy(flash_data, d, sizeof d);

  if (!Flash_store()) {
    printf("Failed to write flash\n");
  }
  // Flash_clear();
  while (1) {
    // HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    // read_gyro();
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    // sum = gyro.read_gyro().y * 0.001f;
    // HAL_Delay(10);
    // printf("%f\r\n", sum);
    batt.read_batt();
    // motor.right->drive(250);
    // HAL_Delay(3000);
    // motor.right->drive(999);
    // printf("%f %f\r\n", enc.left->cnt_total * 3.3 / 360 * 3.14,
    //        enc.right->cnt_total * 3.3 / 360 * 3.14);
    // printf("is: %f %f\r\n", enc.left->cnt_total, enc.right->cnt_total);

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// extern "C" int _write(int file, char *ptr, int len)
// {
//   HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 100);
//   return len;
// }
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
