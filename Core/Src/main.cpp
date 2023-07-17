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
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./battery.hpp"
#include "./buzzer.hpp"
#include "./controller.hpp"
#include "./encoder.hpp"
#include "./gyro.hpp"
#include "./ir_sensor.hpp"
#include "./motor.hpp"
#include "./pid.hpp"
#include "./state.hpp"
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
void SystemClock_Config(void);  // NOLINT
/* USER CODE BEGIN PFP */
// extern "C" void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
std::unique_ptr<spi::Gyro> gyro;
parts::wheel<std::unique_ptr<pwm::Encoder<float, int32_t>>, std::unique_ptr<pwm::Encoder<float, int16_t>>> enc;
parts::wheel<std::unique_ptr<pwm::Motor>, std::unique_ptr<pwm::Motor>> motor;
std::unique_ptr<state::Controller<float, state::Status<float>, state::Pid<float>>> ctrl;
std::unique_ptr<adc::IrSensor<uint16_t>> ir_sensor;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {
    ctrl->update();

    ctrl->drive_motor<pwm::Motor, &pwm::Motor::drive_vcc>(*(motor.left), *(motor.right), 1, -1);
  }
  if (htim == &htim1) {
    ctrl->status
        .update<pwm::Encoder<float, int32_t>, pwm::Encoder<float, int16_t>, &pwm::Encoder<float, int32_t>::read_encoder_value, &pwm::Encoder<float, int16_t>::read_encoder_value>(
            *(enc.left), *(enc.right), []() { return gyro->read_gyro().z; });
  }

  static uint32_t callback_counter = 0;
  const uint32_t frequency = 160 * 10000;
  if (htim == &htim6)  // 100kHz
  {
    // callback_counter = (callback_counter + 1) % frequency;
    // if (callback_counter == 0) {
    //   // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
    //   // GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    // }
    // ir_sensor->ir_sampling();
    // if (callback_counter % 64 == 0)  // 2kHz
    // {
    //   ir_sensor->ir_update();
    //   printf("ok\r\n");
    // }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle) {
  static uint32_t callback_counter = 0;
  const uint32_t frequency = 160 * 1000;
  if (AdcHandle == &hadc2) {
    callback_counter = (callback_counter + 1) % frequency;
    ir_sensor->ir_sampling();
    if (callback_counter % 64 == 0)  // 2kHz
    {
      ir_sensor->ir_update();
      printf("ok\r\n");
    }
  }
}

void HAL_SYSTICK_Callback(void) {  // 1kHz
  // This is system clock timer
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main() {
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM12_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, nullptr);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_Delay(1000);
  gyro = std::make_unique<spi::Gyro>();
  HAL_Delay(100);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  adc::Battery<float, uint32_t> batt(&hadc1);
  enc.right = std::make_unique<pwm::Encoder<float, int16_t>>(TIM8);
  enc.left = std::make_unique<pwm::Encoder<float, int32_t>>(TIM2);
  motor.left = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2);
  motor.right = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4);
  ctrl = std::make_unique<state::Controller<float, state::Status<float>, state::Pid<float>>>();
  ir_sensor = std::make_unique<adc::IrSensor<uint16_t>>(&hadc2, 4);
  pwm::Buzzer buzzer(&htim12, TIM_CHANNEL_2);
  printf("stroberry\r\n");
  HAL_Delay(1000);
  buzzer.beep("ok");
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim6);
  pwm::IrLight ir_light_1(&htim9, TIM_CHANNEL_1), ir_light_2(&htim9, TIM_CHANNEL_2);
  // ir_light_1.ir_flash_start();
  ir_light_2.ir_flash_start();
  HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setbuf(stdout, NULL);
  setbuf(stdin, NULL);
  while (true) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // printf("ADC %.0f %.0f %.0f %.0f\r\n", ir_sensor->get_ir_value(0), ir_sensor->get_ir_value(1), ir_sensor->get_ir_value(2), ir_sensor->get_ir_value(3));
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,
                      ir_sensor->get_ir_value(0) >= 2200 ? GPIO_PIN_SET : GPIO_PIN_RESET);  // Left front
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,
                      ir_sensor->get_ir_value(1) >= 2200 ? GPIO_PIN_SET : GPIO_PIN_RESET);  // Right front
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,
                      ir_sensor->get_ir_value(2) >= 2200 ? GPIO_PIN_SET : GPIO_PIN_RESET);  // Left
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                      ir_sensor->get_ir_value(3) >= 2200 ? GPIO_PIN_SET : GPIO_PIN_RESET);  // Right
    // batt.read_batt();
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

// NOLINTBEGIN
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  /* User can add his own implementation to report the HAL error return state
   */

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Stop_IT(&htim10);
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Stop(&htim8, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500);

  __disable_irq();
  while (true) {
  }
  /* USER CODE END Error_Handler_Debug */
}
// NOLINTEND
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
