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

#include <cstdint>

#include "./battery.hpp"
#include "./buzzer.hpp"
#include "./controller.hpp"
#include "./data.hpp"
#include "./encoder.hpp"
#include "./flash.hpp"
#include "./global_state.hpp"
#include "./gyro.hpp"
#include "./ir_sensor.hpp"
#include "./maze_run.hpp"
#include "./motor.hpp"
#include "./pid.hpp"
#include "./robot_operation.hpp"
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
using global_state::GlobalState;

namespace {
std::uint8_t run_mode = 0;
std::uint8_t system_mode = 0;

bool led_mode = true;
}  // namespace

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {
    GlobalState::ctrl.status.update_encoder<pwm::Encoder<float, std::int16_t>, pwm::Encoder<float, std::int16_t>, &pwm::Encoder<float, std::int16_t>::read_encoder_value,
                                            &pwm::Encoder<float, std::int16_t>::read_encoder_value>(GlobalState::enc.left, GlobalState::enc.right);
  }
  if (htim == &htim6) {
    switch (GlobalState::test_mode) {
      case param::TestMode::TURN_MODE:
        GlobalState::drive_rec.push_back(data::drive_record(GlobalState::ctrl.status.get_ang_vel(), GlobalState::motor_signal));
        break;
      case param::TestMode::STRAIGHT_MODE:
        GlobalState::drive_rec.push_back(data::drive_record(GlobalState::ctrl.status.get_speed(), GlobalState::motor_signal));
        break;
      default:
        break;
    }
  }
  if (htim == &htim11) {
    GlobalState::ctrl.status.update_wall_sensor([]() { return GlobalState::ir_sensor.get_average_ir_values(); });
    GlobalState::ctrl.status.update_gyro([]() { return GlobalState::gyro.read_gyro().z; });
  }

  if (htim == &htim13) {
    GlobalState::ctrl.update();
    GlobalState::ctrl.drive_motor<pwm::Motor, &pwm::Motor::drive_vcc>(GlobalState::motor.left, GlobalState::motor.right, -1, 1);
  }

  if (htim == &htim7) {
    if (led_mode) {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (run_mode & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ((run_mode >> 1) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ((run_mode >> 2) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ((system_mode & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ((system_mode >> 1) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    GlobalState::batt.read_batt();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle) {
  __disable_irq();
  if (AdcHandle == &hadc2) {
    GlobalState::ir_sensor.ir_sampling();

    if (GlobalState::ir_sensor.ir_selection == adc::IrSensor<std::uint32_t>::IrSelection::FRONT) {
      GlobalState::ir_light_2.ir_flash_stop();
      GlobalState::ir_light_1.ir_flash_start();
    } else {
      GlobalState::ir_light_1.ir_flash_stop();
      GlobalState::ir_light_2.ir_flash_start();
    }
  }
  __enable_irq();
}
void HAL_GPIO_EXTI_Callback(std::uint16_t GPIO_Pin) {
  if (GPIO_Pin == Button1_Pin) {
    system_mode++;
    system_mode %= 3;
  }
  if (GPIO_Pin == Button2_Pin) {
    run_mode++;
    run_mode %= 8;
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
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, nullptr);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_Delay(500);
  GlobalState::buzzer.init();
  GlobalState::ir_sensor.init();
  GlobalState::motor.left.init();
  GlobalState::motor.right.init();
  // printf("stroberry\r\n");
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  // GlobalState::buzzer.beep("ok");
  // HAL_TIM_Base_Start_IT(&htim10);
  // HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim7);
  // HAL_TIM_Base_Start_IT(&htim6);
  GlobalState::ir_light_1.ir_flash_start();  // front
  GlobalState::ir_light_2.ir_flash_start();  // side
  HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (true) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    while (system_mode > 0) {
      if (GlobalState::ir_sensor.get_ir_value(0) >= 12.0f && GlobalState::ir_sensor.get_ir_value(1) >= 12.0f) {
        led_mode = false;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
        GlobalState::gyro.init();
        HAL_Delay(3000);

        switch (system_mode) {
          case 1:
            robot_operation::trueRunMode(run_mode);
            break;

          case 2:
            robot_operation::abjustMode(run_mode);
            break;
        }
        system_mode = 0;
      }

      HAL_Delay(1);
    }

    led_mode = true;

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
//   HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, 400);
//   return len;
// }
// int _io_put_char(int ch)
// {
//   HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 400);
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
  printf("error\r\n");

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Stop_IT(&htim10);
  HAL_TIM_Base_Stop_IT(&htim11);
  HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
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
