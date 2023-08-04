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
#include "../lib/Mseq/Mseq.h"
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
namespace {
std::unique_ptr<spi::Gyro> gyro;
parts::wheel<std::unique_ptr<pwm::Encoder<float, int16_t>>, std::unique_ptr<pwm::Encoder<float, int16_t>>> enc;
parts::wheel<std::unique_ptr<pwm::Motor>, std::unique_ptr<pwm::Motor>> motor;
std::unique_ptr<state::Controller<float, state::Status<float>, state::Pid<float>>> ctrl;
std::unique_ptr<adc::IrSensor<uint32_t>> ir_sensor;
std::unique_ptr<adc::Battery<float, uint32_t>> batt;
std::uint16_t mode = 0;
bool safe_mode = false;
}  // namespace
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {
    ctrl->status.update_encoder<pwm::Encoder<float, int16_t>, pwm::Encoder<float, int16_t>, &pwm::Encoder<float, int16_t>::read_encoder_value,
                                &pwm::Encoder<float, int16_t>::read_encoder_value>(*(enc.left), *(enc.right));
  }
  if (htim == &htim11) {
    ctrl->update();
    ctrl->drive_motor<pwm::Motor, &pwm::Motor::drive_vcc>(*(motor.left), *(motor.right), -1, 1);
    ctrl->status.update_gyro([]() { return gyro->read_gyro().z; });
  }
  if (htim == &htim7) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (mode & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ((mode >> 1) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ((mode >> 2) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (safe_mode ? GPIO_PIN_SET : GPIO_PIN_RESET));
    batt->read_batt();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle) {
  if (AdcHandle == &hadc2) {
    ir_sensor->ir_sampling();
  }
}
void HAL_GPIO_EXTI_Callback(std::uint16_t GPIO_Pin) {
  if (GPIO_Pin == Button1_Pin) {
    mode++;
    mode %= 8;
  }
  if (GPIO_Pin == Button2_Pin) {
    safe_mode = !safe_mode;
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
  MX_TIM2_Init();
  MX_TIM12_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, nullptr);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_Delay(1000);
  gyro = std::make_unique<spi::Gyro>();
  HAL_Delay(100);
  batt = std::make_unique<adc::Battery<float, uint32_t>>(&hadc1);

  enc.right = std::make_unique<pwm::Encoder<float, int16_t>>(TIM8);
  enc.left = std::make_unique<pwm::Encoder<float, int16_t>>(TIM2);
  motor.left = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4);
  motor.right = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2);
  ctrl = std::make_unique<state::Controller<float, state::Status<float>, state::Pid<float>>>();
  ir_sensor = std::make_unique<adc::IrSensor<uint32_t>>(&hadc2, 4, 160, 10);
  pwm::IrLight ir_light_1(&htim9, TIM_CHANNEL_1), ir_light_2(&htim9, TIM_CHANNEL_2);
  pwm::Buzzer buzzer(&htim12, TIM_CHANNEL_2);
  printf("stroberry\r\n");
  buzzer.beep("ok");
  // HAL_TIM_Base_Start_IT(&htim10);
  // HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim7);
  ir_light_1.ir_flash_start();
  ir_light_2.ir_flash_start();
  HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (true) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (safe_mode) {
      while (true) {
        if (ir_sensor->get_ir_value(0) >= 1e8 && ir_sensor->get_ir_value(1) >= 1e8 && ir_sensor->get_ir_value(2) >= 1e8 && ir_sensor->get_ir_value(3) >= 1e8) {
          switch (mode) {
            case 1: {
              HAL_TIM_Base_Start_IT(&htim10);
              HAL_TIM_Base_Start_IT(&htim11);
              ctrl->turn(90, 360, 180);
            } break;
            case 2: {
              Mseq mseq(7);
              for (int i = 0; i < (1 << 7) - 1; ++i) {
                mseq.update();
              }
            } break;

            default:
              break;
          }
          safe_mode = false;
        }

        HAL_Delay(1);
      }
    }
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
  printf("error\r\n");

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Stop_IT(&htim10);
  HAL_TIM_Base_Stop_IT(&htim11);
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
