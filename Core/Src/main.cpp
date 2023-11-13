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
#include "../lib/micromouse-maze-library/include/MazeLib/Maze.h"
#include "../lib/micromouse-maze-library/include/MazeLib/StepMap.h"
#include "./battery.hpp"
#include "./buzzer.hpp"
#include "./controller.hpp"
#include "./encoder.hpp"
#include "./gyro.hpp"
#include "./ir_sensor.hpp"
#include "./maze_run.hpp"
#include "./motor.hpp"
#include "./pid.hpp"
#include "./state.hpp"
#include "data.hpp"
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
std::unique_ptr<pwm::IrLight> ir_light_1, ir_light_2;
std::uint16_t mode = 0;
bool safe_mode = false;
data::drive_records drive_rec;
float motor_signal = 0.0f;
param::TestMode test_mode = param::TestMode::NONE;
bool is_drive_motor = false;
bool is_led_on = true;
bool side_wall_off_allowed = false;

}  // namespace
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {
    ctrl->status.update_encoder<pwm::Encoder<float, int16_t>, pwm::Encoder<float, int16_t>, &pwm::Encoder<float, int16_t>::read_encoder_value,
                                &pwm::Encoder<float, int16_t>::read_encoder_value>(*(enc.left), *(enc.right));
    ctrl->status.update_wall_sensor([]() { return ir_sensor->get_ir_values(); },
                                    []() {
                                      ir_light_2->ir_flash_stop();
                                      ir_light_1->ir_flash_start();
                                    },
                                    []() {
                                      ir_light_1->ir_flash_stop();
                                      ir_light_2->ir_flash_start();
                                    });
  }
  if (htim == &htim6) {
    switch (test_mode) {
      case param::TestMode::TURN_MODE:
        drive_rec.push_back(data::drive_record(ctrl->status.get_ang_vel(), motor_signal));
        break;
      case param::TestMode::STRAIGHT_MODE:
        drive_rec.push_back(data::drive_record(ctrl->status.get_speed(), motor_signal));
        break;
      default:
        break;
    }
  }
  if (htim == &htim11) {
    if (is_drive_motor) {
      ctrl->update();
      ctrl->drive_motor<pwm::Motor, &pwm::Motor::drive_vcc>(*(motor.left), *(motor.right), -1, 1);
    }
    ctrl->status.update_gyro([]() { return gyro->read_gyro().z; });
  }
  if (htim == &htim7) {
    if (is_led_on) {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (mode & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ((mode >> 1) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ((mode >> 2) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (safe_mode ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
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
    safe_mode = !safe_mode;
  }
  if (GPIO_Pin == Button2_Pin) {
    mode++;
    mode %= 8;
  }
}

void HAL_SYSTICK_Callback(void) {  // 1kHz
  // This is system clock timer
}

void maze_run::robot_move(Direction dir) {
  int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir.byte == NORTH << robot_dir_index) break;
    robot_dir_index++;
  }

  int8_t next_dir_index = 0;
  while (1) {
    if (dir.byte == NORTH << next_dir_index) break;
    next_dir_index++;
  }

  int8_t dir_diff = next_dir_index - robot_dir_index;
  // 直進
  if (dir_diff == 0) {
    if (is_start_block) {
      batt->monitoring_state = false;
      ctrl->back_1s();

      ctrl->reset();
      HAL_Delay(1);
      ctrl->straight(180.0 - 40.0, 400, 800, 0.0);
      batt->monitoring_state = true;
      is_start_block = false;
    } else
      ctrl->straight(180.0, 400, 800, 0.0);

  }
  // 右
  else if (dir_diff == 1 || dir_diff == -3) {
    ctrl->side_wall_control = false;
    ctrl->straight(90.0, 400, 800, 0.0);
    HAL_Delay(1);
    ctrl->turn(-90, 540, 720);
    HAL_Delay(1);
    ctrl->straight(90.0, 400, 800, 0.0);
    ctrl->side_wall_control = true;
  }
  // 左
  else if (dir_diff == -1 || dir_diff == 3) {
    ctrl->side_wall_control = false;
    ctrl->straight(90.0, 400, 800, 0.0);
    HAL_Delay(1);
    ctrl->turn(90, 540, 720);
    HAL_Delay(1);
    ctrl->straight(90.0, 400, 800, 0.0);
    ctrl->side_wall_control = true;
  }
  // 180度ターン
  else {
    if (prev_wall_cnt == 3) {
      ctrl->straight(90.0, 400, 800, 0.0);
      ctrl->turn(180, 540, 720);
      batt->monitoring_state = false;
      ctrl->back_1s();

      ctrl->reset();
      HAL_Delay(1);
      ctrl->straight(180.0 - 40.0, 400, 800, 0.0);
      batt->monitoring_state = true;
    } else {
      ctrl->straight(90.0, 400, 800, 0.0);
      ctrl->turn(180, 540, 720);
      ctrl->straight(90.0, 400, 800, 0.0);
    }
  }

  robot_dir = dir;
  // robot positionをdirの分だけ動かす
  if (NORTH == dir.byte) {
    robot_position += IndexVec::vecNorth;
  } else if (SOUTH == dir.byte) {
    robot_position += IndexVec::vecSouth;
  } else if (EAST == dir.byte) {
    robot_position += IndexVec::vecEast;
  } else if (WEST == dir.byte) {
    robot_position += IndexVec::vecWest;
  }
  return;
}

Direction maze_run::get_wall_data() {
  Direction wall;

  uint8_t wall_front = 0;
  uint8_t wall_left = 0;
  uint8_t wall_right = 0;
  for (int i = 0; i < 5; i++) {
    wall_front += ctrl->status.get_front_wall();
    wall_left += ctrl->status.get_left_wall();
    wall_right += ctrl->status.get_right_wall();
    HAL_Delay(100);
  }
  bool is_front_wall = wall_front >= 3;
  bool is_left_wall = wall_left >= 3;
  bool is_right_wall = wall_right >= 3;
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, is_left_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, is_front_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, is_front_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, is_right_wall ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (!side_wall_off_allowed) {
    if (!is_left_wall || !is_right_wall) {
      ctrl->side_wall_control = false;
    } else {
      ctrl->side_wall_control = true;
    }
  }
  int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir.byte == NORTH << robot_dir_index) break;
    robot_dir_index++;
  }

  if (is_front_wall) {
    wall.byte |= robot_dir;
  }

  if (is_right_wall) {
    wall.byte |= NORTH << (robot_dir_index + 1) % 4;
  }

  if (is_left_wall) {
    if (robot_dir_index == 0)
      wall.byte |= WEST;
    else
      wall.byte |= NORTH << (robot_dir_index - 1) % 4;
  }

  prev_wall_cnt = wall.nWall();

  return wall;
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
  /* USER CODE BEGIN 2 */
  setbuf(stdout, nullptr);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_Delay(4000);
  gyro = std::make_unique<spi::Gyro>();
  HAL_Delay(400);
  batt = std::make_unique<adc::Battery<float, uint32_t>>(&hadc1);

  enc.right = std::make_unique<pwm::Encoder<float, int16_t>>(TIM8);
  enc.left = std::make_unique<pwm::Encoder<float, int16_t>>(TIM1);
  motor.left = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4);
  motor.right = std::make_unique<pwm::Motor>(&htim4, &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2);
  ctrl = std::make_unique<state::Controller<float, state::Status<float>, state::Pid<float>>>();
  ir_sensor = std::make_unique<adc::IrSensor<uint32_t>>(&hadc2, 4, 160, 10);
  ir_light_1 = std::make_unique<pwm::IrLight>(&htim9, TIM_CHANNEL_1);
  ir_light_2 = std::make_unique<pwm::IrLight>(&htim9, TIM_CHANNEL_2);

  pwm::Buzzer buzzer(&htim12, TIM_CHANNEL_2);
  printf("stroberry\r\n");
  buzzer.beep("ok");
  // HAL_TIM_Base_Start_IT(&htim10);
  // HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim7);
  // HAL_TIM_Base_Start_IT(&htim6);
  ir_light_1->ir_flash_start();
  ir_light_2->ir_flash_start();
  HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  std::cout << "cout" << std::endl;
  while (true) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    while (safe_mode) {
      if (ir_sensor->get_ir_value(0) >= 1e4 && ir_sensor->get_ir_value(1) >= 1e4 && ir_sensor->get_ir_value(2) >= 1e4 && ir_sensor->get_ir_value(3) >= 1e4) {
        safe_mode = false;
        is_led_on = false;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
        HAL_Delay(4000);
        switch (mode) {
          case 6: {
            is_drive_motor = true;
            HAL_TIM_Base_Start_IT(&htim10);
            HAL_TIM_Base_Start_IT(&htim11);
            // ctrl->turn(3600, 540, 720);
            // ctrl->front_wall_control = true;

            ctrl->turn(3600, 540, 720);
            // ctrl->turn(-90, 540, 180);
          } break;
          case 5: {
            is_drive_motor = true;
            HAL_TIM_Base_Start_IT(&htim10);
            HAL_TIM_Base_Start_IT(&htim11);
            // ctrl->front_wall_control = true;
            side_wall_off_allowed = true;
            ctrl->back_1s();
            ctrl->straight(180.0 * 8 - 40.0, 400, 800, 0.0);
          } break;
          // case 3: {
          //   Mseq mseq(7);
          //   HAL_TIM_Base_Start_IT(&htim10);
          //   HAL_TIM_Base_Start_IT(&htim6);
          //   HAL_TIM_Base_Start_IT(&htim11);
          //   HAL_Delay(1);
          //   test_mode = param::TestMode::TURN_MODE;
          //   for (int i = 0; i < (1 << 7) - 1; ++i) {
          //     uint8_t signal = mseq.update();
          //     motor_signal = (static_cast<float>(signal) - 0.5f) * 2 * 2.5f;
          //     motor.left->drive_vcc(motor_signal);
          //     motor.right->drive_vcc(motor_signal);
          //     HAL_Delay(400);
          //   }
          //   test_mode = param::TestMode::NONE;
          //   HAL_TIM_Base_Stop_IT(&htim10);
          //   HAL_TIM_Base_Stop_IT(&htim11);
          //   HAL_TIM_Base_Stop_IT(&htim6);
          //   motor.left->drive_vcc(0);
          //   motor.right->drive_vcc(0);

          //   printf("%d\r\n", drive_rec.size() * sizeof(data::twist));
          //   if (drive_rec.size() * sizeof(data::twist) > BACKUP_FLASH_SECTOR_SIZE) {
          //     HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
          //     Error_Handler();
          //   }

          //   std::copy(drive_rec.begin(), drive_rec.end(), reinterpret_cast<data::drive_record *>(flash::work_ram));

          //   flash::Store();
          //   buzzer.beep("save");

          // } break;

          // case 4: {
          //   Mseq mseq(6);
          //   HAL_TIM_Base_Start_IT(&htim10);
          //   HAL_TIM_Base_Start_IT(&htim6);
          //   HAL_TIM_Base_Start_IT(&htim11);
          //   HAL_Delay(1);
          //   test_mode = param::TestMode::STRAIGHT_MODE;
          //   for (int i = 0; i < (1 << 6) - 1; ++i) {
          //     uint8_t signal = mseq.update();
          //     motor_signal = (static_cast<float>(signal) - 0.5f) * 2 * 2.5f;
          //     motor.left->drive_vcc(-motor_signal);
          //     motor.right->drive_vcc(motor_signal);
          //     HAL_Delay(400);
          //     printf("%f, %f\r\n", enc.left->get_incremental_degrees(), enc.right->get_incremental_degrees());
          //   }
          //   test_mode = param::TestMode::NONE;
          //   HAL_TIM_Base_Stop_IT(&htim10);
          //   HAL_TIM_Base_Stop_IT(&htim11);
          //   HAL_TIM_Base_Stop_IT(&htim6);
          //   motor.left->drive_vcc(0);
          //   motor.right->drive_vcc(0);
          //   printf("%d\r\n", drive_rec.size() * sizeof(data::twist));
          //   if (drive_rec.size() * sizeof(data::twist) > BACKUP_FLASH_SECTOR_SIZE) {
          //     HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
          //     Error_Handler();
          //   }

          //   std::copy(drive_rec.begin(), drive_rec.end(), reinterpret_cast<data::drive_record *>(flash::work_ram));

          //   flash::Store();
          //   buzzer.beep("save");

          // } break;
          // case 5: {
          //   flash::Load();
          //   std::copy((data::drive_record *)flash::work_ram, (data::drive_record *)flash::work_ram + 4000, std::back_inserter(drive_rec));
          //   printf("speed, signal\r\n");
          //   for (auto rec : drive_rec) {
          //     printf("%f, %f\r\n", rec.speed, rec.signal);

          //     HAL_Delay(1);
          //   }
          //   buzzer.beep("done");
          // } break;
          // case 6: {
          //   // HAL_TIM_Base_Start_IT(&htim10);
          //   HAL_TIM_Base_Start_IT(&htim11);
          //   while (1) {
          //     uint32_t ir_value[4];
          //     ir_light_1->ir_flash_start();
          //     ir_light_2->ir_flash_stop();
          //     HAL_Delay(10);

          //     for (uint8_t i = 0; i < 2; ++i) {
          //       ir_value[i] = ir_sensor->get_ir_value(i);
          //     }
          //     ir_light_1->ir_flash_stop();
          //     ir_light_2->ir_flash_start();
          //     HAL_Delay(10);
          //     for (uint8_t i = 2; i < 4; ++i) {
          //       ir_value[i] = ir_sensor->get_ir_value(i);
          //     }

          //     printf("ir: %ld,%ld, %ld, %ld\r\n", ir_value[0], ir_value[1], ir_value[2], ir_value[3]);
          //     HAL_Delay(1);
          //     HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ctrl->status.get_left_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
          //     HAL_GPIO_WritePin(LED6_GPIO_Port, LED5_Pin, ctrl->status.get_front_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
          //     HAL_GPIO_WritePin(LED6_GPIO_Port, LED2_Pin, ctrl->status.get_front_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
          //     HAL_GPIO_WritePin(LED6_GPIO_Port, LED1_Pin, ctrl->status.get_right_wall() ? GPIO_PIN_SET : GPIO_PIN_RESET);
          //   }

          // } break;
          case 0: {
            is_drive_motor = true;
            HAL_TIM_Base_Start_IT(&htim10);
            HAL_TIM_Base_Start_IT(&htim11);
            side_wall_off_allowed = false;
            maze_run::search_run();
            // MazeLib::Maze maze;
            // MazeLib::Positions goals;
            // goals.push_back(MazeLib::Position(0, 1));
            // maze.setGoals(goals);
            // // HAL_Delay(4000);
            // // maze_run::SearchRun(
            // //     maze, []() { return ctrl->status.get_front_wall(); }, []() { return ctrl->status.get_left_wall(); }, []() { return ctrl->status.get_right_wall(); });

            // // HAL_TIM_Base_Stop_IT(&htim10);
            // // HAL_TIM_Base_Stop_IT(&htim11);
            // const MazeLib::WallRecords &wall_records = maze.getWallRecords();
            // for (auto x : wall_records) {
            //   std::cout << x << std::endl;
            // }
            // std::copy(wall_records.begin(), wall_records.end(), reinterpret_cast<MazeLib::WallRecord *>(flash::work_ram));
            // flash::Store();
            // buzzer.beep("save");
            // is_drive_motor = false;
            // while (1) {
            //   maze.print();
            // }

          } break;
          case 1: {
            is_drive_motor = true;
            HAL_TIM_Base_Start_IT(&htim10);
            HAL_TIM_Base_Start_IT(&htim11);
            side_wall_off_allowed = true;
            maze_run::search_run();
            // MazeLib::WallRecords wall_records;
            // flash::Load();
            // std::copy((MazeLib::WallRecord *)flash::work_ram, (MazeLib::WallRecord *)flash::work_ram + 4000, std::back_inserter(wall_records));
            // for (auto x : wall_records) {
            //   std::cout << x << std::endl;
            // }
          } break;
          case 2: {
            is_drive_motor = true;
            HAL_TIM_Base_Start_IT(&htim10);
            HAL_TIM_Base_Start_IT(&htim11);
            ctrl->maekabe = false;
            side_wall_off_allowed = false;
            maze_run::search_run();
          } break;
          case 3: {
            is_drive_motor = true;
            HAL_TIM_Base_Start_IT(&htim10);
            HAL_TIM_Base_Start_IT(&htim11);
            ctrl->maekabe = false;
            side_wall_off_allowed = true;
            maze_run::search_run();
          }

          default:
            break;
        }
      }

      HAL_Delay(1);
    }
    is_led_on = true;

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
