/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "ins.h"
#include "mpu6050.h"
#include "OLED.h"
#include "OLED_Font.h"
#include "motor.h"
#include "../Inc/PID_degree.h"
#include "PID_speed.h"
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
#define RX_BUF_SIZE 32
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int count = 0;
float goal_degree = 0;
float speed_ref_1 = 0;
float Kp1 = 2.1, Ki1 = 0.4, Kd1 = 0.5;
float degree_error;
float last_roll = 0;
float degree_integral = 0;
float last_degree_error = 0;
float vary_speed_degree = 0;
float motor_speed = 0;
float target_speed = 0;
float speed_error = 0;
float real_speed = 0;
float speed_integral = 0;
float vary_speed_speed = 0;
float last_speed_error = 0;
float speed_ref_2 = 0;
float Kp2 = 4.2, Ki2 = 0.7, Kd2 = 0.12;
float last_real_speed = 0;
float goal_degree_2 = 0;
uint8_t rx_buf[RX_BUF_SIZE];
uint16_t rx_index = 0;
uint8_t rx_byte = 0;
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  ins_init();
  HAL_Delay(20);
  OLED_Init();
  OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
  char buff[256];
  while (1)
  {
    // 获取陀螺仪数据的指针
    ins_data_t *ins = ins_data();
    sprintf(buff, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
            real_speed, motor_speed, speed_integral, target_speed, ins->roll, goal_degree, degree_integral);
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff),HAL_MAX_DELAY);
    OLED_ShowString(0, 0, "Speed:", 16, 0);
    OLED_Showdecimal(56, 0, real_speed, 3, 2, 16, 0);
    OLED_ShowString(0, 2, "Degree:", 16, 0);
    OLED_Showdecimal(56, 2, ins->roll, 3, 2, 16, 0);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    //测速
    count = __HAL_TIM_GET_COUNTER(&htim3);
    if (count <= 32768)
    real_speed = 2.3 * count;//乘2.3是为了使real_speed和target_speed的量纲保持一致
    else real_speed = 2.3 * (count - 65536);
    //傻逼滤波，加上净是拖后腿:(
    //real_speed = last_real_speed * 0.8f + real_speed * 0.2f;
    last_real_speed = real_speed;
    // 更新 MPU6050 数据（陀螺仪 + 加速度计 + 姿态解算）
    ins_update();
    goal_degree_2 = goal_degree + 16.5;
    motor(PID_speed(PID_degree(goal_degree_2)));
    __HAL_TIM_SET_COUNTER(&htim3, 0);
  }
}
//VOFA+控制目标角度
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 收到结束符（回车或换行），开始转换
    if (rx_byte == '\r' || rx_byte == '\n')
    {
      if (rx_index > 0)                  // 缓冲区里有数据才转换
      {
        rx_buf[rx_index] = '\0';         // 字符串结束
        goal_degree = (float)atof((char *)rx_buf);  // 转成浮点数（支持整数和小数）
      }
      rx_index = 0;                      // 清空，准备下次接收
    }
    else
    {
      // 普通字符，存入缓冲区
      if (rx_index < RX_BUF_SIZE - 1)
      {
        rx_buf[rx_index++] = rx_byte;
      }
      else
      {
        rx_index = 0;                    // 防止溢出，直接丢弃
      }
    }

    // 继续接收下一个字节
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}
//归零按钮
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  goal_degree = 0;
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
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
