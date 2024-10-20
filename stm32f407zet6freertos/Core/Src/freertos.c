/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "adc_data.h"
#include "motor.h"
#include "mpu.h"
#include "stepper_a4988.h"
#include "openmv.h"
#include "user_task.h"
#include "servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern SYS_STATE_Data sys_state_data;
extern MOTOR_Encoder motor_encoder;
extern OPENMV_data openmv_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
    .name = "myTask04",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */

/**
 * @brief �???????????查剩余内内存
 *
 * @param task
 */
void check_stack_usage(TaskHandle_t task)
{
  UBaseType_t stack_water_mark = uxTaskGetStackHighWaterMark(task);

  printf("Stack high water mark: %u\n", stack_water_mark);
}

/**
 * @brief 获取当前时间
 *
 * @return float  单位：秒
 */
float get_time()
{
  return (float)HAL_GetTick() / 1000.0f;
}
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for (;;)
  {
    // if (HAL_GetTick() / 1000 % 8 >= 4)
    // printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n", radiansToDegrees(sys_state_data.real_Roll), radiansToDegrees(sys_state_data.real_Pitch), radiansToDegrees(sys_state_data.real_Yaw));
    // printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n", radiansToDegrees(sys_state_data.Roll), radiansToDegrees(sys_state_data.Pitch), radiansToDegrees(sys_state_data.FullYaw));
    encoders_read();
    if (Get_IMU_Is_Working())
    {
    }
    else
    {
      MPU_Init();
    }

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
// OLED显示任务
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  /* Infinite loop */
  for (;;)
  {
    OLED_display_task();
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
extern int stepper_count;
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  // for (;;)
  // {
  //   if (Get_IMU_Is_Working())
  //   {
  //     break;
  //   }
  // }
  stepper_enable(0);
  Set_Table_Pos(0);
  Set_Sliding_table_Pos(0);
  Camera_switch_mode(FIND_LINE_MODE);
  // Slider_position_init();
  // osDelay(1000);
  // set_Slider_position(150, 7);

  osDelay(1000);
  for (;;)
  {

    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
    {
      break;
    }
  }
  // Camera_switch_mode(FIND_LINE_MODE);
  Set_Sliding_table_Pos(0);
  osDelay(1000);
  Slider_position_init();
  osDelay(1000);
  set_Slider_position(150, 7);
  // main_task();
  // base_run_distance(20, 1.5);
  // base_run_distance(-20, 1.5);
  //  base_run_angle(90, 1);

  // find_line_calibrate_MPU();
  // MaterialArea_Task();

  RoughProcessingArea_Task();
  // base_control(0, 0, 10);

  // // Get_material(0);
  // // osDelay(1000);
  // Put_material(2);
  // osDelay(1000);

  // base_run_distance(1, 2);
  // osDelay(1000);
  // base_run_distance(-1, 2);
  // osDelay(1000);

  // base_Horizontal_run_distance(20, 2);
  // osDelay(1000);
  // base_Horizontal_run_distance(-20, 2);
  // osDelay(1000);
  // osDelay(1000);
  // base_run_angle(3, 3);

  // osDelay(1000);
  /* Infinite loop */
  float test_speed;
  float temp = 2.5;
  for (;;)
  {
    // set_motor_speed(1, 4);
    if (fmod(get_time(), 5.0) < 2.5)
    {
      test_speed = temp;
    }
    else
    {
      test_speed = -temp;
    }
    // set_motor_speed(0, test_speed);
    // set_motor_speed(1, test_speed);
    // set_motor_speed(2, test_speed);
    // set_motor_speed(3, test_speed);
    // base_control(0, test_speed, 0);
    // Catch_material();
    // osDelay(1000);
    // Release_material();
    // osDelay(1000);
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for (;;)
  {
    // RemoteControl_Task();
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
