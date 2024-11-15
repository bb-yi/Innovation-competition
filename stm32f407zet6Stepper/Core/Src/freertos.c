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
#include "MecanumMotionControl.h"
#include "mpu.h"
#include "SliderElevatorControl.h"
#include "openmv.h"
#include "user_task.h"
#include "servo.h"
#include "ZDT_Stepper.h"
#include "beep.h"
#include "uart_screen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern SYS_STATE_Data sys_state_data;
extern OPENMV_data openmv_data;
extern ZDTStepperData stepperdata_1;
extern ZDTStepperData stepperdata_2;
extern ZDTStepperData stepperdata_3;
extern ZDTStepperData stepperdata_4;
extern ZDTStepperData stepperdata_5;
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
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow2,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
    .name = "myTask04",
    .stack_size = 128 * 4,
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
// �?查任务剩余堆栈内存的函数
void CheckTaskMemoryUsage(TaskHandle_t taskHandle)
{
  // 获取任务的堆栈高水位标记并转换为字节�?
  UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
  printf("任务历史剩余�?小内�?: %lu 字节\r\n", (unsigned long)(stackHighWaterMark * sizeof(StackType_t)));
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
extern uint32_t pulse_count;
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  /* Infinite loop */
  for (;;)
  {
    OLED_display_task();
    uasrt_screen_task();

    // printf("pulse_count=%d\r\n", pulse_count);
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
extern uint8_t delay_time;
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

  init_task();
  // screen_printf_with_quotes("??");
  // screen_printf("page1.t15.txt=\"C\"\xff\xff\xff");

  // osDelay(1000);
  // set_solid_enable(0);
  // Release_material();
  // Catch_material();

  // Get_material_floor(0);
  // Put_material(0);

  // Get_material(1);
  // Get_material(2);
  // Put_material(0);
  // Put_material(1);
  // Put_material(2);
  // Camera_switch_mode(CENTER_POSITION_MODE);
  // find_circle(0);
  // printf("finish\r\n");
  // Get_material(openmv_data.last_identify_color - 1);
  // Get_material(openmv_data.last_identify_color - 1);
  // set_Slider_position(0, 500);
  // osDelay(1000);
  // set_Slider_position(100, 500);
  // osDelay(1000);
  // Set_Stepper_run_T_angle(1, 200, 200, 360, SYNC_ENABLE);
  // Set_Stepper_run_T_angle(2, 200, 200, 360, SYNC_ENABLE);
  // Set_Stepper_run_T_angle(3, 200, 200, 360, SYNC_ENABLE);
  // Set_Stepper_run_T_angle(4, 200, 200, 360, SYNC_ENABLE);

  // base_Horizontal_run_distance(300, 100);
  // osDelay(1000);
  // base_Horizontal_run_distance(-300, 100);s
  // osDelay(1000);
  // ZDT_Stepper_Set_T_position(2, CW, 240, 240, 100, 360 * 2, REL_POS_MODE, SYNC_DISABLE); // ?????????????
  // osDelay(3000);
  // ZDT_Stepper_Set_T_position(2, CCW, 240, 240, 100, 360 * 2, REL_POS_MODE, SYNC_DISABLE); // ?????????????
  // osDelay(3000);

  // main_task();

  // base_run_distance(100, 100);
  // osDelay(1000);
  // base_run_distance(100, 100);
  // osDelay(1000);
  // base_run_distance(-100, 100);
  // osDelay(1000);
  // base_Horizontal_run_distance(100, 100);
  // osDelay(1000);
  // base_Horizontal_run_distance(-100, 100);
  // osDelay(1000);
  // float out_speed = 0;
  // for (uint16_t i = 0; i < 100; i++)
  // {
  //   float alpha = (float)i / 100.0f;
  //   if (alpha > 0.5f)
  //   {
  //     out_speed = S_Curve_Smoothing(0, 100, 0.2, 1 - alpha, 0.5f, 0.01);
  //   }
  //   else
  //   {
  //     out_speed = S_Curve_Smoothing(100, 4, 0.2, 0.5f - alpha, 0.5f, 0.01);
  //   }
  //   printf("alpha=%.2f, out_speed=%.2f\r\n", alpha, out_speed);
  //   osDelay(10);
  // }
  // for (uint8_t j = 0; j < 7; j++)
  // {
  //   for (uint8_t i = 0; i < 1; i++)
  //   {
  //     // base_run_distance_base_fix(0, (j + 1) * 10, 60);
  //     base_run_distance_base_fix((j + 1) * 10, 0, 60);

  //     osDelay(500);
  //   }
  //   osDelay(1000);
  //   for (uint8_t i = 0; i < 1; i++)
  //   {
  //     // base_run_distance_base_fix(0, -(j + 1) * 10, 60);
  //     base_run_distance_base_fix(-(j + 1) * 10, 0, 60);

  //     osDelay(500);
  //   }
  //   osDelay(1000);
  // }
  // base_run_distance_base_fix(0, 100, 60);
  // osDelay(1000);
  // base_run_distance_base_fix(0, -100, 60);
  // osDelay(1000);
  // base_run_distance_base_fix(20, 0, 60);
  // osDelay(1000);
  // base_run_distance_base_fix(0, 180, 60);
  // osDelay(1000);
  // base_run_distance_base_fix(0, -180, 60);
  // osDelay(1000);
  // base_run_distance_base_fix(-20, 0, 60);
  // osDelay(1000);

  // base_speed_control(0, 120, 0, 220);
  // motor_rotation_test();
  // motor_rotation_test();
  // motor_rotation_test();
  // motor_rotation_test();

  // motor_test();

  // base_speed_control(0, 0, 1);
  // base_run_distance_base(20, 20, 0, 40);
  //  osDelay(1000);
  //  base_run_distance_base(-20, -20, 0, 40);

  /* Infinite loop */
  for (;;)
  {

    // motor_stop_all();
    // ZDT_Stepper_Enable(0, Disable, SYNC_DISABLE);
    // Set_Stepper_run_T_angle(1, 200, 200, 360, SYNC_ENABLE);
    // Set_Stepper_run_T_angle(2, 200, 200, 360, SYNC_ENABLE);
    // Set_Stepper_run_T_angle(3, 200, 200, 360, SYNC_ENABLE);
    // Set_Stepper_run_T_angle(4, 200, 200, 360, SYNC_ENABLE);

    // CheckTaskMemoryUsage(myTask03Handle);
    // for (uint8_t i = 0; i < 128; i++)
    // {
    //   Set_display_solid_num(i, i * 2);
    //   osDelay(10);
    // }

    osDelay(100);
    osDelay(delay_time);
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
    if (beep_short_flag == 1)
    {
      beep_short();
    }
    else if (beep_long_flag == 1)
    {
      beep_long();
    }
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
