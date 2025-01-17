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
#include "camera_led.h"
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
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint8_t uart_screen_rx_data;

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
    .priority = (osPriority_t)osPriorityLow3,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow7,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh7,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
    .name = "myTask04",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
    .name = "myTask05",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

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

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
// ��??查任务剩余堆栈内存的函数
void CheckTaskMemoryUsage(TaskHandle_t taskHandle)
{
  // 获取任务的堆栈高水位标记并转换为字节��??
  UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
  printf("任务内存剩余: %lu 字节\r\n", (unsigned long)(stackHighWaterMark * sizeof(StackType_t)));
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
    check_uart_receive_status();
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
    // uasrt_screen_task();

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
  // set_solid_enable(1);

  init_task();
  // osDelay(1000);
  main_task();
  // find_line_calibrate_MPU_PID(0);
  // openmv_data.object_list[0] = 312;
  // openmv_data.object_list[1] = 312;
  // MaterialArea_Task(0);
  // RoughProcessingArea_Task(0); // 粗加工区任务
  // TemporaryStorageArea_Task(1);
  // set_Slider_position(0, 130);
  // osDelay(1000);
  // set_Slider_position(75, 800);
  // osDelay(1000);
  // set_Slider_position(150, 800);
  // osDelay(1000);
  // set_Slider_position(0, 800);
  // osDelay(1000);
  // set_Slider_position_2(0, 100);
  // osDelay(2000);
  // set_Slider_position_2(75, 100);
  // osDelay(2000);
  // set_Slider_position_2(150, 100);

  // Set_Sliding_table_Pos(1);
  // Set_Servo_angle(0, 210);
  // Set_Servo_angle(1, 130);

  // Set_Table_Pos(0);
  // osDelay(2000);
  // Set_Table_Pos(1);
  // osDelay(2000);
  // Set_Table_Pos(2);
  // osDelay(2000);

  // Set_Servo_angle(1, 135);
  // Set_Servo_angle(1, 255);
  // catch_material_in_middle();
  // Put_material(0);
  // Put_material(1);
  // Put_material(2);
  // Get_material(0);
  // Get_material(1);
  // Get_material(2);

  // base_run_distance_base(10, 25, 0, 100); // ?????

  /* Infinite loop */
  for (;;)
  {
    // Set_Servo_angle(2, 0);
    // osDelay(1000);
    // Set_Servo_angle(2, 180);
    // osDelay(1000);

    // for (uint16_t i = 0; i < 270; i++)
    // {
    //   // Set_Servo_angle(1, i);
    //   osDelay(10);
    // }
    // osDelay(1000);
    // Camera_switch_mode(CENTER_POSITION_MODE);
    // osDelay(2000);
    // Camera_switch_mode(CENTER_POSITION_MODE);
    // osDelay(2000);
    // Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
    // osDelay(2000);
    // Camera_switch_mode(HIGH_CENTER_POSITION_MODE);
    // osDelay(2000);
    // Camera_switch_mode(FIND_LINE_MODE);
    // osDelay(2000);
    // Camera_switch_mode(Stacking_MODE);
    // osDelay(2000);
    // Camera_switch_mode(Stacking_MODE);
    // osDelay(2000);
    // Set_Camera_Led_light(20);
    // osDelay(1000);
    // Set_Camera_Led_light(80);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    osDelay(1000);
    // camera_led_huxideng();
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
    osDelay(5);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Function implementing the myTask05 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for (;;)
  {
    uasrt_screen_task();
    osDelay(800);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
