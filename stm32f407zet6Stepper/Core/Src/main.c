/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "MecanumMotionControl.h"
#include "mpu.h"
#include "openmv.h"
#include "SliderElevatorControl.h"
#include "servo.h"
#include "ZDT_Stepper.h"
#include "uart_screen.h"
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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/*
ä¸˛ĺŁDMAćĽćśä¸­ć­ĺč°ĺ˝ć°
*/
uint8_t Rx_data[200] = {0};
char received_string[256];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t MPU_RX_flag;
extern uint8_t openmv_rx_flag;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)
  {

    strcpy(received_string, (char *)Rx_data);
    HAL_UART_Transmit(&huart3, Rx_data, Size, HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart1, Rx_data, Size, HAL_MAX_DELAY);

    // fprintf(stdout, "%s\r\n", received_string); // ĺ°ä¸˛?????????????????????1ćĽćśĺ°çć°ćŽčżĺĺ°ä¸˛?????????????????????1
    // HAL_UART_Transmit(&huart5, Rx_data, Size, HAL_MAX_DELAY);
    memset(Rx_data, 0, sizeof(Rx_data));
    HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_data, sizeof(Rx_data) - 1);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
  else if (huart == &huart5) // ćĽćśMPUć°ćŽ
  {
    MPU_UARTE_Rx_Callback(Size);
    MPU_RX_flag = 1 - MPU_RX_flag;
  }
  else if (huart == &huart4) // ćĽćśOpenMVć°ćŽ
  {
    openmv_uart_rx_callback(Size, 4);
    openmv_rx_flag = 1 - openmv_rx_flag;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
  else if (huart == &huart2)
  {
    uart_screen_init();
    openmv_uart_rx_callback(Size, 2);
    openmv_rx_flag = 1 - openmv_rx_flag;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
  else if (huart == &huart3)
  {
    ZDT_Stepper_USRT_RX_callback(Size);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
  }
}

extern uint8_t start_flag;
uint8_t button_fix_flag = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) // ç¨ćˇćéŽ ćä¸ĺ˝éś???????čşäťŞĺč˝Žĺ­çźç ĺ¨
  {
    calibrateAngleToZero(0);
    start_flag = 1;
  }
  else if (GPIO_Pin == GPIO_PIN_3)
  {
    // for (volatile uint32_t i = 0; i < 1000; i++)
    // {
    //   // çŠşĺžŞçŻďźĺżç­ĺžäťĽćść
    // }
    if (button_fix_flag == 0)
    {
      button_fix_flag = 1;
    }
    else
    {
      button_fix_flag = 0;
      for (volatile uint32_t i = 0; i < 1000; i++)
      {
      }
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
      {
        ZDT_Stepper_Enable(0, 1, SYNC_DISABLE);
        // printf("ZDT_Stepper_Enable\r\n");

        for (volatile uint32_t i = 0; i < 1000; i++)
        {
        }
      }
      else
      {
        ZDT_Stepper_Enable(0, 0, SYNC_DISABLE);
        // printf("ZDT_Stepper_Disable\r\n");

        for (volatile uint32_t i = 0; i < 1000; i++)
        {
        }
      }
    }
  }
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) // ĺ¤ć­ćŻĺŞä¸ŞĺŽćśĺ¨
  {
    // ĺ? PWM čĺ˛çťććśĺä¸?äşćä˝?
    Silder_TIM_Callback();
  }
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MPU_Init();
  OLED_Init();
  motor_init();
  Servo_Init();

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_data, 200 - 1); // ĺŻç¨çŠşé˛ä¸­ć­ćĽćś
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);        // ĺłé­DMAäź čžčżĺä¸­ć­
  ZDT_Stepper_init();

  HAL_Delay(50);

  openmv_uart_init();
  HAL_Delay(50);
  uart_screen_init();
  Slider_position_init();
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

  HAL_Delay(50);
  calibrateAngleToZero(0);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // uint8_t send_data[16] = {0x01, 0xFD, 0x01, 0x01, 0xFF, 0x01, 0xFA, 0x27, 0x10, 0x00, 0x00, 0x8C, 0xA0, 0x00, 0x00, 0x6B};
  // HAL_UART_Transmit(&Stepper_Uart_Handle, send_data, 16, HAL_MAX_DELAY);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    HAL_Delay(20);
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3)
  {
    Silder_TIM_Callback();
  }
  /* USER CODE END Callback 1 */
}

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
