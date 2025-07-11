/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "iwdg.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "usbd_cdc_if.h"
#include <string.h>
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
TaskHandle_t systatTH;
TaskHandle_t watchdogTH;
TaskHandle_t usbcdcTestTH;
TaskHandle_t usbcdcRxShowTH;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void watchdog_task(void *pvParameters)
{
    (void)pvParameters; // Suppress unused parameter warning
    
    while(1)
    {
        for(uint8_t i = 3; i > 0; i--) 
        {
            LED_ON; vTaskDelay(100);
            LED_OFF; vTaskDelay(100);
        }

        watch_dog_check_feed();
    }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
    (void)xTask; // Suppress unused parameter warning
    CHECKPOINTA("task:%s stack is overflowed!", pcTaskName);
}

void systat_task(void *pvParameters)
{
    (void)pvParameters; // Suppress unused parameter warning
    size_t remainOSHeapBytes = 0;
    size_t minOSHeapBytes = 0;
    char pcWriteBuffer[300];

    uint32_t wdgEvent = watch_dog_register();
    CHECKPOINTA("systat_task start, get watch dog event id: 0x%X", wdgEvent);
    
    while(1)
    {
        CHECKPOINTA("\r\n\r\n");
        CHECKPOINTA("=================================================");
        CHECKPOINTA("taskName  taskState  priority  freeStack  taskNum");
        vTaskList(pcWriteBuffer);
        CHECKPOINTA("%s\r\n", pcWriteBuffer);
        
        //CHECKPOINTA("\r\n taskName  runningCount  usage\r\n");
        //vTaskGetRunTimeStats(pcWriteBuffer);
        //CHECKPOINTA("%s\r\n", pcWriteBuffer);
        
        remainOSHeapBytes = xPortGetFreeHeapSize();
        minOSHeapBytes = xPortGetMinimumEverFreeHeapSize();
        CHECKPOINTA("FreeRTOS remain heap size: %d", remainOSHeapBytes);
        CHECKPOINTA("FreeRTOS minimum ever heap size: %d", minOSHeapBytes);
        CHECKPOINTA("=================================================");
        CHECKPOINTA("\r\n\r\n");

        watch_dog_feed_event(wdgEvent);
        vTaskDelay(5000);
    }
}

void usb_cdc_test_task(void *pvParameters)
{
    (void)pvParameters;
    const char *msg = "USB CDC Test\r\n";
    for(;;)
    {
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        vTaskDelay(1000);
    }
}

extern uint8_t UserRxBufferFS[];
extern volatile uint32_t usb_cdc_rx_len;

void usb_cdc_rx_show_task(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t last_rx_len = 0;
    for(;;)
    {
        // TODO: Use a more efficient way to check for new data
        // This is a simple polling method, which may not be optimal.
        if(usb_cdc_rx_len > 0 && usb_cdc_rx_len != last_rx_len) {
            CHECKPOINTA("USB RX: %.*s", usb_cdc_rx_len, UserRxBufferFS);
            last_rx_len = usb_cdc_rx_len;
        }
        vTaskDelay(100);
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
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  xTaskCreate(watchdog_task, "wdg", 64, NULL, SYS_CTRL_PORIRITY, &watchdogTH);
  xTaskCreate(systat_task, "systat", 256, NULL, LOWEST_PORIORIT, &systatTH);
  xTaskCreate(usb_cdc_test_task, "usbcdc", 128, NULL, LOWEST_PORIORIT, &usbcdcTestTH);
  xTaskCreate(usb_cdc_rx_show_task, "usbcdc_rx", 128, NULL, LOWEST_PORIORIT, &usbcdcRxShowTH);

  vTaskStartScheduler();

  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
     ERRORPOINT("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
