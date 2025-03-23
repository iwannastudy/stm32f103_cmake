/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */
#include "common.h"



static EventGroupHandle_t wdgEventGroup;
static SemaphoreHandle_t eventMutex;
static EventBits_t wdgWaitEvents = 0;
/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
  wdgEventGroup = xEventGroupCreate();
  if(wdgEventGroup == NULL)
  {
      ERRORPOINT("IWDG event group create failed");
  }

  eventMutex = xSemaphoreCreateMutex();
  if(eventMutex == NULL)
  {
      ERRORPOINT("IWDG event mutex create failed");
  }
  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

uint32_t watch_dog_register(void)
{
    uint8_t index = 0;
    uint8_t isUsed = 0;

    if(!wdgEventGroup)
    {
        return 0;
    }

    if(eventMutex)
    {
        xSemaphoreTake(eventMutex, portMAX_DELAY);
    }

    for(index = 0; (index < (sizeof(EventBits_t) * 8)); index++)
    {
        isUsed = (wdgWaitEvents >> index) & 1;
        if(isUsed == 0)    // this bit event is available
        {
            wdgWaitEvents |= (1 << index);
            break;
        }
    }

    if(eventMutex)
    {
        xSemaphoreGive(eventMutex);
    }

    return (1 << index);
}

void watch_dog_unregister(uint32_t event)
{
    if(!wdgEventGroup)
    {
        return 0;
    }

    if(eventMutex)
    {
        xSemaphoreTake(eventMutex, portMAX_DELAY);
    }

    wdgWaitEvents &= ~event;

    if(eventMutex)
    {
        xSemaphoreGive(eventMutex);
    }
}

void watch_dog_feed_event(uint32_t event)
{
    xEventGroupSetBits(wdgEventGroup, event);
}

void watch_dog_check_feed(void)
{
    if(wdgEventGroup && wdgWaitEvents)
    {
      xEventGroupWaitBits(wdgEventGroup, wdgWaitEvents, 
                          pdTRUE, pdTRUE, portMAX_DELAY);
    }

    HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END 1 */
