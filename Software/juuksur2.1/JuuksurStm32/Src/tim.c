/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 56000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 53000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* TIM7 clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  int i;
  uint32_t curTick = HAL_GetTick();
  if(htim->Instance == TIM6) { // TIM6 is used to send data to Pi and FC
      // TODO: distance to ground, yaw, pitch, roll
      // STM -> PI
      //piIbus.txCh.channels[i] = 1503;
      piIbus.txCh.channels[0] = tfmini.tfmini.dist;
      // TODO: does this behave correclty? (assigning signed to unsigned, should be twos complement value)
      piIbus.txCh.channels[1] = *((uint16_t*)&telYaw);
      piIbus.txCh.channels[2] = *((uint16_t*)&telPitch);
      piIbus.txCh.channels[3] = *((uint16_t*)&telRoll);
      piIbus.txCh.channels[4] = *((uint16_t*)&telAccX);
      piIbus.txCh.channels[5] = *((uint16_t*)&telAccY);
      piIbus.txCh.channels[6] = *((uint16_t*)&telAccZ);
      piIbus.txCh.channels[7] = rxChannels.channels[5];
      piIbus.txCh.channels[8] = rxChannels.channels[7];
      for(i = 9; i < 14; i++) {
          piIbus.txCh.channels[i] = rxChannels.channels[i];
      }
      ibus_stm32_try_transmit(&piIbus);

      // RX/PI -> FC
      // TODO: real data and switching between pi and rx channels
      uint8_t useManualMode = rxChannels.channels[AUTO_MODE_CH] < 1800;
      if(useManualMode) { // manual mode
          // 0 - roll, 1 - pitch, 2 - throttle, 3 - yaw
          if(rxChannels.channels[CALIB_MODE_CH] < 1800) {
              for(i = 0; i < 14; i++) {
                  rxIbus.txCh.channels[i] = rxChannels.channels[i];
              }
          } else {
              // use only throttle
              rxIbus.txCh.channels[0] = 1500;
              rxIbus.txCh.channels[1] = 1500;
              rxIbus.txCh.channels[2] = rxChannels.channels[2];
              rxIbus.txCh.channels[3] = 1500;
              for(i = 4; i < 14; i++) {
                  rxIbus.txCh.channels[i] = rxChannels.channels[i];
              }
          }
      } else { // autonomous mode
          uint8_t useFailsafe = (curTick - lastPiData) > 1000;
          // init failsafe state when we either go into failsafe mode or we are already in failsafe but just switched to autonomous mode
          if((!failsafePrev && useFailsafe) || (useFailsafe && manualPrev && !useManualMode)) {
              failsafe_pi_ctrl_lost(&failsafe, curTick, tfmini.tfmini.dist);
          }
          if(!useFailsafe) { // HAL uses 1ms tick by default
              for(i = 0; i < 14; i++) {
                  if(i != AUTO_MODE_CH && i != ARM_CH) {
                      rxIbus.txCh.channels[i] = piChannels.channels[i];
                  } else {
                      if(i == AUTO_MODE_CH) {
                          rxIbus.txCh.channels[i] = rxChannels.channels[i];
                      } else { // arm
                          rxIbus.txCh.channels[i] = 2000;
                      }
                  }
              }
          } else {
              // TODO: is this fast enough to be done in this interrupt?
              uint16_t throttle = failsafe_autohover_update2(&failsafe, curTick, tfmini.tfmini.dist, telAccZ);
              // TODO: use PID controller to keep height constant
              // currently we just hope the throttle is OK and set angles to 0
              rxIbus.txCh.channels[0] = IBUS_CHANNEL_DEFAULT;   // roll
              rxIbus.txCh.channels[1] = IBUS_CHANNEL_DEFAULT;   // pitch
              rxIbus.txCh.channels[2] = throttle; // throttle
              rxIbus.txCh.channels[3] = IBUS_CHANNEL_DEFAULT;   // yaw
              rxIbus.txCh.channels[4] = 2000; // arm
              rxIbus.txCh.channels[6] = 1000 + tfmini.tfmini.dist;
              rxIbus.txCh.channels[7] = 1000 + (uint16_t)failsafe.holdHeightCM;
          }
          failsafePrev = useFailsafe;
      }
      manualPrev = useManualMode;
      ibus_stm32_try_transmit(&rxIbus);
  }
  else if(htim->Instance == TIM7) { // TIM7 is used to get telemetry from FC (yaw, pitch, roll etc)
      // NOTE: this should trigger IBUS_CMD_SENSOR_MEASUREMENT in ibus_event callback
      send_telemetry_sensor_request(&telIbus, 1);
  }
}

/* USER CODE END 1 */
