/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "i2c.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 36000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 36000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16-1;
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

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16-1;
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

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
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
    HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
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
  static uint32_t tempModePrev;
  static uint32_t tempMode;
  static uint16_t controlModePrev;
  tempMode = 0;
  if(htim->Instance == TIM3) {
      HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_I2C_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, mpu.rawData, 14);
  }
  else if(htim->Instance == TIM6) { // TIM6 is used to send data to Pi and FC
      // TODO: clean up this mess!!!! (this is from Robotex 2019 crunch)
      uint16_t *heightF = (uint16_t*)&(hEst.height);
      uint16_t *hVelF = (uint16_t*)&(hEst.heightVel);
      // STM -> PI
      piIbus.txCh.channels[0] = tfmini.tfmini.dist;
      int16_t pitch16 = (int16_t)(mpuFusion.pitch*100.0f);
      int16_t roll16 = (int16_t)(mpuFusion.roll*100.0f);
      /*int16_t accX16 = (int16_t)(mpu.acc[0]*100.0f);
      int16_t accY16 = (int16_t)(mpu.acc[1]*100.0f);
      int16_t accZ16 = (int16_t)(mpu.acc[2]*100.0f);*/
      int16_t accX16 = (int16_t)accWXB;
      int16_t accY16 = (int16_t)accWYB;
      int16_t accZ16 = (int16_t)accWZB;
      int16_t velX16 = (int16_t)(vEst.velX*10.0f);
      int16_t velY16 = (int16_t)(vEst.velY*10.0f);
      piIbus.txCh.channels[1] = *((uint16_t*)&velX16);
      piIbus.txCh.channels[2] = *((uint16_t*)&pitch16);
      piIbus.txCh.channels[3] = *((uint16_t*)&roll16);
      piIbus.txCh.channels[4] = *((uint16_t*)&accX16);
      piIbus.txCh.channels[5] = *((uint16_t*)&accY16);
      piIbus.txCh.channels[6] = *((uint16_t*)&accZ16);
      piIbus.txCh.channels[7] = rxChannels.channels[5]; // auto mode
      piIbus.txCh.channels[8] = rxChannels.channels[7]; // landing mode
      piIbus.txCh.channels[9] = heightF[0];
      piIbus.txCh.channels[10] = heightF[1];
      piIbus.txCh.channels[11] = hVelF[0];
      piIbus.txCh.channels[12] = hVelF[1];
      piIbus.txCh.channels[13] = *((uint16_t*)&velY16); // X axis on channel 1 ! 
      /*for(i = 13; i < 14; i++) {
          piIbus.txCh.channels[i] = rxChannels.channels[i];
      }*/
      ibus_stm32_try_transmit(&piIbus);

      // RX/PI -> FC
      // TODO: real data and switching between pi and rx channels
      uint8_t useManualMode = rxChannels.channels[AUTO_MODE_CH] < 1800;
      flyMode = 0;
      if(useManualMode) { // manual mode
          // 0 - roll, 1 - pitch, 2 - throttle, 3 - yaw
          if(rxChannels.channels[CALIB_MODE_CH] < 1800) {
              for(i = 0; i < 14; i++) {
                  rxIbus.txCh.channels[i] = rxChannels.channels[i];
              }
          } else { // manual calibration mode
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
          // autonomous flight if everything ok and running
          if(!useFailsafe) { // HAL uses 1ms tick by default

              // roll and pitch control
              if(hEst.height > 20.0f) { // attempt to control velocity only if above 50 cm height
                  tempMode = 1;
                  if(tempModePrev == 0) {
                      velocity_controller_init(&vCtrl);
                  }
                  // if lever is down use 0 for velocity setpoints
                  // (otherwise the targets should be calculated by Pi)
                  if(rxChannels.channels[LANDING_MODE_CH] > 1800) {
                      flyMode = 1;
                      // use roll and pitch sticks to control speed up to 50 cm/s
                      /*float xspd = ((float)rxChannels.channels[1] - 1500.0f) / 10.0f;
                      float yspd = ((float)rxChannels.channels[0] - 1500.0f) / 10.0f;*/
                      vCtrl.setpointX = controlVelX;
                      vCtrl.setpointY = controlVelY;
                      //vCtrl.setpointX = xspd;
                      //vCtrl.setpointY = yspd;
                      // yaw
                      rxIbus.txCh.channels[3] = rxChannels.channels[3];
                  } else {
                      vCtrl.setpointX = controlVelX;
                      vCtrl.setpointY = controlVelY;
                      rxIbus.txCh.channels[3] = 1500;
                  }

                  velocity_controller_update(&vCtrl, vEst.velX, vEst.velY, accWXB, accWYB, 0.01244444444f);
                  rxIbus.txCh.channels[0] = vCtrl.rollControl;
                  rxIbus.txCh.channels[1] = vCtrl.pitchControl;
              } else { // too low, just hope angle mode keeps us steady
                  rxIbus.txCh.channels[0] = 1500;
                  rxIbus.txCh.channels[1] = 1500;
              }

              // throttle control
              if(controlModePrev == 0 && controlMode == 1) { // height control just engaged
                  height_controller_init(&hCtrl);
              }
              if(controlMode == 0) {
                  rxIbus.txCh.channels[2] = 1000;
              } else {
                  if(rxChannels.channels[LANDING_MODE_CH] < 1800) {
                      hCtrl.setPoint = controlHeight;
                  } else {
                      hCtrl.setPoint = controlHeight;
                      /*float hei = ((float)rxChannels.channels[2] - 1000.0f) / 5.0f;
                      hCtrl.setPoint = hei;
                      */
                  }
                  height_controller_update(&hCtrl, hEst.heightVel, hEst.height, 0.01244444444f);
                  rxIbus.txCh.channels[2] = hCtrl.throttleControl;
              }

              // everything else
              rxIbus.txCh.channels[3] = piChannels.channels[3];
              for(i = 4; i < 14; i++) {
                  if(i != AUTO_MODE_CH && i != ARM_CH) {
                      rxIbus.txCh.channels[i] = piChannels.channels[i];
                  } else {
                      rxIbus.txCh.channels[i] = rxChannels.channels[i];
                  }
              }
          } else { // something horribly wrong
              // TODO: is this fast enough to be done in this interrupt?
              uint16_t throttle = failsafe_autohover_update2(&failsafe, curTick, tfmini.tfmini.dist, telAccZ);
              // TODO: use PID controller to keep height constant
              // currently we just hope the throttle is OK and set angles to 0
              rxIbus.txCh.channels[0] = IBUS_CHANNEL_DEFAULT;   // roll
              rxIbus.txCh.channels[1] = IBUS_CHANNEL_DEFAULT;   // pitch
              rxIbus.txCh.channels[2] = throttle; // throttle
              rxIbus.txCh.channels[3] = IBUS_CHANNEL_DEFAULT;   // yaw
              rxIbus.txCh.channels[4] = rxChannels.channels[4]; // arm
              rxIbus.txCh.channels[6] = 1000 + tfmini.tfmini.dist;
              rxIbus.txCh.channels[7] = 1000 + (uint16_t)failsafe.holdHeightCM;
          }
          failsafePrev = useFailsafe;
      }
      tempModePrev = tempMode;
      manualPrev = useManualMode;
      controlModePrev = controlMode;
      ibus_stm32_try_transmit(&rxIbus);
  }
  // TIM7 was used to get telemetry from FC (yaw, pitch, roll etc)
  // now a seprate IMU is used
  else if(htim->Instance == TIM7) { 
      // NOTE: this should trigger IBUS_CMD_SENSOR_MEASUREMENT in ibus_event callback
      /*uint32_t ctick = HAL_GetTick();
      if((lastTelemetryResponse > lastTelemetryRequest && (ctick - lastTelemetryResponse) > 5)  || (ctick - lastTelemetryRequest > 100)) {
          send_telemetry_sensor_request(&telIbus, 1);
          lastTelemetryRequest = ctick;
      }*/
  }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
