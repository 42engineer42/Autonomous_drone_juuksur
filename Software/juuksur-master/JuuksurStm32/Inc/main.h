/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ibus_stm32.h"
#include "tfmini_stm32.h"
#include "failsafe.h"
#include "mpu6050.h"
#include "HeightEstimator.h"
#include "VelocityEstimator.h"
#include "velocity_controller.h"
#include "angle_mode.h"
#include "height_controller.h"
#include "led_controller.h"
#include "vl53l1_def.h"
#include "vl53l1_api.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern IbusChannels rxChannels; // latest channel state from receiver
extern IbusChannels piChannels; // latest channel state from pi (autonomous)
extern IbusStm32State piIbus; // both rx and tx to pi
extern IbusStm32State rxIbus; // rx comes from receiver, tx goes to FC
extern IbusStm32State telIbus; // half-duplex UART to FC for telemetry
extern TfminiStm32State tfmini;
extern int16_t telYaw, telPitch, telRoll;
extern int16_t telAccX, telAccY, telAccZ;
extern uint32_t lastPiData; // last tick we received control data from Pi
extern uint32_t lastTelemetryRequest;
extern uint32_t lastTelemetryResponse;
extern FailsafeState failsafe;
extern uint8_t failsafePrev;
extern uint8_t manualPrev;
extern uint16_t realHeight;

// time in seconds since startup when optical flow update was performed
extern double lastOpticalFlowUpdate;
extern float opticalFlowX;
extern float opticalFlowY;

extern float avgOFX[10];
extern float avgOFY[10];

// velocity set points
extern float controlVelX;
extern float controlVelY;

// height set point
extern float controlHeight;

// current auto control mode
extern uint16_t controlMode;

// time in seconds since startup
extern double curTime;

extern VL53L1_RangingMeasurementData_t vl53Data;
extern VL53L1_DEV vl53Dev;
extern uint32_t lastVl53Data;

extern Mpu6050Data mpu;
extern MpuFusion mpuFusion;
extern HeightEstimator hEst;
extern VelocityEstimator vEst;
extern VelocityController vCtrl;
extern HeightController hCtrl;
extern AngleController aCtrl;
extern LEDController leds;

extern float accWX;
extern float accWY;
extern float accWZ;

extern float accWXB;
extern float accWYB;
extern float accWZB;

extern uint8_t flyMode;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ARM_CH      4
#define AUTO_MODE_CH 5
#define CALIB_MODE_CH 6
#define LANDING_MODE_CH 7
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TestLED2_Red_Pin GPIO_PIN_1
#define TestLED2_Red_GPIO_Port GPIOC
#define TestLED2_Green_Pin GPIO_PIN_2
#define TestLED2_Green_GPIO_Port GPIOC
#define TestLED2_Blue_Pin GPIO_PIN_3
#define TestLED2_Blue_GPIO_Port GPIOC
#define TestLED3_Red_Pin GPIO_PIN_0
#define TestLED3_Red_GPIO_Port GPIOB
#define TestLED3_Green_Pin GPIO_PIN_1
#define TestLED3_Green_GPIO_Port GPIOB
#define TestLED3_Blue_Pin GPIO_PIN_2
#define TestLED3_Blue_GPIO_Port GPIOB
#define TestLED4_Red_Pin GPIO_PIN_12
#define TestLED4_Red_GPIO_Port GPIOB
#define TestLED4_Green_Pin GPIO_PIN_13
#define TestLED4_Green_GPIO_Port GPIOB
#define TestLED4_Blue_Pin GPIO_PIN_14
#define TestLED4_Blue_GPIO_Port GPIOB
#define TestLED1_Red_Pin GPIO_PIN_7
#define TestLED1_Red_GPIO_Port GPIOC
#define TestLED1_Green_Pin GPIO_PIN_8
#define TestLED1_Green_GPIO_Port GPIOC
#define TestLED1_Blue_Pin GPIO_PIN_9
#define TestLED1_Blue_GPIO_Port GPIOC
#define TOF_Int_Pin GPIO_PIN_11
#define TOF_Int_GPIO_Port GPIOA
#define TOF_Int_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
