/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <assert.h>
#include <string.h>
#include "vl53l1_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEG2RAD 0.01745329251f
#define TIM2_MS_PER_TICK 0.01388888888f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
IbusChannels rxChannels; // latest channel state from receiver
IbusChannels piChannels; // latest channel state from pi (autonomous)
IbusStm32State piIbus; // both rx and tx to pi
IbusStm32State rxIbus; // rx comes from receiver, tx goes to FC
IbusStm32State telIbus; // half-duplex UART to FC for telemetry
TfminiStm32State tfmini;
int16_t telYaw, telPitch, telRoll;
int16_t telAccX, telAccY, telAccZ;
void ibus_event(void *usrData, IbusEvent *e);
float realHeightF;
uint16_t realHeight;
uint16_t tfminiLast;
uint32_t lastPiData; // last tick we received control data from Pi
uint8_t failsafePrev; // was failsafe enabled last time we checked (for edge detection)
uint8_t manualPrev; // was manual mode enabled last time we checked? (for edge detection)
uint32_t debugBufferCount;
uint8_t debugBuffer[256];
FailsafeState failsafe; // state for failsafe autohover

float opticalFlowX;
float opticalFlowY;
float avgOFX[10];
float avgOFY[10];
uint32_t ofIdx;
double lastOpticalFlowUpdate;

float controlVelX;
float controlVelY;

// height set point
float controlHeight;
// current auto control mode
uint16_t controlMode;

double curTime;

// acceleration considering roll and pitch (cm/s*s)
// NOTE: both have their XY axis parallel to ground plane (they are world space not local)
float accWX;
float accWY;
float accWZ;
// acceleration with bias adjustments
float accWXB;
float accWYB;
float accWZB;

VL53L1_RangingMeasurementData_t vl53Data;

Mpu6050Data mpu;
MpuFusion mpuFusion;
HeightEstimator hEst;
VelocityEstimator vEst;
VelocityController vCtrl;
AngleController aCtrl;
HeightController hCtrl;
LEDController leds;

uint8_t flyMode;

uint32_t lastTelemetryRequest;
uint32_t lastTelemetryResponse;

uint8_t errorOccured;

uint16_t startL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file,char *ptr, int len);

#pragma pack(push, 1)
typedef struct SensorData {
    float acc[3];
    float gyro[3];
    float pitch, roll;
    char hello[6];
} SensorData;
#pragma pack(pop)

// this was used for uart graphing
uint8_t header[3] = {'H', 'H', 'H'};

uint32_t lastVl53Data;
VL53L1_Dev_t vl53;
VL53L1_DEV vl53Dev = &vl53;

SensorData sdata;

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
  lastPiData = 0;
  failsafePrev = 0;
  manualPrev = 0;
  realHeight = 30;
  tfminiLast = 0;
  lastOpticalFlowUpdate = 0.0;
  telRoll = 0;
  telPitch = 0;
  telYaw = 0;
  telAccX = 0;
  telAccY = 0;
  telAccZ = 985; // we expect to be on earth (9.85 m/s2)

  accWX = 0.0f;
  accWY = 0.0f;
  accWZ = 985.0f;

  accWXB = 0.0f;
  accWYB = 0.0f;
  accWZB = 985.0f;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(50);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // enabli pi
  //HAL_GPIO_WritePin(PiEnable_GPIO_Port, PiEnable_Pin, GPIO_PIN_SET);

  /*HAL_GPIO_WritePin(TestLED2_Green_GPIO_Port, TestLED2_Green_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TestLED3_Green_GPIO_Port, TestLED3_Green_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TestLED1_Green_GPIO_Port, TestLED1_Green_Pin, GPIO_PIN_SET);*/

  HAL_Delay(10);

  vl53Dev->I2cHandle = &hi2c2;
  vl53Dev->I2cDevAddr = 0x52;

  int vlstatus = VL53L1_WaitDeviceBooted(vl53Dev);
  vlstatus = VL53L1_DataInit(vl53Dev);
  vlstatus = VL53L1_StaticInit(vl53Dev);
  vlstatus = VL53L1_SetDistanceMode(vl53Dev, VL53L1_DISTANCEMODE_SHORT);
  vlstatus = VL53L1_SetMeasurementTimingBudgetMicroSeconds(vl53Dev, 15000);
  vlstatus = VL53L1_SetInterMeasurementPeriodMilliSeconds(vl53Dev, 20);
  vlstatus = VL53L1_StartMeasurement(vl53Dev);
  if(vlstatus) {
      printf("VL53L1_StartMeasurement failed \n");
      while(1);
  }


  mpu.lastData = 0;
  mpu.lastProcess = 0;
  mpu.rawData[0] = 0xFF;

  if(mpu_init(&hi2c1, MPU6050_I2C_ADDR) != HAL_OK) {
      HAL_GPIO_WritePin(TestLED4_Red_GPIO_Port, TestLED4_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(TestLED3_Red_GPIO_Port, TestLED3_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(TestLED2_Red_GPIO_Port, TestLED2_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(TestLED1_Red_GPIO_Port, TestLED1_Red_Pin, GPIO_PIN_SET);
      printf("Failed to find IMU on I2C bus\r\n");
      assert(0); // failed to find mpu6050
  }
  mpu_fusion_init(&mpuFusion);

  height_estimator_init(&hEst);
  velocity_estimator_init(&vEst);

  tfmini_stm32_init(&tfmini, &huart3);
  ibus_stm32_init(&piIbus, &huart2, &huart2, 0);
  ibus_stm32_init(&rxIbus, &huart3, &huart1, 0);
  //ibus_stm32_init(&telIbus, &huart1, &huart1, 1);

  ibus_set_event_callback(&piIbus.ibRx, ibus_event, &piIbus); 
  ibus_set_event_callback(&rxIbus.ibRx, ibus_event, &rxIbus); 
  //ibus_set_event_callback(&telIbus.ibRx, ibus_event, &telIbus);

  led_controller_init(&leds);

  printf("\r\nstarting...\r\n");
  // start uart TX timer
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  // start timer for time measurement
  HAL_TIM_Base_Start(&htim2);
  // start timer for i2c IMU measurement
  HAL_TIM_Base_Start_IT(&htim3);


  startL = TIM2->CNT;

  // initialize ibus channels to default value
  int i;
  for(i = 0; i < 14; i++) {
      rxChannels.channels[i] = IBUS_CHANNEL_DEFAULT;
  }
  for(i = 0; i < 14; i++) {
      piChannels.channels[i] = IBUS_CHANNEL_DEFAULT;
  }
  float lastHeightEstimation = 0.0f;
  float lastVelocityEstimation = 0.0f;
  float lastPrint = 0.0f;

  ibus_stm32_start_receive(&piIbus);
  ibus_stm32_start_receive(&rxIbus);
  //ibus_stm32_start_receive(&telIbus);
  tfmini_stm32_start_receive(&tfmini);

  while (1)
  {
    uint16_t endL = TIM2->CNT;
    uint16_t dif;
    if(endL > startL) {
        dif = endL - startL;
    } else {
        dif = endL + (36000-startL);
    }
    float dt = (dif*TIM2_MS_PER_TICK)/1000.0f;
    startL = endL;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    led_controller_update(&leds, HAL_GetTick());

    if(rxChannels.channels[AUTO_MODE_CH] < 1800) { // auto mode off
        leds.mask[0] = 0x0f;
        leds.mask[1] = 0x00;
        leds.mask[2] = 0x00;
        for(int i = 0; i < 4; i++) {
            leds.toggleInterval[i] = 50;
        }
    } else { // auto mode on
        if((HAL_GetTick() - lastPiData) > 200) { // no pi data
            leds.mask[0] = 0x0f;
            leds.mask[1] = 0x0f;
            leds.mask[2] = 0x00;
            for(int i = 0; i < 4; i++) {
                leds.toggleInterval[i] = 250;
            }
        }  else { // all good
            leds.mask[0] = 0x00;
            leds.mask[1] = 0x0f;
            if(flyMode == 0) {
                leds.mask[2] = 0x00;
            } else {
                leds.mask[2] = 0x0f;
            }
            for(int i = 0; i < 4; i++) {
                leds.toggleInterval[i] = 0;
            }
        }
    }

    ibus_stm32_process_rx(&piIbus);
    ibus_stm32_process_rx(&rxIbus);
    //ibus_stm32_process_rx(&telIbus);
    tfmini_stm32_process_rx(&tfmini);
    if(tfminiLast != tfmini.tfmini.dist) {
        realHeightF = (cosf((((float)telRoll/100.0f)*DEG2RAD)) * cosf(((float)telPitch/100.0f)*DEG2RAD) * tfmini.tfmini.dist);
        realHeight = (uint16_t)realHeightF;
        tfminiLast = tfmini.tfmini.dist;
    }
    mpu_process_data(mpu.rawData, &mpu);
    mpu_update_angle_estimates(&mpu, &mpuFusion, dt);
    if(debugBufferCount > 0) {
        __disable_irq();
        HAL_UART_Transmit_IT(&huart5, debugBuffer, debugBufferCount);
        __enable_irq();
        debugBufferCount = 0;
    }
    
    float alpha = mpuFusion.roll*DEG2RAD;
    float beta = mpuFusion.pitch*DEG2RAD;
    float accX = mpu.acc[0] - vEst.accBiasX;
    float accY = mpu.acc[1] - vEst.accBiasY;
    float accZ = mpu.acc[2] - hEst.accBias;
    accWX = (mpu.acc[0]*cosf(beta) + mpu.acc[2]*cosf(alpha)*sinf(beta) + mpu.acc[1]*sinf(alpha)*sinf(beta)) * 100.0f;
    accWY = (mpu.acc[1]*cosf(alpha) - mpu.acc[2]*sinf(alpha)) * 100.0f;
    accWZ = (mpu.acc[2]*cosf(alpha)*cosf(beta) - mpu.acc[0]*sinf(beta) + mpu.acc[1]*cosf(beta)*sinf(alpha)) * 100.0f;

    accWXB = (accX*cosf(beta) + accZ*cosf(alpha)*sinf(beta) + accY*sinf(alpha)*sinf(beta)) * 100.0f;
    accWYB = (accY*cosf(alpha) - accZ*sinf(alpha)) * 100.0f;
    accWZB = (accZ*cosf(alpha)*cosf(beta) - accX*sinf(beta) + accY*cosf(beta)*sinf(alpha)) * 100.0f;
    if(lastHeightEstimation+dt >= 0.01f) {
        float hdt = dt+lastHeightEstimation;
        //float accx = mpu.acc[0]*cosf(beta) + mpu.acc[2]*cosf(alpha)*sinf(beta) + mpu.acc[1]*sinf(alpha)*sinf(beta);
        //float accy = mpu.acc[1]*cosf(alpha) - mpu.acc[2]*sinf(alpha);
        if(accWZ != 985.0f) {
            height_estimator_predict(&hEst, accWZ, hdt);
            if(tfminiLast != 0 && tfmini.tfmini.dist != 30) {
                height_estimator_update(&hEst, realHeight, hdt);
            } else if((HAL_GetTick() - lastVl53Data) < 100) {
                height_estimator_update(&hEst, (float)vl53Data.RangeMilliMeter/10.0f, hdt);
            } else /*if(HAL_GetTick() < 5000)*/ {
                height_estimator_update(&hEst, 30.0f, hdt);
            }
        }
        lastHeightEstimation = 0.0f;
    } else {
        lastHeightEstimation += dt;
    }

    if(lastVelocityEstimation+dt >= 0.01f) {
        float vdt = dt+lastVelocityEstimation;
        velocity_estimator_predict(&vEst, accWX, accWY, vdt);
        lastVelocityEstimation = 0.0f;

        avgOFX[ofIdx%10] = vEst.velX;
        avgOFY[ofIdx%10] = vEst.velY;
        ofIdx++;
    } else {
        lastVelocityEstimation += dt;
    }

    if(lastPrint+dt >= 0.1f) {
#if 1
        //float hacc = mpu.acc[2]*100.0f;
        //printf("accZ %f height %f hvel %f accbias %f\r\n", hacc, hEst.height, hEst.heightVel, hEst.accBias);
        //printf("realHeight %f (%d r %f p %f)\r\n", realHeightF, tfmini.tfmini.dist, (float)telRoll/100.0f, (float)telPitch/100.0f);
        
        //printf("velx estimation %f %f bias %f \r\n", vEst.velX, vEst.velY, vEst.accBiasX);
        //printf("vely estimation %f %f bias %f \r\n", vEst.velY, vEst.velX, vEst.accBiasX);
        //printf("kk %f %f\r\n", vEst.kkx[0], vEst.kkx[1]);
        lastPrint = 0.0f;
#else
        _write(0, (char*)header, 3);
        sdata.acc[0] = mpu.acc[0]; 
        sdata.acc[1] = mpu.acc[1]; 
        sdata.acc[2] = mpu.acc[2]; 
        sdata.gyro[0] = mpu.gyro[0]; 
        sdata.gyro[1] = mpu.gyro[1]; 
        sdata.gyro[2] = (float)tfmini.tfmini.dist; 
        sdata.pitch = hEst.height;
        sdata.roll = hEst.heightVel;
        strcpy(sdata.hello, "hello");
        _write(0, (char*)&sdata, sizeof(sdata));
#endif
    } else {
        lastPrint += dt;
    }

    curTime += dt;
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch)
{
    //110
  //uint8_t c[1];
  //c[0] = ch & 0xFF;
  //HAL_UART_Transmit(&huart5, &*c, 1, 10);
  return ch;
}

// this is used for libc stdio (printf etc)
int _write(int file,char *ptr, int len)
{
  int left = sizeof(debugBuffer) - debugBufferCount;
  if(len < left) {
      left = len;
  }
  memcpy(debugBuffer + debugBufferCount, ptr, left);
  debugBufferCount += left;
  return len;
}


// NOTE: this should always be called from main not an interrupt
void ibus_event(void *usrData, IbusEvent *e) {
  if(usrData == &piIbus) {
    //printf("pi event!\r\n");
    switch(e->type) {
        case IBUS_CMD_CH:
            piChannels = e->e.ch;
            // 0xFFFF means the value has not been updated
            if(piChannels.channels[8] != 0xFFFF) {
                opticalFlowX = (float)(*((int16_t*)(piChannels.channels+8)))/10.0f;
                opticalFlowY = (float)(*((int16_t*)(piChannels.channels+9)))/10.0f;

                velocity_estimator_update(&vEst, opticalFlowY, opticalFlowX, curTime-lastOpticalFlowUpdate);

                lastOpticalFlowUpdate = curTime;
            }

            // target velocity from Pi
            controlVelX = (float)(*((int16_t*)(piChannels.channels+10)))/10.0f;
            controlVelY = (float)(*((int16_t*)(piChannels.channels+11)))/10.0f;

            controlHeight = (float)(*((int16_t*)(piChannels.channels+12)))/10.0f;
            controlMode = piChannels.channels[13];

            lastPiData = HAL_GetTick();
            //printf("pi ch: %d %d %d %d\r\n", e->e.ch.channels[0], e->e.ch.channels[1], e->e.ch.channels[2], e->e.ch.channels[3]);
            break;
        case IBUS_CMD_ERROR:
            errorOccured = 1;
            break;
    }
  } else if(usrData == &rxIbus) {
    switch(e->type) {
        case IBUS_CMD_CH:
            rxChannels = e->e.ch;
            //printf("rx channels: %d %d %d %d %d\r\n", e->e.ch.channels[0], e->e.ch.channels[1], e->e.ch.channels[2], e->e.ch.channels[3], e->e.ch.channels[5]);
            break;
    }
  } else if(usrData == &telIbus) {
      int sensor;
      int nextSensor = 1;
      switch(e->type) {
          case IBUS_CMD_SENSOR_MEASUREMENT:
              lastTelemetryResponse = HAL_GetTick();
              sensor = e->e.smeas.sensorId;
              nextSensor = sensor+1;
              if(nextSensor > 1 && nextSensor <= 6) { // 1 - roll, 2- pitch, 3 - yaw, 4 - accX, 5 - accY, 6 - accZ
                  send_telemetry_sensor_request(&telIbus, nextSensor);
              }
              // NOTE: ideally we would do discovery, but aint nobody got time for that

              switch(sensor) {
                  case 1: // roll
                    telRoll = e->e.smeas.meas; // angle*100
                    break;
                  case 2: // pitch
                    telPitch = e->e.smeas.meas;
                    break;
                  case 3: // yaw
                    telYaw = e->e.smeas.meas;
                    break;
                  case 4:
                    telAccX = e->e.smeas.meas;
                    break;
                  case 5:
                    telAccY = e->e.smeas.meas;
                    break;
                  case 6:
                    telAccZ = e->e.smeas.meas;
                    break;
              }
              break;
      }
  }
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
    //HAL_GPIO_WritePin(TestLED2_Green_GPIO_Port, TestLED2_Green_Pin, GPIO_PIN_SET);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    //HAL_GPIO_WritePin(TestLED3_Green_GPIO_Port, TestLED3_Green_Pin, GPIO_PIN_SET);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
