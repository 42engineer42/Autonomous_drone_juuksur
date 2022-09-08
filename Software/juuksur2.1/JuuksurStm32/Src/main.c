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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <ibus.h>
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
uint16_t realHeight;
uint16_t tfminiLast;
uint32_t lastPiData; // last tick we received control data from Pi
uint8_t failsafePrev; // was failsafe enabled last time we checked (for edge detection)
uint8_t manualPrev; // was manual mode enabled last time we checked? (for edge detection)
FailsafeState failsafe; // state for failsafe autohover

uint8_t errorOccured;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  telRoll = 0;
  telPitch = 0;
  telYaw = 0;
  telAccX = 0;
  telAccY = 0;
  telAccZ = 985; // we expect to be on earth (9.85 m/s2)
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
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    /*while (1) {
        // uint8_t wank = 0xAA;
        // HAL_UART_Transmit(&huart3, &wank, 1, 100);
        // HAL_GPIO_TogglePin(BOB2_GPIO_Port, BOB2_Pin);
        HAL_Delay(50);
    }*/
    /*
    // Manually init the pins for uart
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOB->MODER |= (2<<22); // PB11 alternate mode
    GPIOB->AFR[1] |= (7<<11); // PB11 alternate mode 7 should be uart. Probably. Maybe
    USART3->CR1 = 0x00; // Clear
    USART3->CR1 |= (1<<0); // Enable USART
    USART3->BRR =
     */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // enabli pi
  //HAL_GPIO_WritePin(PiEnable_GPIO_Port, PiEnable_Pin, GPIO_PIN_SET);
    // while(1) HAL_UART_Transmit(&huart2, "Wanker\n\r", 8, 100);
    HAL_Delay(5);
  tfmini_stm32_init(&tfmini, &huart5);
  ibus_stm32_init(&piIbus, &huart2, 0);
  ibus_stm32_init(&rxIbus, &huart3, 0);
  ibus_stm32_init(&telIbus, &huart1, 1);

  ibus_set_event_callback(&piIbus.ibRx, ibus_event, &piIbus); 
  ibus_set_event_callback(&rxIbus.ibRx, ibus_event, &rxIbus); 
  ibus_set_event_callback(&telIbus.ibRx, ibus_event, &telIbus);

  // printf("\r\nstarting...\r\n");
  // start uart TX timer
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  // initialize ibus channels to default value
  int i;
  for(i = 0; i < 14; i++) {
      rxChannels.channels[i] = IBUS_CHANNEL_DEFAULT;
  }
  for(i = 0; i < 14; i++) {
      piChannels.channels[i] = IBUS_CHANNEL_DEFAULT;
  }
  uint32_t curTick;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    curTick = HAL_GetTick();

    if((curTick % 2000) < 500){
    	HAL_GPIO_WritePin(RGB_LED_0_R_GPIO_Port, RGB_LED_0_R_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(RGB_LED_1_R_GPIO_Port, RGB_LED_1_R_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(RGB_LED_2_R_GPIO_Port, RGB_LED_2_R_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(RGB_LED_3_R_GPIO_Port, RGB_LED_3_R_Pin, GPIO_PIN_SET);

    	HAL_GPIO_WritePin(RGB_LED_0_G_GPIO_Port, RGB_LED_0_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_1_G_GPIO_Port, RGB_LED_1_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_2_G_GPIO_Port, RGB_LED_2_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_3_G_GPIO_Port, RGB_LED_3_G_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(RGB_LED_0_B_GPIO_Port, RGB_LED_0_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_1_B_GPIO_Port, RGB_LED_1_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_2_B_GPIO_Port, RGB_LED_2_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_3_B_GPIO_Port, RGB_LED_3_B_Pin, GPIO_PIN_RESET);
    }else if((curTick % 2000) < 1000){
    	HAL_GPIO_WritePin(RGB_LED_0_R_GPIO_Port, RGB_LED_0_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_1_R_GPIO_Port, RGB_LED_1_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_2_R_GPIO_Port, RGB_LED_2_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_3_R_GPIO_Port, RGB_LED_3_R_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(RGB_LED_0_G_GPIO_Port, RGB_LED_0_G_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(RGB_LED_1_G_GPIO_Port, RGB_LED_1_G_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(RGB_LED_2_G_GPIO_Port, RGB_LED_2_G_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(RGB_LED_3_G_GPIO_Port, RGB_LED_3_G_Pin, GPIO_PIN_SET);

    	HAL_GPIO_WritePin(RGB_LED_0_B_GPIO_Port, RGB_LED_0_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_1_B_GPIO_Port, RGB_LED_1_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_2_B_GPIO_Port, RGB_LED_2_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_3_B_GPIO_Port, RGB_LED_3_B_Pin, GPIO_PIN_RESET);
    }else if((curTick % 2000) < 1500){
    	HAL_GPIO_WritePin(RGB_LED_0_R_GPIO_Port, RGB_LED_0_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_1_R_GPIO_Port, RGB_LED_1_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_2_R_GPIO_Port, RGB_LED_2_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_3_R_GPIO_Port, RGB_LED_3_R_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(RGB_LED_0_G_GPIO_Port, RGB_LED_0_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_1_G_GPIO_Port, RGB_LED_1_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_2_G_GPIO_Port, RGB_LED_2_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_LED_3_G_GPIO_Port, RGB_LED_3_G_Pin, GPIO_PIN_RESET);

    	HAL_GPIO_WritePin(RGB_LED_0_B_GPIO_Port, RGB_LED_0_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_1_B_GPIO_Port, RGB_LED_1_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_2_B_GPIO_Port, RGB_LED_2_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_3_B_GPIO_Port, RGB_LED_3_B_Pin, GPIO_PIN_SET);
    }else{
    	HAL_GPIO_WritePin(RGB_LED_0_R_GPIO_Port, RGB_LED_0_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_1_R_GPIO_Port, RGB_LED_1_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_2_R_GPIO_Port, RGB_LED_2_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_3_R_GPIO_Port, RGB_LED_3_R_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(RGB_LED_0_G_GPIO_Port, RGB_LED_0_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_1_G_GPIO_Port, RGB_LED_1_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_2_G_GPIO_Port, RGB_LED_2_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_3_G_GPIO_Port, RGB_LED_3_G_Pin, GPIO_PIN_SET);

    	HAL_GPIO_WritePin(RGB_LED_0_B_GPIO_Port, RGB_LED_0_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_1_B_GPIO_Port, RGB_LED_1_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_2_B_GPIO_Port, RGB_LED_2_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_LED_3_B_GPIO_Port, RGB_LED_3_B_Pin, GPIO_PIN_SET);
    }


    if(rxChannels.channels[AUTO_MODE_CH] < 1800) {
        HAL_GPIO_WritePin(TestLED_GPIO_Port, TestLED_Pin, (curTick%100)>50);
    } else {
        if((HAL_GetTick() - lastPiData) > 200) {
            HAL_GPIO_WritePin(TestLED_GPIO_Port, TestLED_Pin, (curTick%500)>250);
        }  else {
            HAL_GPIO_WritePin(TestLED_GPIO_Port, TestLED_Pin, GPIO_PIN_SET);
        }
    }
    ibus_stm32_process_rx(&piIbus);
    ibus_stm32_process_rx(&rxIbus);
    ibus_stm32_process_rx(&telIbus);
    tfmini_stm32_process_rx(&tfmini);
    char payload[200];
    int len = sprintf(payload, "Payload: ");
    HAL_UART_Transmit(&huart2, payload, len, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\rpiIbus: ", 2, 100);
    for (int i = 0; i < 28; ++i) {
        len = sprintf(payload, "%d ", piIbus.ibRx.payload[i]);
        HAL_UART_Transmit(&huart2, payload, len, 100);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r rxIbus: ", 2, 100);
    for (int i = 0; i < 28; ++i) {
        len = sprintf(payload, "%d ", rxIbus.ibRx.payload[i]);
        HAL_UART_Transmit(&huart2, payload, len, 100);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\rtelIbus: ", 2, 100);
    for (int i = 0; i < 28; ++i) {
        len = sprintf(payload, "%d ", telIbus.ibRx.payload[i]);
        HAL_UART_Transmit(&huart2, payload, len, 100);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);

    uint8_t ringBuf[RBUF_SIZE];
    uint8_t prevBuf[RBUF_SIZE];
    uint8_t different = 0;
    HAL_UART_Receive(&huart3, ringBuf, IBUS_PACKET_SIZE, 100);
    uint8_t rBuf[200];
    len = sprintf(rBuf, "RBUF: ");
    HAL_UART_Transmit(&huart2, rBuf, len, 100);
    for (int i = 0; i < RBUF_SIZE; ++i) {
        if (prevBuf[i] != ringBuf[i]) {
            different = 1;
        }
        len = sprintf(rBuf, "%d ", ringBuf[i]);
        HAL_UART_Transmit(&huart2, rBuf, len, 100);
        prevBuf[i] = ringBuf[i];
    }

    if (different == 1) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);
        char rBuf[200];
        len = sprintf(rBuf, "Different: %d", different);
        HAL_UART_Transmit(&huart2, rBuf, len, 100);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);

    // uint8_t* thing = &rxIbus.ibRx.state;
    // PRINT STATE OF RXIBUS
    char value[20];
    int len1 = sprintf(value, "rxIbus state: %X", rxIbus.ibRx.state);
    HAL_UART_Transmit(&huart2, (uint8_t*)value, len1, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);


    // PRINT PAYLOAD OF RXIBUS
    uint8_t* what = &rxIbus.ibRx.payload;
    /*for (int i = 0; i < 28; i++) {
        char yep[20];
        int len = sprintf(yep, "rxIBus payload: %X\n\r", what[i]);

        HAL_UART_Transmit(&huart2, yep, len, 100);
    }*/
    char channel_chars[120];
    int channel_lens = sprintf(channel_chars, "rxIbus ");
    HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
    for(int i=0; i<14; i++) {
        channel_lens = snprintf(channel_chars, 120, "%u ", rxIbus.txCh.channels[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
    }
      HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);

    channel_lens = sprintf(channel_chars, "telIbus ");
    HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
    for(int i=0; i<14; i++) {
        channel_lens = snprintf(channel_chars, 120, "%u ", telIbus.txCh.channels[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
    }

    //channel_lens = sprintf(channel_chars, "\n");
    //HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);

      channel_lens = sprintf(channel_chars, "rxIbus ");
      HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      for(int i=0; i<RBUF_SIZE; i++) {
          channel_lens = snprintf(channel_chars, 120, "%u ", rxIbus.ringBuf[i]);
          HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);


      channel_lens = sprintf(channel_chars, "piIbus ");
      HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      for(int i=0; i<RBUF_SIZE; i++) {
          channel_lens = snprintf(channel_chars, 120, "%u ", piIbus.ringBuf[i]);
          HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);


      channel_lens = sprintf(channel_chars, "telIbus ");
      HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      for(int i=0; i<RBUF_SIZE; i++) {
          channel_lens = snprintf(channel_chars, 120, "%u ", telIbus.ringBuf[i]);
          HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);


      channel_lens = sprintf(channel_chars, "rxChannels ");
      HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      for(int i=0; i<14; i++) {
          channel_lens = snprintf(channel_chars, 120, "%u ", rxChannels.channels[i]);
          HAL_UART_Transmit(&huart2, (uint8_t*)channel_chars, channel_lens, 100);
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);

      // PRINT CUR CMD OF RXIBUS
    uint8_t* yeep = &rxIbus.ibRx.curCmd;
    char no[20];
    int len2 = sprintf(no, "rxIBus curCMD: %X%X\n\r", yeep[0], yeep[1]);
      HAL_UART_Transmit(&huart2, no, len2, 100);


    uint8_t* wank = &telIbus;

    HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
    for (int i = 0; i < sizeof(IbusStm32State); i++) {
        if (wank[i] == 0) continue;
        char cunt[20];
        int len = sprintf(cunt, "%X", wank[i]);
        HAL_UART_Transmit(&huart2, cunt, len, 100);
    }
    HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
    HAL_Delay(1000);

    char fuck[200];
    len = sprintf(fuck, "%X\n\r", telIbus.rxData);
    HAL_UART_Transmit(&huart2, fuck, len, 100);
    HAL_Delay(1000);

    //tsprintf(bigAssThing, "Start %d\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r%d\n\r", piIbus.halfDuplex, piIbus.txBusy,
    //piIbus.rxData, piIbus.rxReadPos, piIbus.rxWritePos
    //)

    if(tfminiLast != tfmini.tfmini.dist) {
        realHeight = (uint16_t)(cosf((((float)telRoll*100.0f)*DEG2RAD)) * cosf(((float)telPitch*100.0f)*DEG2RAD) * tfmini.tfmini.dist);
        tfminiLast = tfmini.tfmini.dist;
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART5;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
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
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}


// NOTE: this should always be called from main not an interrupt
void ibus_event(void *usrData, IbusEvent *e) {
  if(usrData == &piIbus) {
    // printf("pi event!\r\n");
    switch(e->type) {
        case IBUS_CMD_CH:
            piChannels = e->e.ch;
            lastPiData = HAL_GetTick();
            // printf("pi ch: %d %d %d %d\r\n", e->e.ch.channels[0], e->e.ch.channels[1], e->e.ch.channels[2], e->e.ch.channels[3]);
            break;
        case IBUS_CMD_ERROR:
            errorOccured = 1;
            break;
    }
  } else if(usrData == &rxIbus) {
    switch(e->type) {
        case IBUS_CMD_CH:
            rxChannels = e->e.ch;
            // printf("rx channels: %d %d %d %d %d\r\n", e->e.ch.channels[0], e->e.ch.channels[1], e->e.ch.channels[2], e->e.ch.channels[3], e->e.ch.channels[5]);
            break;
    }
  } else if(usrData == &telIbus) {
      int sensor;
      int nextSensor = 1;
      switch(e->type) {
          case IBUS_CMD_SENSOR_MEASUREMENT:
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
