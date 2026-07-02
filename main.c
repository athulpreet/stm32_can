/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : OBD2 CAN reader on STM32F446RE (bxCAN + transceiver)
  *                   - Polls SPEED (PID 0x0D) and RPM (PID 0x0C) @ 500 kbps
  *                   - RX by POLLING (no CAN NVIC interrupt needed)
  *                   - Prints "STATUS: ... SPEED ... RPM ..." on USART2 @115200
  *                   - PWM (TIM1/TIM2) left as-is, to be corrected later
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t vehicle_speed = 0;
volatile uint32_t engine_rpm = 0;

uint8_t standard_mode = 1;
uint32_t functional_id = 0x7DF;

uint32_t prev_tx_time = 0;
const uint32_t tx_interval = 50;
uint32_t last_can_msg_time = 0;
uint32_t last_mode_switch_time = 0;
uint32_t message_fail_count = 0;

bool query_toggle = false;
bool vehicle_connected = false;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint8_t txDataSpeed[8] = {0x02, 0x01, 0x0D, 0x55, 0x55, 0x55, 0x55, 0x55};
uint8_t txDataRPM[8]  = {0x02, 0x01, 0x0C, 0x55, 0x55, 0x55, 0x55, 0x55};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Filter_Config(void);
void Send_OBD_Request(void);
void Update_PWM_Outputs(void);
void Check_Timeout(void);
void Switch_CAN_Mode(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }

  /* NOTE: RX is handled by polling in the loop, so no
   *       HAL_CAN_ActivateNotification / NVIC interrupt is required. */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  last_mode_switch_time = HAL_GetTick();

  /* Startup banner so you can confirm the UART link immediately */
  {
      char boot[] = "\r\nSTM32F446RE OBD2 CAN reader started (500 kbps)\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)boot, strlen(boot), HAL_MAX_DELAY);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_time = 0;
  char uart_buf[100];
  while (1)
  {
      /* ---------- RECEIVE (polling): drain RX FIFO0 ---------- */
      while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
      {
          if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
          {
              last_can_msg_time = HAL_GetTick();
              vehicle_connected = true;
              message_fail_count = 0;

              /* Reply layout: [PCI][0x41][PID][A][B]... */
              if (RxData[1] == 0x41) {
                  if (RxData[2] == 0x0D) {                 /* SPEED (km/h) */
                      vehicle_speed = RxData[3];
                  }
                  else if (RxData[2] == 0x0C) {            /* RPM = ((A*256)+B)/4 */
                      engine_rpm = ((uint32_t)RxData[3] * 256 + RxData[4]) / 4;
                  }
              }
          }
      }

      /* ---------- TRANSMIT: alternate speed / rpm request ---------- */
      if (HAL_GetTick() - prev_tx_time >= tx_interval) {
          prev_tx_time = HAL_GetTick();
          Send_OBD_Request();
      }

      /* ---------- PWM (unchanged, fix later) ---------- */
      Update_PWM_Outputs();

      /* ---------- Connection watchdog ---------- */
      Check_Timeout();

      /* ---------- Print SPEED + RPM every 250 ms ---------- */
      if (HAL_GetTick() - last_print_time >= 250) {
          last_print_time = HAL_GetTick();

          if (vehicle_connected) {
              snprintf(uart_buf, sizeof(uart_buf),
                       "STATUS: Connected | SPEED: %d km/h | RPM: %lu RPM\r\n",
                       (int)vehicle_speed, (unsigned long)engine_rpm);
          } else {
              snprintf(uart_buf, sizeof(uart_buf),
                       "STATUS: Searching for vehicle (Mode: %s)...\r\n",
                       standard_mode ? "Standard 11-bit" : "Extended 29-bit");
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration  (HSI -> PLL -> 84 MHz, APB1 = 42 MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

/**
  * @brief CAN1 Initialization Function  (500 kbps @ APB1 42 MHz)
  */
static void MX_CAN1_Init(void)
{
  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */
  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */
  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Redirect printf to UART2 (optional; STATUS line uses HAL_UART_Transmit) */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void CAN_Filter_Config(void) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    if (standard_mode) {
        /* Accept standard replies 0x7E8 .. 0x7EF */
        sFilterConfig.FilterIdHigh = (0x7E8 << 5);
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = (0x7F8 << 5);
        sFilterConfig.FilterMaskIdLow = 0x0000;
    } else {
        /* Accept extended replies 0x18DAF1xx (29-bit ID sits in bits [31:3],
         * IDE bit = 1). Low byte (responding ECU) is masked out.            */
        uint32_t idE = ((uint32_t)0x18DAF100 << 3) | 0x4U;   /* IDE = 1 */
        uint32_t mkE = ((uint32_t)0x1FFFFF00 << 3) | 0x4U;   /* mask low byte */
        sFilterConfig.FilterIdHigh     = (uint16_t)(idE >> 16);
        sFilterConfig.FilterIdLow      = (uint16_t)(idE & 0xFFFF);
        sFilterConfig.FilterMaskIdHigh = (uint16_t)(mkE >> 16);
        sFilterConfig.FilterMaskIdLow  = (uint16_t)(mkE & 0xFFFF);
    }

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}

void Send_OBD_Request(void) {
    if (standard_mode) {
        TxHeader.StdId = 0x7DF;
        TxHeader.IDE = CAN_ID_STD;
    } else {
        TxHeader.ExtId = 0x18DB33F1;
        TxHeader.IDE = CAN_ID_EXT;
    }

    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (!query_toggle) {
        memcpy(TxData, txDataSpeed, 8);
    } else {
        memcpy(TxData, txDataRPM, 8);
    }
    query_toggle = !query_toggle;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        message_fail_count++;
        if (message_fail_count >= 3) {
            vehicle_connected = false;
            if (HAL_GetTick() - last_mode_switch_time >= 3000) {
                Switch_CAN_Mode();
            }
        }
    }
}

/* PWM section - left as-is per request, to be corrected later.
 * NOTE: htimX Prescaler is 0 in init, but the math below assumes /64 and /1024,
 *       so the output frequency is currently off. Does not affect CAN or UART.  */
void Update_PWM_Outputs(void) {
    if (vehicle_speed > 0) {
        uint32_t speed_hz = vehicle_speed * 2;
        uint32_t timer_clock = 84000000;
        uint32_t prescaler = 64;

        uint32_t arr_val = (timer_clock / (prescaler * speed_hz)) - 1;
        if (arr_val < 65535 && arr_val > 0) {
            if (__HAL_TIM_GET_COUNTER(&htim1) >= arr_val) {
                __HAL_TIM_SET_COUNTER(&htim1, 0);
            }
            __HAL_TIM_SET_AUTORELOAD(&htim1, arr_val);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr_val / 2);
        }
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }

    if (engine_rpm > 200) {
        uint32_t rpm_hz = engine_rpm / 10;
        uint32_t timer_clock = 84000000;
        uint32_t prescaler = 1024;

        uint32_t arr_val = (timer_clock / (prescaler * rpm_hz)) - 1;
        if (arr_val < 0xFFFFFFFF && arr_val > 10) {
            if (__HAL_TIM_GET_COUNTER(&htim2) >= arr_val) {
                __HAL_TIM_SET_COUNTER(&htim2, 0);
            }
            __HAL_TIM_SET_AUTORELOAD(&htim2, arr_val);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr_val / 2);
        }
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void Check_Timeout(void) {
    if (vehicle_connected && (HAL_GetTick() - last_can_msg_time > 1000)) {
        vehicle_connected = false;
        engine_rpm = 0;
        vehicle_speed = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void Switch_CAN_Mode(void) {
    standard_mode = !standard_mode;
    message_fail_count = 0;

    HAL_CAN_Stop(&hcan1);
    CAN_Filter_Config();
    HAL_CAN_Start(&hcan1);
    /* No ActivateNotification: RX is polled in the main loop */

    last_mode_switch_time = HAL_GetTick();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
