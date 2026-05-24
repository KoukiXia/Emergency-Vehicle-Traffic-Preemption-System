/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Ambulance node — FreeRTOS edition.
  *                   4 tasks: button (50 Hz), radio (every 2s), siren (2 Hz),
  *                   watchdog (1 Hz, via defaultTask).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "packet.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

#define AMBULANCE_ID        0x01
#define TX_INTERVAL_MS      2000U
#define BUTTON_DEBOUNCE_MS   150U

static const uint8_t HMAC_KEY[]   = "DEMO_KEY_DO_NOT_USE_IN_PRODUCTION";
static const size_t  HMAC_KEY_LEN = sizeof(HMAC_KEY) - 1;

/* Shared state (volatile because read/written by multiple tasks and the ISR) */
static volatile bool      toggle_pending     = false;
static volatile uint32_t  button_last_irq_ms = 0;
static volatile bool      emergency_active   = false;
static volatile uint32_t  sequence_number    = 0;
static volatile bool      lora_ok            = false;

/* Additional task handles */
osThreadId_t buttonTaskHandle;
osThreadId_t radioTaskHandle;
osThreadId_t sirenTaskHandle;

const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t sirenTask_attributes = {
  .name = "sirenTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void StartButtonTask(void *argument);
static void StartRadioTask(void *argument);
static void StartSirenTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == BTN_ACTIVATE_Pin) {
        if (now - button_last_irq_ms > BUTTON_DEBOUNCE_MS) {
            toggle_pending = true;
            button_last_irq_ms = now;
        }
    }
    else if (GPIO_Pin == LORA_DIO0_Pin) {
        lora_handle_dio0_irq();
    }
}

static void uart_send(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_log(const char *fmt, ...)
{
    char buf[120];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
}

static void set_led(bool on)
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void send_emergency_packet(void)
{
    emergency_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic        = EMERGENCY_PACKET_MAGIC;
    pkt.ambulance_id = AMBULANCE_ID;
    pkt.direction    = (uint8_t)DIR_EAST;
    pkt.flags        = 0x01;
    pkt.sequence     = ++sequence_number;
    pkt.timestamp_ms = HAL_GetTick();

    packet_sign(&pkt, HMAC_KEY, HMAC_KEY_LEN);

    if (!lora_ok) {
        uart_log("[%lu] would TX seq=%lu (LoRa unavailable)\r\n",
                 (unsigned long)HAL_GetTick(), (unsigned long)pkt.sequence);
        return;
    }

    if (lora_send((uint8_t *)&pkt, sizeof(pkt)) == LORA_OK) {
        uart_log("[%lu] TX seq=%lu\r\n",
                 (unsigned long)HAL_GetTick(), (unsigned long)pkt.sequence);
    } else {
        uart_log("[%lu] TX failed seq=%lu\r\n",
                 (unsigned long)HAL_GetTick(), (unsigned long)pkt.sequence);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
    uart_send("\r\n\r\n========================================\r\n");
    uart_send("Ambulance unit - FreeRTOS edition\r\n");
    uart_send("========================================\r\n");

    /* Diagnostic: print before lora_init in case it hangs */
    uart_send("Calling lora_init...\r\n");

    lora_config_t cfg = {
        .hspi      = &hspi1,
        .nss_port  = LORA_NSS_GPIO_Port,
        .nss_pin   = LORA_NSS_Pin,
        .rst_port  = LORA_RST_GPIO_Port,
        .rst_pin   = LORA_RST_Pin,
    };

    if (lora_init(&cfg) == LORA_OK) {
        lora_set_frequency(433000000);
        lora_set_spreading_factor(7);
        lora_set_bandwidth_125khz();
        lora_set_coding_rate_4_5();
        lora_set_tx_power(2);
        lora_set_sync_word(0xF3);
        lora_ok = true;
        uart_send("LoRa initialized OK\r\n");
    } else {
        uart_send("ERROR: LoRa init failed - check wiring (continuing without TX)\r\n");
    }

    set_led(false);
    uart_send("Starting FreeRTOS scheduler...\r\n\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  /* USER CODE BEGIN RTOS_THREADS */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);
  radioTaskHandle  = osThreadNew(StartRadioTask,  NULL, &radioTask_attributes);
  sirenTaskHandle  = osThreadNew(StartSirenTask,  NULL, &sirenTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_NSS_Pin|LORA_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_S_Pin DIR_W_Pin DIR_E_Pin DIR_N_Pin */
  GPIO_InitStruct.Pin = DIR_S_Pin|DIR_W_Pin|DIR_E_Pin|DIR_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_NSS_Pin LORA_RST_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin|LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_ACTIVATE_Pin */
  GPIO_InitStruct.Pin = BTN_ACTIVATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_ACTIVATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ============================================================
 * Task 1: Button watcher — 50 Hz polling for toggle events.
 * The EXTI ISR sets toggle_pending; this task picks it up,
 * flips emergency_active, and prints the state change.
 * ============================================================ */
static void StartButtonTask(void *argument)
{
    (void)argument;
    for (;;) {
        if (toggle_pending) {
            toggle_pending = false;
            emergency_active = !emergency_active;
            if (emergency_active) {
                uart_log("[%lu] ===== EMERGENCY MODE ON =====\r\n",
                         (unsigned long)HAL_GetTick());
            } else {
                uart_log("[%lu] ===== EMERGENCY MODE OFF =====\r\n",
                         (unsigned long)HAL_GetTick());
                set_led(false);  /* immediately turn off siren */
            }
        }
        osDelay(20);  /* 50 Hz polling */
    }
}

/* ============================================================
 * Task 2: Radio TX — every 2 seconds, sends a packet
 * if emergency mode is active.
 * ============================================================ */
static void StartRadioTask(void *argument)
{
    (void)argument;
    for (;;) {
        if (emergency_active) {
            send_emergency_packet();
        }
        osDelay(TX_INTERVAL_MS);
    }
}

/* ============================================================
 * Task 3: Siren — 2 Hz blink of LD3 while in emergency mode.
 * ============================================================ */
static void StartSirenTask(void *argument)
{
    (void)argument;
    bool led_state = false;
    for (;;) {
        if (emergency_active) {
            led_state = !led_state;
            set_led(led_state);
        } else {
            led_state = false;
            /* Note: buttonTask already turns the LED off on toggle-off */
        }
        osDelay(500);  /* 2 Hz blink rate */
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  *         Repurposed as a watchdog/heartbeat task: prints status to UART
  *         every 10 seconds so we know the system is alive.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;
  uint32_t beat = 0;
  /* Infinite loop */
  for(;;)
  {
    beat++;
    if ((beat % 10) == 0) {
        uart_log("[%lu] alive - emergency=%s seq=%lu\r\n",
                 (unsigned long)HAL_GetTick(),
                 emergency_active ? "ON" : "off",
                 (unsigned long)sequence_number);
    }
    osDelay(1000);  /* 1 Hz */
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
#ifdef USE_FULL_ASSERT
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