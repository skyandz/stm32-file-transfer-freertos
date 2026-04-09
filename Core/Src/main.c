/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm_crc32.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    DWORD readPoint;
    uint32_t file_crc;

    uint32_t write_crc;
    char last_filename[32];

    uint16_t file_index;
} sd_path_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP30_CAL_ADDR   (*((uint16_t*)0x1FFF75A8))
#define TEMP130_CAL_ADDR  (*((uint16_t*)0x1FFF75CA))

#define OBC_LOG_PERIOD_MS   30000
#define LINES_PER_FILE  3
//#define OBC_MAX_FILES       3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart_rx;
DMA_HandleTypeDef hdma_lpuart_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId UartRxTaskHandle;
osThreadId UartTxTaskHandle;
osThreadId DispTaskHandle;
osThreadId SdTaskHandle;
osThreadId FlashTaskHandle;
osThreadId CircularTaskHandle;
osThreadId FileIntegrityChHandle;
osThreadId ObcLogFileHandle;
osThreadId SpiRxTaskHandle;
osThreadId PayloadLogFileHandle;
osMessageQId UartRxQueueHandle;
osMessageQId UartTxQueueHandle;
osMessageQId DispQueueHandle;
osMessageQId SdQueueHandle;
osMessageQId FlashQueueHandle;
osMessageQId PayloadQueueHandle;
/* USER CODE BEGIN PV */
osPoolId req_pool;
osPoolId resp_pool;
osPoolId payload_pool;

uint8_t dma_state = 0;

static uint32_t obc_write_crc = 0xFFFFFFFF;
static char obc_last_filename[32] = {0};

static uint32_t payload_write_crc = 0xFFFFFFFF;
static char payload_last_filename[32] = {0};
// Configuration variables
uint64_t CIRCULAR_SIZE = 0;
uint64_t CIRCULAR_INTERVAL = 0;
uint64_t INTEGRITY_CHECK_INTERVAL = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void const * argument);
void uart_rx_task(void const * argument);
void uart_tx_task(void const * argument);
void disp_task(void const * argument);
void sd_task(void const * argument);
void flash_task(void const * argument);
void circular_task(void const * argument);
void file_integrity_check(void const * argument);
void obc_log_file(void const * argument);
void spi_rx_task(void const * argument);
void payload_log_file(void const * argument);

/* USER CODE BEGIN PFP */
// Flash Configuration Functions
HAL_StatusTypeDef Flash_Write_Config(uint64_t circular_size, uint64_t circular_interval, uint64_t integrity_interval);
void Flash_Read_Config(uint64_t *circular_size, uint64_t *circular_interval, uint64_t *integrity_interval);
uint8_t Flash_Is_Config_Valid(void);
HAL_StatusTypeDef handle_delete_file(msg_req_t *req, msg_resp_t *res, const char *base_path);
static const char *get_base_path_from_cmd(uint8_t cmd_id);
static sd_path_state_t *get_sd_state_from_cmd(uint8_t cmd_id);
static uint8_t is_sd_cmd(uint8_t cmd_id);
static void integrity_check_path(const char *base_path,
                                 uint32_t *line_index);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  // Load configuration from Flash on startup
  if (Flash_Is_Config_Valid())
  {
      Flash_Read_Config(&CIRCULAR_SIZE, &CIRCULAR_INTERVAL, &INTEGRITY_CHECK_INTERVAL);

      // Indicate successful load with LED
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  }


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of UartRxQueue */
  osMessageQDef(UartRxQueue, 5, sizeof(msg_req_t *));
  UartRxQueueHandle = osMessageCreate(osMessageQ(UartRxQueue), NULL);

  /* definition and creation of UartTxQueue */
  osMessageQDef(UartTxQueue, 5, sizeof(msg_resp_t *));
  UartTxQueueHandle = osMessageCreate(osMessageQ(UartTxQueue), NULL);

  /* definition and creation of DispQueue */
  osMessageQDef(DispQueue, 5, sizeof(msg_req_t *));
  DispQueueHandle = osMessageCreate(osMessageQ(DispQueue), NULL);

  /* definition and creation of SdQueue */
  osMessageQDef(SdQueue, 5, sizeof(msg_req_t *));
  SdQueueHandle = osMessageCreate(osMessageQ(SdQueue), NULL);

  /* definition and creation of FlashQueue */
  osMessageQDef(FlashQueue, 5, sizeof(msg_req_t *));
  FlashQueueHandle = osMessageCreate(osMessageQ(FlashQueue), NULL);

  /* definition and creation of PayloadQueue */
  osMessageQDef(PayloadQueue, 5, sizeof(payload_msg_t *));
  PayloadQueueHandle = osMessageCreate(osMessageQ(PayloadQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UartRxTask */
  osThreadDef(UartRxTask, uart_rx_task, osPriorityNormal, 0, 512);
  UartRxTaskHandle = osThreadCreate(osThread(UartRxTask), NULL);

  /* definition and creation of UartTxTask */
  osThreadDef(UartTxTask, uart_tx_task, osPriorityNormal, 0, 512);
  UartTxTaskHandle = osThreadCreate(osThread(UartTxTask), NULL);

  /* definition and creation of DispTask */
  osThreadDef(DispTask, disp_task, osPriorityNormal, 0, 512);
  DispTaskHandle = osThreadCreate(osThread(DispTask), NULL);

  /* definition and creation of SdTask */
  osThreadDef(SdTask, sd_task, osPriorityNormal, 0, 1024);
  SdTaskHandle = osThreadCreate(osThread(SdTask), NULL);

  /* definition and creation of FlashTask */
  osThreadDef(FlashTask, flash_task, osPriorityNormal, 0, 512);
  FlashTaskHandle = osThreadCreate(osThread(FlashTask), NULL);

  /* definition and creation of CircularTask */
  osThreadDef(CircularTask, circular_task, osPriorityNormal, 0, 1024);
  CircularTaskHandle = osThreadCreate(osThread(CircularTask), NULL);

  /* definition and creation of FileIntegrityCh */
  osThreadDef(FileIntegrityCh, file_integrity_check, osPriorityNormal, 0, 1024);
  FileIntegrityChHandle = osThreadCreate(osThread(FileIntegrityCh), NULL);

  /* definition and creation of ObcLogFile */
  osThreadDef(ObcLogFile, obc_log_file, osPriorityNormal, 0, 1024);
  ObcLogFileHandle = osThreadCreate(osThread(ObcLogFile), NULL);

  /* definition and creation of SpiRxTask */
  osThreadDef(SpiRxTask, spi_rx_task, osPriorityNormal, 0, 512);
  SpiRxTaskHandle = osThreadCreate(osThread(SpiRxTask), NULL);

  /* definition and creation of PayloadLogFile */
  osThreadDef(PayloadLogFile, payload_log_file, osPriorityNormal, 0, 1024);
  PayloadLogFileHandle = osThreadCreate(osThread(PayloadLogFile), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osPoolDef(REQ_POOL, 5, msg_req_t);
  req_pool = osPoolCreate(osPool(REQ_POOL));
  osPoolDef(RESP_POOL, 5, msg_resp_t);
  resp_pool = osPoolCreate(osPool(RESP_POOL));
  osPoolDef(PAYLOAD_POOL, 5, payload_msg_t);
  payload_pool = osPoolCreate(osPool(PAYLOAD_POOL));
  /* USER CODE END RTOS_THREADS */

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x11;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_MARCH;
  sDate.Date = 0x30;
  sDate.Year = 0x26;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


static sd_path_state_t sd_test_state    = {0, 0xFFFFFFFF, 0xFFFFFFFF, {0}, 0};
static sd_path_state_t sd_obc_state     = {0, 0xFFFFFFFF, 0xFFFFFFFF, {0}, 0};
static sd_path_state_t sd_payload_state = {0, 0xFFFFFFFF, 0xFFFFFFFF, {0}, 0};

static const char *get_base_path_from_cmd(uint8_t cmd_id)
{
    switch (cmd_id)
    {
    case CMD_WRITE_SD_TEST:
    case CMD_READ_SD_TEST:
    case CMD_LIST_FILE_TEST:
    case CMD_DEL_FILE_TEST:
    case CMD_EDIT_FILE_TEST:
        return "test";

    case CMD_WRITE_SD_OBC:
    case CMD_READ_SD_OBC:
    case CMD_LIST_FILE_OBC:
    case CMD_DEL_FILE_OBC:
    case CMD_EDIT_FILE_OBC:
        return "obc";

    case CMD_WRITE_SD_PAYLOAD:
    case CMD_READ_SD_PAYLOAD:
    case CMD_LIST_FILE_PAYLOAD:
    case CMD_DEL_FILE_PAYLOAD:
    case CMD_EDIT_FILE_PAYLOAD:
        return "payload";

    default:
        return NULL;
    }
}

static sd_path_state_t *get_sd_state_from_cmd(uint8_t cmd_id)
{
    switch (cmd_id)
    {
    case CMD_WRITE_SD_TEST:
    case CMD_READ_SD_TEST:
    case CMD_LIST_FILE_TEST:
    case CMD_DEL_FILE_TEST:
    case CMD_EDIT_FILE_TEST:
        return &sd_test_state;

    case CMD_WRITE_SD_OBC:
    case CMD_READ_SD_OBC:
    case CMD_LIST_FILE_OBC:
    case CMD_DEL_FILE_OBC:
    case CMD_EDIT_FILE_OBC:
        return &sd_obc_state;

    case CMD_WRITE_SD_PAYLOAD:
    case CMD_READ_SD_PAYLOAD:
    case CMD_LIST_FILE_PAYLOAD:
    case CMD_DEL_FILE_PAYLOAD:
    case CMD_EDIT_FILE_PAYLOAD:
        return &sd_payload_state;

    default:
        return NULL;
    }
}

static uint8_t is_sd_cmd(uint8_t cmd_id)
{
    return (get_base_path_from_cmd(cmd_id) != NULL);
}

/**
 * @brief Handle CMD_READ_SD
 * @retval HAL_StatusTypeDef HAL_OK if success
 */
static HAL_StatusTypeDef handle_read_sd(msg_req_t *req, msg_resp_t *res,
                                        DWORD *readPoint, uint32_t *file_crc,
                                        const char *base_path)
{
    FATFS FatFs;
    FRESULT fres;
    FIL fil;
    UINT bytesRead;

    res->header.type = MSG_RESP;
    res->header.module_id = req->header.module_id;
    res->header.seq_id = req->header.seq_id;
    res->header.payload_len = sizeof(resp_read_sd_t);

    char filename[32];
    strcpy(filename, req->body.read_sd.filename);

    if (*readPoint == 0)
    {
        *file_crc = 0xFFFFFFFF;
    }

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
    {
    	res->header.cmd_id = RESP_ERR;
        return HAL_ERROR;
    }


    char fullpath[64];
    snprintf(fullpath, sizeof(fullpath), "%s/%s", base_path, filename);

    fres = f_open(&fil, fullpath, FA_READ);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        return HAL_ERROR;
    }

    fres = f_lseek(&fil, *readPoint);
    if (fres != FR_OK)
    {
        f_close(&fil);
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        return HAL_ERROR;
    }

    fres = f_read(&fil, res->body.read_sd_resp.data,
                 sizeof(res->body.read_sd_resp.data), &bytesRead);
    if (fres != FR_OK)
    {
        f_close(&fil);
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        return HAL_ERROR;
    }

    // Calculate CRC
    *file_crc = crc32(*file_crc, res->body.read_sd_resp.data, bytesRead);

    res->body.read_sd_resp.data_len = bytesRead;
    res->body.read_sd_resp.crc32 = *file_crc;
    *readPoint += bytesRead;

    // Check if last chunk
    if (bytesRead < sizeof(res->body.read_sd_resp.data))
    {
        res->header.cmd_id = RESP_OK;
        *readPoint = 0;
        *file_crc = 0xFFFFFFFF;
    }
    else
    {
        res->header.cmd_id = RESP_CONTINUE;
    }

    f_close(&fil);
    f_mount(NULL, "", 0);

    return HAL_OK;
}

/**
 * @brief Handle CMD_WRITE_SD
 * @retval HAL_StatusTypeDef HAL_OK if success
 */
static HAL_StatusTypeDef handle_write_sd(msg_req_t *req, msg_resp_t *res,
                                         uint32_t *write_crc, char *last_filename,
                                         const char *base_path)
{
    FATFS FatFs;
    FRESULT fres;
    FIL fil;
    UINT bytesWrote;

    res->header.type     = MSG_RESP;
    res->header.module_id = req->header.module_id;
    res->header.seq_id   = req->header.seq_id;
    res->header.payload_len = sizeof(resp_write_sd_t);

    char filename[32];
    strncpy(filename, req->body.write_sd.filename, sizeof(filename) - 1);
    filename[sizeof(filename) - 1] = '\0';

    /* Reset CRC เมื่อเป็นไฟล์ใหม่ */
    if (strcmp(last_filename, filename) != 0)
    {
        *write_crc = 0xFFFFFFFF;
        strncpy(last_filename, filename, 31);
        last_filename[31] = '\0';
    }

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
    {
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        *write_crc = 0xFFFFFFFF;
        memset(last_filename, 0, 32);
        return HAL_ERROR;
    }

    char fullpath[64];
    snprintf(fullpath, sizeof(fullpath), "%s/%s", base_path, filename);

    fres = f_open(&fil, fullpath, FA_WRITE | FA_OPEN_APPEND);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        *write_crc = 0xFFFFFFFF;
        memset(last_filename, 0, 32);
        return HAL_ERROR;
    }

    fres = f_write(&fil, req->body.write_sd.data,
                   req->body.write_sd.data_len, &bytesWrote);

    if (fres != FR_OK || bytesWrote != req->body.write_sd.data_len)
    {
        f_close(&fil);
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        *write_crc = 0xFFFFFFFF;
        memset(last_filename, 0, 32);
        return HAL_ERROR;
    }

    /* คำนวณ CRC สะสม */
    *write_crc = crc32(*write_crc, req->body.write_sd.data,
                       req->body.write_sd.data_len);
    res->body.write_sd_resp.crc32 = *write_crc;

    f_close(&fil);

    /* ตรวจสอบว่าเป็น chunk สุดท้ายหรือไม่ (data_len < max) */
    if (req->body.write_sd.data_len < sizeof(req->body.write_sd.data))
    {
        /* Last chunk → บันทึก CRC ลง crc.txt */
    	char crc_path[64];
    	snprintf(crc_path, sizeof(crc_path), "%s/CRC.TXT", base_path);
    	fres = f_open(&fil, crc_path, FA_WRITE | FA_OPEN_APPEND);
        if (fres == FR_OK)
        {
            char crc_line[128];
            snprintf(crc_line, sizeof(crc_line),
                     "%s 0x%08lX\n", filename, *write_crc);
            f_write(&fil, crc_line, strlen(crc_line), &bytesWrote);
            f_close(&fil);
        }

        res->header.cmd_id = RESP_OK;

        /* Reset state */
        *write_crc = 0xFFFFFFFF;
        memset(last_filename, 0, 32);
    }
    else
    {
        res->header.cmd_id = RESP_CONTINUE;
    }

    f_mount(NULL, "", 0);
    return HAL_OK;
}

/**
 * @brief Handle CMD_LIST_FILE
 * @retval HAL_StatusTypeDef HAL_OK if success
 *
 * ส่งทีละไฟล์ → RESP_CONTINUE จนหมด → RESP_OK
 * file_index เก็บ state ว่าอ่านมาถึงไฟล์ที่เท่าไหร่แล้ว
 */
static HAL_StatusTypeDef handle_list_file(msg_req_t *req, msg_resp_t *res,
                                          uint16_t *file_index,
                                          const char *base_path)
{
    FATFS FatFs;
    FRESULT fres;
    DIR dir;
    FILINFO fno;

    res->header.type     = MSG_RESP;
    res->header.module_id = req->header.module_id;
    res->header.seq_id   = req->header.seq_id;
    res->header.payload_len = sizeof(resp_list_file_t);

//    /* ถ้าเป็น request แรก (seq=1) ให้ reset index เสมอ */
//    if (req->header.seq_id == 1)
//        *file_index = 0;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
        return HAL_ERROR;

    fres = f_opendir(&dir, base_path);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        return HAL_ERROR;
    }

    uint16_t current_index = 0;
    uint8_t found = 0;

    while (1)
    {
        fres = f_readdir(&dir, &fno);
        if (fres != FR_OK || fno.fname[0] == 0)
            break;  /* หมด directory */

        /* ข้ามไดเรกทอรีย่อยและไฟล์ crc.txt */
        if (fno.fattrib & AM_DIR) continue;
        if (strcmp(fno.fname, "CRC.TXT") == 0) continue;
        if (strcmp(fno.fname, "CHECK.TXT") == 0) continue;

        if (current_index == *file_index)
        {
            found = 1;
            break;
        }
        current_index++;
    }

    f_closedir(&dir);
    f_mount(NULL, "", 0);

    if (found)
    {
        res->header.cmd_id = RESP_CONTINUE;
        strncpy(res->body.list_file_resp.filename, fno.fname,
                sizeof(res->body.list_file_resp.filename) - 1);
        res->body.list_file_resp.filename[sizeof(res->body.list_file_resp.filename) - 1] = '\0';
        (*file_index)++;
    }
    else
    {
        /* ไม่มีไฟล์เหลือ → จบ */
        res->header.cmd_id = RESP_OK;
        memset(res->body.list_file_resp.filename, 0,
               sizeof(res->body.list_file_resp.filename));
        *file_index = 0;
        res->header.payload_len = 0;
    }

    return HAL_OK;
}

/**
 * @brief Handle CMD_DEL_FILE
 * @retval HAL_StatusTypeDef HAL_OK if success
 */
HAL_StatusTypeDef handle_delete_file(msg_req_t *req, msg_resp_t *res, const char *base_path)
{
    FATFS FatFs;
    FRESULT fres;
    FIL fil;
    UINT bw;

    res->header.type     = MSG_RESP;
    res->header.module_id = req->header.module_id;
    res->header.cmd_id   = req->header.cmd_id;
    res->header.seq_id   = req->header.seq_id;
    res->header.payload_len = sizeof(resp_common_t);

    char filename[32];
    strncpy(filename, req->body.delete_file.filename, sizeof(filename) - 1);
    filename[sizeof(filename) - 1] = '\0';

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
    {
        res->body.common_resp.status = RESP_ERR;
        return HAL_ERROR;
    }

    /* Step 1: ลบไฟล์ */
    char fullpath[64];
    snprintf(fullpath, sizeof(fullpath), "%s/%s", base_path, filename);

    fres = f_unlink(fullpath);
//    if (fres == FR_OK || fres == FR_NO_FILE)
    if (fres == FR_OK)
    {
        res->body.common_resp.status = RESP_OK;

        /* Step 2: อัพเดท crc.txt (ลบ entry ของไฟล์นี้ออก) */
        static char buffer[2048];
        memset(buffer, 0, sizeof(buffer));

        char crc_path[64];
        snprintf(crc_path, sizeof(crc_path), "%s/CRC.TXT", base_path);

        fres = f_open(&fil, crc_path, FA_READ);
        if (fres == FR_OK)
        {
            char line[128];
            while (f_gets(line, sizeof(line), &fil))
            {
                char crc_filename[64];
                uint32_t crc_value;
                if (sscanf(line, "%63s %lx", crc_filename, &crc_value) == 2)
                {
                    if (strcasecmp(crc_filename, filename) != 0)
                    {
                        strncat(buffer, line,
                                sizeof(buffer) - strlen(buffer) - 1);
                    }
                }
            }
            f_close(&fil);

            /* เขียน crc.txt ใหม่ */
            fres = f_open(&fil, crc_path, FA_WRITE | FA_CREATE_ALWAYS);
            if (fres == FR_OK)
            {
                f_write(&fil, buffer, strlen(buffer), &bw);
                f_close(&fil);
            }
        }
    }
    else
    {
        res->body.common_resp.status = RESP_ERR;
    }

    f_mount(NULL, "", 0);
    return (res->body.common_resp.status == RESP_OK) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef handle_edit_file(msg_req_t *req, msg_resp_t *res,
                                          const char *base_path)
{
    FATFS FatFs;
    FRESULT fres;
    FIL fil;
    UINT bw;

    res->header.type        = MSG_RESP;
    res->header.module_id   = req->header.module_id;
    res->header.cmd_id      = req->header.cmd_id;
    res->header.seq_id      = req->header.seq_id;
    res->header.payload_len = sizeof(resp_write_sd_t);

    char filename[32];
    strncpy(filename, req->body.edit_file.filename, sizeof(filename) - 1);
    filename[sizeof(filename) - 1] = '\0';

    uint32_t offset   = req->body.edit_file.offset;
    uint32_t data_len = req->body.edit_file.data_len;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
    {
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        return HAL_ERROR;
    }

    char fullpath[64];
    snprintf(fullpath, sizeof(fullpath), "%s/%s", base_path, filename);

    /* เปิดแบบ read/write ไม่ truncate */
    fres = f_open(&fil, fullpath, FA_READ | FA_WRITE);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        return HAL_ERROR;
    }

    /* seek ไปยัง offset ที่ต้องการแก้ */
    fres = f_lseek(&fil, offset);
    if (fres != FR_OK)
    {
        f_close(&fil);
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        return HAL_ERROR;
    }

    /* เขียนข้อมูลใหม่ทับ */
    fres = f_write(&fil, req->body.edit_file.data, data_len, &bw);
    if (fres != FR_OK || bw != data_len)
    {
        f_close(&fil);
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        return HAL_ERROR;
    }

    f_close(&fil);

    /* คำนวณ CRC ใหม่ของทั้งไฟล์ */
    fres = f_open(&fil, fullpath, FA_READ);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        res->header.cmd_id = RESP_ERR;
        res->body.write_sd_resp.crc32 = 0;
        return HAL_ERROR;
    }

    uint32_t new_crc = 0xFFFFFFFF;
    uint8_t  read_buf[256];
    UINT     bytes_read;
    while (1)
    {
        fres = f_read(&fil, read_buf, sizeof(read_buf), &bytes_read);
        if (fres != FR_OK || bytes_read == 0) break;
        new_crc = crc32(new_crc, read_buf, bytes_read);
    }
    f_close(&fil);

//    /* อัพเดท CRC.TXT */
//    static char crc_buffer[2048];
//    memset(crc_buffer, 0, sizeof(crc_buffer));
//
//    FIL crc_fil;
//    fres = f_open(&crc_fil, "test/CRC.TXT", FA_READ);
//    if (fres == FR_OK)
//    {
//        char line[128];
//        while (f_gets(line, sizeof(line), &crc_fil))
//        {
//            /* trim \r\n */
//            char *p = line + strlen(line) - 1;
//            while (p >= line && (*p == '\r' || *p == '\n')) *p-- = '\0';
//
//            char     entry_filename[64];
//            uint32_t entry_crc;
//            if (sscanf(line, "%63s 0x%lX", entry_filename, &entry_crc) == 2
//             || sscanf(line, "%63s %lx",   entry_filename, &entry_crc) == 2)
//            {
//                if (strcasecmp(entry_filename, filename) == 0)
//                {
//                    /* บรรทัดของไฟล์นี้ → เขียน CRC ใหม่ */
//                    char new_line[128];
//                    snprintf(new_line, sizeof(new_line),
//                             "%s 0x%08lX", entry_filename, new_crc);
//                    strncat(crc_buffer, new_line,
//                            sizeof(crc_buffer) - strlen(crc_buffer) - 1);
//                }
//                else
//                {
//                    /* บรรทัดอื่น → คงไว้เหมือนเดิม */
//                    strncat(crc_buffer, line,
//                            sizeof(crc_buffer) - strlen(crc_buffer) - 1);
//                }
//                strncat(crc_buffer, "\n",
//                        sizeof(crc_buffer) - strlen(crc_buffer) - 1);
//            }
//        }
//        f_close(&crc_fil);
//
//        fres = f_open(&crc_fil, "test/CRC.TXT", FA_WRITE | FA_CREATE_ALWAYS);
//        if (fres == FR_OK)
//        {
//            f_write(&crc_fil, crc_buffer, strlen(crc_buffer), &bw);
//            f_close(&crc_fil);
//        }
//    }

    f_mount(NULL, "", 0);

    res->header.cmd_id        = RESP_OK;
    res->body.write_sd_resp.crc32 = new_crc;
    return HAL_OK;
}

/**
 * @brief Write configuration to Flash
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_Write_Config(uint64_t circular_size, uint64_t circular_interval, uint64_t integrity_interval)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    // Unlock Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return status;

    // Erase config page
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = FLASH_CONFIG_PAGE;
    eraseInitStruct.NbPages = 1;
    eraseInitStruct.Banks = FLASH_CONFIG_BANK;

    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    // Write configuration values
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDR_CIRCULAR_SIZE, circular_size);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDR_CIRCULAR_INTERVAL, circular_interval);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDR_INTEGRITY_INTERVAL, integrity_interval);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    // Lock Flash
    HAL_FLASH_Lock();

    return HAL_OK;
}

/**
 * @brief Read configuration from Flash
 */
void Flash_Read_Config(uint64_t *circular_size, uint64_t *circular_interval, uint64_t *integrity_interval)
{
    // Read directly from Flash memory
    *circular_size = *(__IO uint64_t *)FLASH_ADDR_CIRCULAR_SIZE;
    *circular_interval = *(__IO uint64_t *)FLASH_ADDR_CIRCULAR_INTERVAL;
    *integrity_interval = *(__IO uint64_t *)FLASH_ADDR_INTEGRITY_INTERVAL;
}

/**
 * @brief Check if configuration is valid (not erased/corrupted)
 * @retval 1 if valid, 0 if invalid
 */
uint8_t Flash_Is_Config_Valid(void)
{
    uint64_t size = *(__IO uint64_t *)FLASH_ADDR_CIRCULAR_SIZE;
    uint64_t interval1 = *(__IO uint64_t *)FLASH_ADDR_CIRCULAR_INTERVAL;
    uint64_t interval2 = *(__IO uint64_t *)FLASH_ADDR_INTEGRITY_INTERVAL;

    // Check if Flash is erased (all 0xFF)
    if (size == 0xFFFFFFFFFFFFFFFFULL || interval1 == 0xFFFFFFFFFFFFFFFFULL || interval2 == 0xFFFFFFFFFFFFFFFFULL)
    {
        return 0;  // Invalid (erased)
    }

    // Basic sanity check
    if (size == 0 || interval1 == 0 || interval2 == 0)
    {
        return 0;  // Invalid
    }

    return 1;  // Valid
}

static void led3_blink(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
    	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        if (i < n - 1)
            osDelay(100);
    }
}

static float read_internal_temperature(void)
{
    uint32_t adc_temp = 0;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
        adc_temp = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    float cal30  = (float)TEMP30_CAL_ADDR;
    float cal130 = (float)TEMP130_CAL_ADDR;

    return (((float)adc_temp - cal30) / (cal130 - cal30)) * 100.0f + 30.0f;
}

static HAL_StatusTypeDef write_obc_file_via_handler(const char *filename,
                                                    const uint8_t *data,
                                                    uint32_t data_len)
{
    msg_req_t req;
    msg_resp_t res;

    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));

    req.header.type = MSG_REQ;
    req.header.module_id = MOD_SD;
    req.header.cmd_id = CMD_WRITE_SD_OBC;
    req.header.seq_id = 1;
    req.header.payload_len = sizeof(cmd_write_sd_t);

    strncpy(req.body.write_sd.filename, filename, sizeof(req.body.write_sd.filename) - 1);
    req.body.write_sd.filename[sizeof(req.body.write_sd.filename) - 1] = '\0';

    if (data_len > sizeof(req.body.write_sd.data))
        data_len = sizeof(req.body.write_sd.data);

    memcpy(req.body.write_sd.data, data, data_len);
    req.body.write_sd.data_len = data_len;

    return handle_write_sd(&req, &res, &obc_write_crc, obc_last_filename, "obc");
}


static HAL_StatusTypeDef circular_cleanup_path(const char *base_path, uint32_t max_files)
{
    FATFS   FatFs;
    DIR     dir;
    FILINFO fno;
    FRESULT fres;

    if (max_files == 0)
        return HAL_OK;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
        return HAL_ERROR;

    fres = f_opendir(&dir, base_path);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        return HAL_ERROR;
    }

    uint32_t file_count = 0;
    for (;;)
    {
        fres = f_readdir(&dir, &fno);
        if (fres != FR_OK || fno.fname[0] == 0) break;

        if (fno.fattrib & AM_DIR) continue;
        if (strcmp(fno.fname, "CRC.TXT") == 0) continue;
        if (strcmp(fno.fname, "CHECK.TXT") == 0) continue;

        file_count++;
    }

    f_closedir(&dir);
    f_mount(NULL, "", 0);

    while (file_count > max_files)
    {
        fres = f_mount(&FatFs, "", 1);
        if (fres != FR_OK)
            return HAL_ERROR;

        fres = f_opendir(&dir, base_path);
        if (fres != FR_OK)
        {
            f_mount(NULL, "", 0);
            return HAL_ERROR;
        }

        char oldest_filename[32] = {0};
        uint32_t oldest_time = 0xFFFFFFFF;

        for (;;)
        {
            fres = f_readdir(&dir, &fno);
            if (fres != FR_OK || fno.fname[0] == 0) break;

            if (fno.fattrib & AM_DIR) continue;
            if (strcmp(fno.fname, "CRC.TXT") == 0) continue;
            if (strcmp(fno.fname, "CHECK.TXT") == 0) continue;

            uint32_t file_time = ((uint32_t)fno.fdate << 16) | fno.ftime;
            if (file_time < oldest_time)
            {
                oldest_time = file_time;
                strncpy(oldest_filename, fno.fname, sizeof(oldest_filename) - 1);
                oldest_filename[sizeof(oldest_filename) - 1] = '\0';
            }
        }

        f_closedir(&dir);
        f_mount(NULL, "", 0);

        if (oldest_filename[0] == '\0')
            return HAL_ERROR;

        msg_req_t  dummy_req;
        msg_resp_t dummy_res;

        memset(&dummy_req, 0, sizeof(dummy_req));
        memset(&dummy_res, 0, sizeof(dummy_res));

        strncpy(dummy_req.body.delete_file.filename,
                oldest_filename,
                sizeof(dummy_req.body.delete_file.filename) - 1);
        dummy_req.body.delete_file.filename[sizeof(dummy_req.body.delete_file.filename) - 1] = '\0';

        if (handle_delete_file(&dummy_req, &dummy_res, base_path) != HAL_OK)
            return HAL_ERROR;

        file_count--;
    }

    return HAL_OK;
}

static uint32_t get_next_obc_index(void)
{
    FATFS   FatFs;
    DIR     dir;
    FILINFO fno;
    FRESULT fres;
    uint32_t max_idx = 0;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
        return 1;

    fres = f_opendir(&dir, "obc");
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        return 1;
    }

    for (;;)
    {
        fres = f_readdir(&dir, &fno);
        if (fres != FR_OK || fno.fname[0] == 0) break;

        if (fno.fattrib & AM_DIR) continue;
        if (strcmp(fno.fname, "CRC.TXT") == 0) continue;
        if (strcmp(fno.fname, "CHECK.TXT") == 0) continue;

        uint32_t idx = 0;
        if (sscanf(fno.fname, "obc%lu.txt", &idx) == 1)
        {
            if (idx > max_idx)
                max_idx = idx;
        }
    }

    f_closedir(&dir);
    f_mount(NULL, "", 0);

    return max_idx + 1;
}

static void integrity_check_path(const char *base_path,
                                 uint32_t *line_index)
{
    FATFS   FatFs;
    FIL     crc_file, target_file, check_file;
    FRESULT fres;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
        return;

    char crc_path[64];
    snprintf(crc_path, sizeof(crc_path), "%s/CRC.TXT", base_path);

    fres = f_open(&crc_file, crc_path, FA_READ);
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        return;
    }

    char line[128] = {0};
    uint8_t found = 1;

    for (uint32_t i = 0; i <= *line_index; i++)
    {
        if (!f_gets(line, sizeof(line), &crc_file))
        {
            found = 0;
            break;
        }
    }

    f_close(&crc_file);

    if (found == 0)
    {
        f_mount(NULL, "", 0);
        *line_index = 0;
        return;
    }
    else if (found == 1)
    {
        char check_filename[64] = {0};
        uint32_t expected_crc = 0xFFFFFFFF;;

        if (sscanf(line, "%63s 0x%lx", check_filename, &expected_crc) == 2)
        {
            char fullpath[96];
            snprintf(fullpath, sizeof(fullpath), "%s/%s", base_path, check_filename);

            fres = f_open(&target_file, fullpath, FA_READ);
            if (fres == FR_OK)
            {
                uint32_t computed_crc = 0xFFFFFFFF;;
                uint8_t buf[256];
                UINT br;

                do
                {
                    fres = f_read(&target_file, buf, sizeof(buf), &br);
                    if (fres != FR_OK) break;
                    if (br > 0)
                        computed_crc = crc32(computed_crc, buf, br);
                } while (br > 0);

                f_close(&target_file);

                if (computed_crc != expected_crc)
                {
                    char check_path[64];
                    snprintf(check_path, sizeof(check_path), "%s/CHECK.TXT", base_path);

                    fres = f_open(&check_file, check_path, FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
                    if (fres == FR_OK)
                    {
                        char msg[160];
                        UINT bw;
                        snprintf(msg, sizeof(msg),
                                 "%s CRC MISMATCH expected=0x%08lX got=0x%08lX\r\n",
                                 check_filename, expected_crc, computed_crc);
                        f_write(&check_file, msg, strlen(msg), &bw);
                        f_close(&check_file);
                    }
                }
            }
            else
            {
                char check_path[64];
                snprintf(check_path, sizeof(check_path), "%s/CHECK.TXT", base_path);

                fres = f_open(&check_file, check_path, FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
                if (fres == FR_OK)
                {
                    char msg[128];
                    UINT bw;
                    snprintf(msg, sizeof(msg),
                             "%s FILE NOT FOUND\r\n", check_filename);
                    f_write(&check_file, msg, strlen(msg), &bw);
                    f_close(&check_file);
                }
            }
            f_mount(NULL, "", 0);
            (*line_index)++;
        }
        else
        {
        	f_mount(NULL, "", 0);
        	(*line_index)++;
        	return;
        }
    }
}


static HAL_StatusTypeDef write_payload_file_via_handler(const char *filename,
                                                        const uint8_t *data,
                                                        uint32_t data_len)
{
    msg_req_t req;
    msg_resp_t res;

    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));

    req.header.type = MSG_REQ;
    req.header.module_id = MOD_SD;
    req.header.cmd_id = CMD_WRITE_SD_PAYLOAD;
    req.header.seq_id = 1;
    req.header.payload_len = sizeof(cmd_write_sd_t);

    strncpy(req.body.write_sd.filename, filename,
            sizeof(req.body.write_sd.filename) - 1);
    req.body.write_sd.filename[sizeof(req.body.write_sd.filename) - 1] = '\0';

    if (data_len > sizeof(req.body.write_sd.data))
        data_len = sizeof(req.body.write_sd.data);

    memcpy(req.body.write_sd.data, data, data_len);
    req.body.write_sd.data_len = data_len;

    return handle_write_sd(&req, &res,
                           &payload_write_crc,
                           payload_last_filename,
                           "payload");
}
static uint32_t get_next_payload_index(void)
{
    FATFS   FatFs;
    DIR     dir;
    FILINFO fno;
    FRESULT fres;
    uint32_t max_idx = 0;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
        return 1;

    fres = f_opendir(&dir, "payload");
    if (fres != FR_OK)
    {
        f_mount(NULL, "", 0);
        return 1;
    }

    for (;;)
    {
        fres = f_readdir(&dir, &fno);
        if (fres != FR_OK || fno.fname[0] == 0) break;

        if (fno.fattrib & AM_DIR) continue;
        if (strcmp(fno.fname, "CRC.TXT") == 0) continue;
        if (strcmp(fno.fname, "CHECK.TXT") == 0) continue;

        uint32_t idx = 0;
        if (sscanf(fno.fname, "payload%lu.txt", &idx) == 1)
        {
            if (idx > max_idx)
                max_idx = idx;
        }
    }

    f_closedir(&dir);
    f_mount(NULL, "", 0);

    return max_idx + 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  osDelay(3000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_uart_rx_task */
/**
* @brief Function implementing the UartRxTask thread.
* @param argument: Not used
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        if (dma_state == 0)
        {
        	osSignalSet(UartRxTaskHandle, 0x01);
        }
        else if (dma_state == 1)
        {
            osSignalSet(UartRxTaskHandle, 0x02);
        }
    }
}
/* USER CODE END Header_uart_rx_task */
void uart_rx_task(void const * argument)
{
  /* USER CODE BEGIN uart_rx_task */
    osEvent evt;
  /* Infinite loop */
    for (;;)
    {
        msg_req_t *req = (msg_req_t *)osPoolAlloc(req_pool);
        if (req == NULL)
        {
            osDelay(100);
            continue;
        }

        // Receive Header
        HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)&req->header, sizeof(msg_header_t));
        evt = osSignalWait(0x01, osWaitForever);

        if (evt.status == osEventSignal)
        {
            size_t expected_size = 0;

            switch (req->header.cmd_id)
            {
            case CMD_READ_SD_TEST:
            case CMD_READ_SD_OBC:
            case CMD_READ_SD_PAYLOAD:
                expected_size = sizeof(cmd_read_sd_t);
                break;

            case CMD_WRITE_SD_TEST:
            case CMD_WRITE_SD_OBC:
            case CMD_WRITE_SD_PAYLOAD:
                expected_size = sizeof(cmd_write_sd_t);
                break;

            case CMD_DEL_FILE_TEST:
            case CMD_DEL_FILE_OBC:
            case CMD_DEL_FILE_PAYLOAD:
                expected_size = sizeof(cmd_delete_file_t);
                break;

            case CMD_WRITE_CONFIG:
                expected_size = sizeof(cmd_write_config_t);
                break;

            case CMD_EDIT_FILE_TEST:
            case CMD_EDIT_FILE_OBC:
            case CMD_EDIT_FILE_PAYLOAD:
                expected_size = sizeof(cmd_edit_file_t);
                break;

            case CMD_LIST_FILE_TEST:
            case CMD_LIST_FILE_OBC:
            case CMD_LIST_FILE_PAYLOAD:
            case CMD_READ_CONFIG:
            case CMD_INIT_SD:
            case CMD_DEINIT_SD:
                expected_size = 0;
                osMessagePut(UartRxQueueHandle, (uint32_t)req, osWaitForever);
                break;

            default:
                osPoolFree(req_pool, req);
                req = NULL;
                break;
            }



            if (expected_size > 0)
            {
                dma_state = 1;
                HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)&req->body, expected_size);
                evt = osSignalWait(0x02, osWaitForever);

                if (evt.status == osEventSignal)
                {
                    osMessagePut(UartRxQueueHandle, (uint32_t)req, osWaitForever);
                    dma_state = 0;
                }
            }
        }
    }
  /* USER CODE END uart_rx_task */
}

/* USER CODE BEGIN Header_uart_tx_task */
/**
* @brief Function implementing the UartTxTask thread.
* @param argument: Not used
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        osSignalSet(UartTxTaskHandle, 0x04);
    }
}
/* USER CODE END Header_uart_tx_task */
void uart_tx_task(void const * argument)
{
  /* USER CODE BEGIN uart_tx_task */
	osEvent evt;
  /* Infinite loop */
    for (;;)
    {
        evt = osMessageGet(UartTxQueueHandle, osWaitForever);

        if (evt.status == osEventMessage)
        {
            msg_resp_t *res = evt.value.p;

            size_t h = sizeof(res->header);
            size_t b = res->header.payload_len;
            size_t total_size = h + b;

            // Clear pending signal by set timeout = 0 (non-blocking)
            osSignalWait(0x04, 0);

            HAL_StatusTypeDef hal_s = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)res, total_size);

            if (hal_s == HAL_OK)
            {
                evt = osSignalWait(0x04, 5000);
                if (evt.status != osEventSignal)
                {
                    HAL_UART_AbortTransmit(&hlpuart1);
                }
            }

            osPoolFree(resp_pool, res);
        }
    }
  /* USER CODE END uart_tx_task */
}

/* USER CODE BEGIN Header_disp_task */
/**
* @brief Function implementing the DispTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_disp_task */
void disp_task(void const * argument)
{
  /* USER CODE BEGIN disp_task */
	osEvent evt;
  /* Infinite loop */
    for (;;)
    {
        evt = osMessageGet(UartRxQueueHandle, osWaitForever);

        if (evt.status == osEventMessage)
        {
            msg_req_t *req = evt.value.p;

            if (is_sd_cmd(req->header.cmd_id) ||
                req->header.cmd_id == CMD_INIT_SD ||
                req->header.cmd_id == CMD_DEINIT_SD)
            {
                osMessagePut(SdQueueHandle, (uint32_t)req, osWaitForever);
            }
            else if (req->header.cmd_id == CMD_READ_CONFIG ||
                     req->header.cmd_id == CMD_WRITE_CONFIG)
            {
                osMessagePut(FlashQueueHandle, (uint32_t)req, osWaitForever);
            }
            else
            {
                osPoolFree(req_pool, req);
            }
        }
    }
  /* USER CODE END disp_task */
}

/* USER CODE BEGIN Header_sd_task */
/**
* @brief Function implementing the SdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sd_task */
void sd_task(void const * argument)
{
  /* USER CODE BEGIN sd_task */
    osEvent evt;
    HAL_StatusTypeDef status;
  /* Infinite loop */
    for(;;)
        {
            evt = osMessageGet(SdQueueHandle, osWaitForever);

            if (evt.status == osEventMessage)
            {
                msg_req_t  *req = evt.value.p;
                msg_resp_t *res = (msg_resp_t *)osPoolAlloc(resp_pool);

                if (res == NULL)
                {
                    osPoolFree(req_pool, req);
                    continue;
                }

                const char *base_path = get_base_path_from_cmd(req->header.cmd_id);
                sd_path_state_t *state = get_sd_state_from_cmd(req->header.cmd_id);

                switch (req->header.cmd_id)
                {
                case CMD_READ_SD_TEST:
                case CMD_READ_SD_OBC:
                case CMD_READ_SD_PAYLOAD:
                    status = handle_read_sd(req, res,
                                            &state->readPoint,
                                            &state->file_crc,
                                            base_path);
                    break;

                case CMD_WRITE_SD_TEST:
                case CMD_WRITE_SD_OBC:
                case CMD_WRITE_SD_PAYLOAD:
                    status = handle_write_sd(req, res,
                                             &state->write_crc,
                                             state->last_filename,
                                             base_path);
                    break;

                case CMD_LIST_FILE_TEST:
                case CMD_LIST_FILE_OBC:
                case CMD_LIST_FILE_PAYLOAD:
                    status = handle_list_file(req, res,
                                              &state->file_index,
                                              base_path);
                    break;

                case CMD_DEL_FILE_TEST:
                case CMD_DEL_FILE_OBC:
                case CMD_DEL_FILE_PAYLOAD:
                    status = handle_delete_file(req, res, base_path);
                    break;

                case CMD_EDIT_FILE_TEST:
                case CMD_EDIT_FILE_OBC:
                case CMD_EDIT_FILE_PAYLOAD:
                    status = handle_edit_file(req, res, base_path);
                    break;

                case CMD_INIT_SD:
                {
                    FATFS FatFs;
                    FRESULT fres;

                    res->header.type = MSG_RESP;
                    res->header.module_id = req->header.module_id;
                    res->header.cmd_id = req->header.cmd_id;
                    res->header.seq_id = req->header.seq_id;
                    res->header.payload_len = sizeof(resp_common_t);

                    fres = f_mount(&FatFs, "", 1);
                    res->body.common_resp.status = (fres == FR_OK) ? RESP_OK : RESP_ERR;
//                    f_mount(NULL, "", 0);

                    status = HAL_OK;
                    break;
                }

                case CMD_DEINIT_SD:
                {
                    res->header.type = MSG_RESP;
                    res->header.module_id = req->header.module_id;
                    res->header.cmd_id = req->header.cmd_id;
                    res->header.seq_id = req->header.seq_id;
                    res->header.payload_len = sizeof(resp_common_t);

                    f_mount(NULL, "", 0);
                    res->body.common_resp.status = RESP_OK;
                    status = HAL_OK;
                    break;
                }

                default:
                    status = HAL_ERROR;
                    break;
                }

//                if (status == HAL_OK)
//                {
//                    osStatus s = osMessagePut(UartTxQueueHandle, (uint32_t)res, osWaitForever);
//                    if (s != osOK)
//                        osPoolFree(resp_pool, res);
//                }
//                else
//                {
//                    osPoolFree(resp_pool, res);
//                }

                osMessagePut(UartTxQueueHandle, (uint32_t)res, osWaitForever);
                osPoolFree(req_pool, req);
            }
        }
  /* USER CODE END sd_task */
}

/* USER CODE BEGIN Header_flash_task */
/**
* @brief Function implementing the FlashTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flash_task */
void flash_task(void const * argument)
{
  /* USER CODE BEGIN flash_task */
	osEvent evt;
  /* Infinite loop */
    for(;;)
    {
        evt = osMessageGet(FlashQueueHandle, osWaitForever);

        if (evt.status == osEventMessage)
        {
        	msg_req_t *req = evt.value.p;
        	msg_resp_t *res = (msg_resp_t *)osPoolAlloc(resp_pool);

        	if (res == NULL) {
        	    osPoolFree(req_pool, req);
        	    continue;
        	}

            if (req->header.cmd_id == CMD_READ_CONFIG)
            {
                res->header.type = MSG_RESP;
                res->header.module_id = req->header.module_id;
                res->header.cmd_id = req->header.cmd_id;
                res->header.seq_id = req->header.seq_id;
                res->header.payload_len = sizeof(resp_read_config_t);

                res->body.read_config_resp.circular_size = CIRCULAR_SIZE;
                res->body.read_config_resp.circular_interval = CIRCULAR_INTERVAL;
                res->body.read_config_resp.integrity_check_interval = INTEGRITY_CHECK_INTERVAL;

                osPoolFree(req_pool, req);
                osMessagePut(UartTxQueueHandle, (uint32_t)res, osWaitForever);
            }
            else if (req->header.cmd_id == CMD_WRITE_CONFIG)
            {
                res->header.type = MSG_RESP;
                res->header.module_id = req->header.module_id;
                res->header.cmd_id = req->header.cmd_id;
                res->header.seq_id = req->header.seq_id;
                res->header.payload_len = sizeof(resp_common_t);

                uint64_t circular_size = req->body.write_config.circular_size;
                uint64_t circular_interval = req->body.write_config.circular_interval;
                uint64_t integrity_interval = req->body.write_config.integrity_check_interval;

                HAL_StatusTypeDef flash_status = Flash_Write_Config(circular_size,
                                                                     circular_interval,
                                                                     integrity_interval);

                if (flash_status == HAL_OK)
                {
                    // Update global variables
                    CIRCULAR_SIZE = circular_size;
                    CIRCULAR_INTERVAL = circular_interval;
                    INTEGRITY_CHECK_INTERVAL = integrity_interval;

                    res->body.common_resp.status = RESP_OK;
                }
                else
                {
                    res->body.common_resp.status = RESP_ERR;
                }

                osPoolFree(req_pool, req);
                osMessagePut(UartTxQueueHandle, (uint32_t)res, osWaitForever);
            }
        }
    }
  /* USER CODE END flash_task */
}

/* USER CODE BEGIN Header_circular_task */
/**
* @brief Function implementing the CircularTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_circular_task */
void circular_task(void const * argument)
{
  /* USER CODE BEGIN circular_task */
    osEvent evt;
  /* Infinite loop */
    for(;;)
    {
        evt = osSignalWait(0x10, (uint32_t)CIRCULAR_INTERVAL);

        if (evt.status == osEventTimeout || evt.status == osEventSignal)
        {
            if (CIRCULAR_SIZE == 0 || CIRCULAR_INTERVAL == 0)
            {
                osDelay(1000);
                continue;
            }

            led3_blink(1);

            circular_cleanup_path("test", (uint32_t)CIRCULAR_SIZE);
            circular_cleanup_path("obc",  (uint32_t)CIRCULAR_SIZE);
            circular_cleanup_path("payload", (uint32_t)CIRCULAR_SIZE);

            led3_blink(1);
        }
    }
  /* USER CODE END circular_task */
}

/* USER CODE BEGIN Header_file_integrity_check */
/**
* @brief Function implementing the FileIntegrityCh thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_file_integrity_check */
void file_integrity_check(void const * argument)
{
  /* USER CODE BEGIN file_integrity_check */
    osEvent evt;

    static uint32_t test_line_index = 0;
    static uint32_t obc_line_index = 0;
    static uint32_t payload_line_index = 0;
  /* Infinite loop */
    for(;;)
    {
        evt = osSignalWait(0x20, (uint32_t)INTEGRITY_CHECK_INTERVAL);

        if (evt.status == osEventTimeout || evt.status == osEventSignal)
        {
            if (INTEGRITY_CHECK_INTERVAL == 0)
            {
                osDelay(1000);
                continue;
            }

            led3_blink(3);

            integrity_check_path("test", &test_line_index);
            integrity_check_path("obc", &obc_line_index);
            integrity_check_path("payload", &payload_line_index);

            led3_blink(3);
        }
    }
  /* USER CODE END file_integrity_check */
}

/* USER CODE BEGIN Header_obc_log_file */
/**
* @brief Function implementing the obcLogFile thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_obc_log_file */
void obc_log_file(void const * argument)
{
  /* USER CODE BEGIN obc_log_file */


    char line[96];
    char file_buf[256];
    char filename[32];


    uint8_t line_count = 0;
    uint16_t buf_len = 0;



    uint32_t current_file = get_next_obc_index();

    memset(file_buf, 0, sizeof(file_buf));
  /* Infinite loop */
    for (;;)
     {
    	led3_blink(2);

         float t = read_internal_temperature();

         RTC_TimeTypeDef sTime;
         RTC_DateTypeDef sDate;
         HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
         HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

         int len = snprintf(line, sizeof(line),
                            "%02d-%02d-20%02d %02d:%02d:%02d Temp=%.2f C\r\n",
                            sDate.Date, sDate.Month, sDate.Year,
                            sTime.Hours, sTime.Minutes, sTime.Seconds,
                            t);

         if (len > 0 && (buf_len + len) < sizeof(file_buf))
         {
             memcpy(&file_buf[buf_len], line, len);
             buf_len += len;
             line_count++;
         }

         if (line_count >= LINES_PER_FILE)
         {
        	 snprintf(filename, sizeof(filename), "obc%lu.txt", current_file);


             obc_write_crc = 0xFFFFFFFF;
             memset(obc_last_filename, 0, sizeof(obc_last_filename));

             if (write_obc_file_via_handler(filename, (uint8_t *)file_buf, buf_len) == HAL_OK)
             {
                 // เขียนสำเร็จ
             }
             else
             {
                 // เขียนไม่สำเร็จ
             }

             memset(file_buf, 0, sizeof(file_buf));
             buf_len = 0;
             line_count = 0;

             current_file++;
//             if (current_file > OBC_MAX_FILES)
//             {
//                 current_file = 1;
//             }
         }
         led3_blink(2);
         osDelay(OBC_LOG_PERIOD_MS);
     }
  /* USER CODE END obc_log_file */
}

/* USER CODE BEGIN Header_spi_rx_task */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3)
    {
        osSignalSet(SpiRxTaskHandle, 0x08);
    }
}
/**
* @brief Function implementing the SpiRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spi_rx_task */
void spi_rx_task(void const * argument)
{
  /* USER CODE BEGIN spi_rx_task */
    osEvent evt_Signal;
  /* Infinite loop */
  for(;;)
  {
      payload_msg_t *data = (payload_msg_t *)osPoolAlloc(payload_pool);
      if (data == NULL)
      {
          osDelay(10);
          continue;
      }

      HAL_SPI_Receive_DMA(&hspi3, (uint8_t*)data, sizeof(payload_msg_t));
      evt_Signal = osSignalWait(0x08, osWaitForever);

      if (evt_Signal.status == osEventSignal)
      {
          if (osMessagePut(PayloadQueueHandle, (uint32_t)data, osWaitForever) != osOK)
          {
              osPoolFree(payload_pool, data);
          }
      }
      else
      {
          osPoolFree(payload_pool, data);
      }
  }
  /* USER CODE END spi_rx_task */
}

/* USER CODE BEGIN Header_payload_log_file */
/**
* @brief Function implementing the PayloadLogFile thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_payload_log_file */
void payload_log_file(void const * argument)
{
  /* USER CODE BEGIN payload_log_file */
    osEvent evt;

    char line[96];
    char file_buf[256];
    char filename[32];

    static uint8_t pl_log_state = 0;

    uint8_t line_count = 0;
    uint16_t buf_len = 0;
//    uint32_t current_file = get_next_payload_index();
    uint32_t current_file;
    memset(file_buf, 0, sizeof(file_buf));
  /* Infinite loop */
    for(;;)
    {


        evt = osMessageGet(PayloadQueueHandle, osWaitForever);

        if (evt.status == osEventMessage)
        {
      	  led3_blink(2);

			if (pl_log_state == 0)
			{
				current_file = get_next_payload_index();
				pl_log_state = 1;
			}

            payload_msg_t *data = (payload_msg_t *)evt.value.p;

            RTC_TimeTypeDef sTime;
            RTC_DateTypeDef sDate;

            HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
            HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

            float temp_c = ((float)data->temp_centi) / 100.0f;

            int len = snprintf(line, sizeof(line),
                               "%02d-%02d-20%02d %02d:%02d:%02d Temp=%.2f C\r\n",
                               sDate.Date, sDate.Month, sDate.Year,
                               sTime.Hours, sTime.Minutes, sTime.Seconds,
                               temp_c);

            if (len > 0 && (buf_len + len) < sizeof(file_buf))
            {
                memcpy(&file_buf[buf_len], line, len);
                buf_len += len;
                line_count++;
            }

            if (line_count >= LINES_PER_FILE)
            {
                snprintf(filename, sizeof(filename), "payload%lu.txt", current_file);

                payload_write_crc = 0xFFFFFFFF;
                memset(payload_last_filename, 0, sizeof(payload_last_filename));

                if (write_payload_file_via_handler(filename,
                                                   (uint8_t *)file_buf,
                                                   buf_len) == HAL_OK)
                {
                    // success
                }
                else
                {
                    // fail
                }

                memset(file_buf, 0, sizeof(file_buf));
                buf_len = 0;
                line_count = 0;
                current_file++;
            }

            led3_blink(2);
            osPoolFree(payload_pool, data);
        }
    }
  /* USER CODE END payload_log_file */
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
