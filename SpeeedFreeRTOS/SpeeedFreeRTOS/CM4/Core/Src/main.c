/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "inputBuffer.h"
#include <inttypes.h>
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for txTask */
osThreadId_t txTaskHandle;
const osThreadAttr_t txTask_attributes = {
  .name = "txTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for readSensors */
osThreadId_t readSensorsHandle;
const osThreadAttr_t readSensors_attributes = {
  .name = "readSensors",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendCANTask */
osThreadId_t sendCANTaskHandle;
const osThreadAttr_t sendCANTask_attributes = {
  .name = "sendCANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for canTestTask */
osThreadId_t canTestTaskHandle;
const osThreadAttr_t canTestTask_attributes = {
  .name = "canTestTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for txSemaphore */
osSemaphoreId_t txSemaphoreHandle;
const osSemaphoreAttr_t txSemaphore_attributes = {
  .name = "txSemaphore"
};
/* Definitions for readSensorSemaphore */
osSemaphoreId_t readSensorSemaphoreHandle;
const osSemaphoreAttr_t readSensorSemaphore_attributes = {
  .name = "readSensorSemaphore"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void startTxTask(void *argument);
void startReadSensor(void *argument);
void startSendCAN(void *argument);
void startCanTest(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RPMSG_SERVICE_NAME		"openamp_test"

//FDCAN STUFF (TESTING)
FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t txData[8];
uint8_t rxData[8];
uint32_t count = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) //if new data
			{
		count++;
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData)
				!= HAL_OK) {
			Error_Handler();
		}

		if (HAL_FDCAN_ActivateNotification(hfdcan,
		FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
			Error_Handler();
		}
	}
}

//testing only
float testing = 1.2;

typedef enum {
	TORQUE_SENSOR, BREAK_SENSOR
} DataIdentifier;
typedef struct TorqueEncoder {
	DataIdentifier id;
	uint32_t valueInt;
	uint32_t sensor1;
	float sensorPercent0;
	float sensorPercent1;
	float travelPercent;
	float valueFloat;
	bool valueBool;
} TorqueEncoder;

struct TorqueEncoder *test;

//UART RECEIVE BUFFER (used for testing)
uint8_t rx_buffer[100];
int indx = 0;

//TIMER STUFF (used for testing)
uint32_t start = 0;
uint32_t end = 0;
uint32_t executionTime = 0;

//below is openamp ipc messaging
volatile uint32_t received_data;

static struct rpmsg_endpoint rp_endpoint;
uint32_t message = 0;

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
		size_t len, uint32_t src, void *prv) {
	DataIdentifier *id = (DataIdentifier*) data;
	if (*id == TORQUE_SENSOR) {
		test = (TorqueEncoder*) data;
	}

//	received_data = test->valueInt; //testing struct(it works)

	if (osSemaphoreRelease(txSemaphoreHandle) != osOK)
		Error_Handler();	//free semaphore whenever there is a msg received
	xTaskNotifyGive(sendCANTaskHandle); // Notify sendCANTask to start

	return 0;
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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
	PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart3, rx_buffer, sizeof(rx_buffer));	// wait for next data
	HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, sizeof(rx_buffer));

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of txSemaphore */
  txSemaphoreHandle = osSemaphoreNew(1, 1, &txSemaphore_attributes);

  /* creation of readSensorSemaphore */
  readSensorSemaphoreHandle = osSemaphoreNew(1, 1, &readSensorSemaphore_attributes);

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

  /* creation of txTask */
  txTaskHandle = osThreadNew(startTxTask, NULL, &txTask_attributes);

  /* creation of readSensors */
  readSensorsHandle = osThreadNew(startReadSensor, NULL, &readSensors_attributes);

  /* creation of sendCANTask */
  sendCANTaskHandle = osThreadNew(startSendCAN, NULL, &sendCANTask_attributes);

  /* creation of canTestTask */
  canTestTaskHandle = osThreadNew(startCanTest, NULL, &canTestTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
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
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 64;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

	FDCAN_FilterTypeDef canFilterConfig;
	canFilterConfig.IdType = FDCAN_STANDARD_ID; //standard CAN
	canFilterConfig.FilterIndex = 0; //FILTER BANK
	canFilterConfig.FilterType = FDCAN_FILTER_MASK; // SAME AS FILTER MODE FDCAN_FILTER_MASK
	canFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //SAME AS FIFO ASSIGNMENT
	canFilterConfig.FilterID1 = 0x000;	//accept from given id
	canFilterConfig.FilterID2 = 0x000;
	canFilterConfig.RxBufferIndex = 0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	indx = Size;
	if (huart->Instance == USART3) {
		// Notify the UART task when data is received
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(readSensorsHandle, &xHigherPriorityTaskWoken);//notify given task from ISR, cant use xTaskNotify
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

uint32_t* parseCSV(char *data)	//data should be coming in as "sensor1, sensor2"
{
	char *token;
	//malloc array size 20 (at most 20 character input)
	uint32_t *valueArray = (uint32_t*) pvPortMalloc(20 * sizeof(uint32_t));
	uint32_t i = 0;

	token = strtok(data, ","); //take token
	while (token != NULL) //loop through
	{
		valueArray[i] = atol(token);
		i++;
		token = strtok(NULL, ",");	//next token
	}
	return valueArray;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	//Receive data from m7 IPC OpenAMP task
	//INIT OPEN AMP ENDPOINT SLAVE
	MAILBOX_Init();

	if (MX_OPENAMP_Init(RPMSG_REMOTE, NULL) != HAL_OK)
		Error_Handler();

	int32_t status = OPENAMP_create_endpoint(&rp_endpoint, RPMSG_SERVICE_NAME,
	RPMSG_ADDR_ANY, rpmsg_recv_callback,
	NULL);
	if (status < 0)
		Error_Handler();

	if (osSemaphoreAcquire(txSemaphoreHandle, osWaitForever) != osOK)
		Error_Handler(); //acquire semaphore so that Nothing is transmitted yet
	/* Infinite loop */
	for (;;) {
		OPENAMP_check_for_message();

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTxTask */
/**
 * @brief Function implementing the txTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startTxTask */
void startTxTask(void *argument)
{
  /* USER CODE BEGIN startTxTask */
	uint32_t bufferData = 0;

	/* Infinite loop */
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Block until notified by another task (read sensor task)
		TorqueEncoder *test = (TorqueEncoder*) pvPortMalloc(
				sizeof(struct TorqueEncoder));	//testing only
		test->id = TORQUE_SENSOR;
		test->valueFloat = 0;
		test->valueInt = 0;
		test->sensor1 = 0;
		test->sensorPercent0 = 0;
		test->sensorPercent1 = 0;
		test->travelPercent = 0;
		test->valueBool = false;

		if (osSemaphoreAcquire(readSensorSemaphoreHandle, osWaitForever)
				!= osOK)
			Error_Handler();	//acquire semaphore for buffer read

		if (bufferGet(&bufferData)) //get data from buffer if not empty
				{
			test->valueInt = bufferData;
			if (bufferGet(&bufferData))
				test->sensor1 = bufferData;

			if (osSemaphoreAcquire(txSemaphoreHandle, osWaitForever) != osOK)
				Error_Handler(); //SYNC with receiving data task from C7 core

			if (OPENAMP_send(&rp_endpoint, test, sizeof(struct TorqueEncoder))
					< 0)
				Error_Handler(); //Send data to m7 core

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		}
		if (osSemaphoreRelease(readSensorSemaphoreHandle) != osOK)
			Error_Handler();	//free semaphore, done with buffer read

		vPortFree(test);
		osDelay(1);

	}
  /* USER CODE END startTxTask */
}

/* USER CODE BEGIN Header_startReadSensor */
/**
 * @brief Function implementing the readSensors thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startReadSensor */
void startReadSensor(void *argument)
{
  /* USER CODE BEGIN startReadSensor */
	uint32_t i = 0;
	/* Infinite loop */
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//blocks until notified, Used for testing (simulation)

		uint32_t *valueArray = parseCSV((char*) rx_buffer);	//TESTING ONLY READ 2 SENSOR VALUES AND BUT INTO BUFFER
		for (i = 0; i < 2; i++) {
			if (osSemaphoreAcquire(readSensorSemaphoreHandle, osWaitForever)
					!= osOK)
				Error_Handler();	//acquire semaphore for buffer write
			bufferPut(valueArray[i]);	//read data and put into buffer
			if (osSemaphoreRelease(readSensorSemaphoreHandle) != osOK)
				Error_Handler();
			osDelay(10);
		}
		xTaskNotifyGive(txTaskHandle); // Notify txTask to start

		memset(rx_buffer, 0, sizeof(rx_buffer)); //free buffer for next data from uart
		vPortFree(valueArray);

		HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, sizeof(rx_buffer)); //get next data from uart
		osDelay(1);
	}
  /* USER CODE END startReadSensor */
}

/* USER CODE BEGIN Header_startSendCAN */
/**
 * @brief Function implementing the sendCANTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startSendCAN */
void startSendCAN(void *argument)
{
  /* USER CODE BEGIN startSendCAN */

	/* Infinite loop */
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		char *data = pvPortMalloc(200); //Malloc 100 char array

		int intPart = test->travelPercent;
		float fracPart = test->travelPercent - intPart;
		int decimals = 4;
		int fracToInt = trunc(fracPart * pow(10, decimals));
//	  sprintf(data, "%d.%04d\n", intPart,fracToInt);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//		uint32_t raw = HAL_ADC_GetValue(&hadc1);
//		sprintf(data, "ADC: %08" PRIx32, raw);
//		HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), 200);
		vPortFree(data);
		osDelay(1);
	}
  /* USER CODE END startSendCAN */
}

/* USER CODE BEGIN Header_startCanTest */
/**
 * @brief Function implementing the canTestTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCanTest */
void startCanTest(void *argument)
{
  /* USER CODE BEGIN startCanTest */
	/* Infinite loop */
	uint32_t Notifications = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, Notifications, 0) != HAL_OK) {
		Error_Handler();
	}

//	txHeader.Identifier = 0xC0; //id of transmitter, from filters
//	txHeader.IdType = FDCAN_STANDARD_ID;
//	txHeader.TxFrameType = FDCAN_DATA_FRAME;
//	txHeader.DataLength = FDCAN_DLC_BYTES_8; //only 8 byte of data send
//	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
//	txHeader.FDFormat = FDCAN_CLASSIC_CAN; //normal CAN, not FDCAN
//	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//	txHeader.MessageMarker = 0;

	FDCAN_ProtocolStatusTypeDef protocolStatus;

//	txData[0] = 0x11;
//	txData[1] = 0x22;
//	txData[2] = 0x33;
//	txData[3] = 0x44;
//	txData[4] = 0x55;
//	txData[5] = 0x66;
//	txData[6] = 10;
//	txData[7] = 10 >> 8;
//	uint32_t errorS = 0;

	for (;;) {
		char *data = pvPortMalloc(100); //Malloc 100 char array
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		uint32_t raw = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		uint32_t raw1 = HAL_ADC_GetValue(&hadc2);

		sprintf(data, "ADC1: %03lu\n ADC2: %03lu\n", raw, raw1);

		HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), 100);

		HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocolStatus);
		if (protocolStatus.BusOff) {
			CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
		}

		SendMotorCommand(&hfdcan1,10,0,1,0,2400);
		SendSensorReading(&hfdcan1, raw);



//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData)
//				!= HAL_OK) //send data to CAN bus
//				{
//			Error_Handler();
//		}
//		uint32_t counter = 500;
//        txHeader.Identifier = counter++;

		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//		char *data = pvPortMalloc(100);
//	  sprintf(data, "%lu\n", count);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);

//	  if(count > 0)
//	  {
//		  sprintf(data, "Received ID: 0x%X\n", rxHeader.Identifier);
//		  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//
//		  sprintf(data, "DLC: %d\n", rxHeader.DataLength);
//		  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//
//		  sprintf(data, "Data: ");
//		  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//
//		  for (uint8_t i = 0; i < 8; i++) {
//			  sprintf(data, "0x%X ", rxData[i]);
//			  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//		  }
//		  count--;
//	  }
//	  errorS ++;
//	  sprintf(data, "ERRORS %d\n", errorS);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);

//	  sprintf(data, "Last Error Code: %d\n", protocolStatus.LastErrorCode);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//	  sprintf(data, "Error Passive: %d\n", protocolStatus.ErrorPassive);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//	  sprintf(data, "Bus Off: %d\n", protocolStatus.BusOff);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
//	  sprintf(data, "Activity: %d\n", protocolStatus.Activity);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data, strlen(data), 100);
		vPortFree(data);

		osDelay(10);
	}
  /* USER CODE END startCanTest */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
	while (1) {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
