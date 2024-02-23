/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * ADC/DMA + ARM/DSP/FIR + CANxSPI + FREERTOS	:::::	SORRENTINO-ORTIZ ECHENIQUE
  * ELECTRÓNICA DIGITAL 3 						::::: 	UNSAM
  * PROYECTO
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
// #include "timers.h"
#include "queue.h"
#include "semphr.h"
// #include "event_groups.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "CANSPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Tension_Muestreada ((float) 3.3 * (float) Lectura_Datos[1] / (float) 4095)
#define V_25 (float) 0.76 //datasheet
#define Pendiente_Prom (float) 0.0025

#define Tension_Muestreada2 Lectura_Datos[0]
//#define Cant_Coefs_K 	2 //6, filtro IIR, prueba
//#define Cant_Coefs_V 	3 //7
#define Elementos 		5 //NUM_BLOCKS

#define BLOCK_SIZE 1
#define NUM_TAPS 301
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
union Temp_Byte{
		float32_t temperatura;
		uint8_t byte_temp[4];
	};

// Coeficientes del filtro FIR, notch para suprimir ruido a 50 Hz
const float32_t fir_coefficients[NUM_TAPS] = {0.0003,  0.0003,  0.0002,  0.0002,  0.0001,  0.0000, -0.0001, -0.0002, -0.0002, -0.0003,
		   -0.0003, -0.0003, -0.0003, -0.0002, -0.0001,  0.0000,  0.0001,  0.0002,  0.0003,  0.0004,
		    0.0004,  0.0004,  0.0004,  0.0003,  0.0002,  0.0000, -0.0002, -0.0003, -0.0005, -0.0006,
		   -0.0006, -0.0006, -0.0005, -0.0004, -0.0002, -0.0000,  0.0002,  0.0005,  0.0006,  0.0008,
		    0.0009,  0.0008,  0.0007,  0.0006,  0.0003, -0.0000, -0.0003, -0.0006, -0.0009, -0.0011,
		   -0.0012, -0.0011, -0.0010, -0.0007, -0.0004,  0.0000,  0.0004,  0.0008,  0.0012,  0.0014,
		    0.0015,  0.0015,  0.0013,  0.0010,  0.0005,  0.0000, -0.0005, -0.0010, -0.0015, -0.0018,
		   -0.0019, -0.0018, -0.0016, -0.0012, -0.0006,  0.0000,  0.0007,  0.0013,  0.0018,  0.0021,
		    0.0023,  0.0022,  0.0019,  0.0014,  0.0008,  0.0000, -0.0008, -0.0015, -0.0021, -0.0025,
		   -0.0027, -0.0026, -0.0022, -0.0016, -0.0009, -0.0000,  0.0009,  0.0017,  0.0024,  0.0028,
		    0.0030,  0.0029,  0.0025,  0.0018,  0.0010,  0.0000, -0.0010, -0.0019, -0.0027, -0.0032,
		   -0.0034, -0.0032, -0.0028, -0.0020, -0.0011,  0.0000,  0.0011,  0.0021,  0.0029,  0.0034,
		    0.0036,  0.0035,  0.0030,  0.0022,  0.0011, -0.0000, -0.0012, -0.0022, -0.0031, -0.0036,
		   -0.0038, -0.0037, -0.0031, -0.0023, -0.0012,  0.0000,  0.0012,  0.0023,  0.0032,  0.0038,
		    0.0040,  0.0038,  0.0032,  0.0023,  0.0012,  0.0000, -0.0012, -0.0023, -0.0032, -0.0038,
		    0.9957, -0.0038, -0.0032, -0.0023, -0.0012,  0.0000,  0.0012,  0.0023,  0.0032,  0.0038,
		    0.0040,  0.0038,  0.0032,  0.0023,  0.0012,  0.0000, -0.0012, -0.0023, -0.0031, -0.0037,
		   -0.0038, -0.0036, -0.0031, -0.0022, -0.0012, -0.0000,  0.0011,  0.0022,  0.0030,  0.0035,
		    0.0036,  0.0034,  0.0029,  0.0021,  0.0011,  0.0000, -0.0011, -0.0020, -0.0028, -0.0032,
		   -0.0034, -0.0032, -0.0027, -0.0019, -0.0010,  0.0000,  0.0010,  0.0018,  0.0025,  0.0029,
		    0.0030,  0.0028,  0.0024,  0.0017,  0.0009, -0.0000, -0.0009, -0.0016, -0.0022, -0.0026,
		   -0.0027, -0.0025, -0.0021, -0.0015, -0.0008,  0.0000,  0.0008,  0.0014,  0.0019,  0.0022,
		    0.0023,  0.0021,  0.0018,  0.0013,  0.0007,  0.0000, -0.0006, -0.0012, -0.0016, -0.0018,
		   -0.0019, -0.0018, -0.0015, -0.0010, -0.0005,  0.0000,  0.0005,  0.0010,  0.0013,  0.0015,
		    0.0015,  0.0014,  0.0012,  0.0008,  0.0004,  0.0000, -0.0004, -0.0007, -0.0010, -0.0011,
		   -0.0012, -0.0011, -0.0009, -0.0006, -0.0003, -0.0000,  0.0003,  0.0006,  0.0007,  0.0008,
		    0.0009,  0.0008,  0.0006,  0.0005,  0.0002, -0.0000, -0.0002, -0.0004, -0.0005, -0.0006,
		   -0.0006, -0.0006, -0.0005, -0.0003, -0.0002,  0.0000,  0.0002,  0.0003,  0.0004,  0.0004,
		    0.0004,  0.0004,  0.0003,  0.0002,  0.0001,  0.0000, -0.0001, -0.0002, -0.0003, -0.0003,
		   -0.0003, -0.0003, -0.0002, -0.0002, -0.0001,  0.0000,  0.0001,  0.0002,  0.0002,  0.0003,
		    0.0003};

arm_fir_instance_f32 fir_instance;
float32_t fir_in_arm[Elementos*BLOCK_SIZE], fir_out_arm[Elementos*BLOCK_SIZE], fir_state[Elementos+BLOCK_SIZE-1];

float Temp2;
float Temperatura_50Hz;
int flag0 = 0;
uint32_t Lectura_Datos[2];

SemaphoreHandle_t Semaforo1;
SemaphoreHandle_t Semaforo2;
SemaphoreHandle_t Semaforo3;

QueueHandle_t Cola_Datos; // 5 datos salen de Toma a Proceso
QueueHandle_t Dato_Procesado; // Única salida filtrada
QueueHandle_t Alerta; // Blue Button

TaskHandle_t xHandle_Datos = NULL; // Manejo dinámico de prioridades

uCAN_MSG txMessage; // Envío de mensaje a la red CAN
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */


void Toma_Datos(void const * argument); 	//TAREAS
void Procesa_Datos(void const * argument);
void Salida_datos(void const * argument);
void Sobre_Datos(void const * argument);

float UNSAM_ADC (void);		//FUNCIONES uint32_t UNSAM_ADC (void)
void UNSAM_CAN (float32_t);	//void UNSAM_CAN (float)
//float UNSAM_iir_Lineal (float);
void get_time(void); // BB


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	static unsigned int a = 0;
	a++;
}
*/
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	V_Muest1 = Tension_Muestreada;
	Temp1 = ((V_Muest1 - V_25) / Pendiente_Prom) +15;

	V_Muest2 = Tension_Muestreada2;
	Temp2 = V_Muest2 * 100;

	fir_in_arm = (float32_t) Temperatura;
	arm_fir_f32(&fir_instance, &fir_in_arm, &fir_out_arm, 1);
	Temperatura_50Hz = fir_out_arm;
	*/
/*
	iir_in_arm = (float32_t) Temp2;
	arm_iir_lattice_f32(&iir_instance, &iir_in_arm, &iir_out_arm, 1);
	Temperatura_50Hz = iir_out_arm/10;
}
*/

// Para usar printf
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
	return ch;
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
  arm_fir_init_f32(&fir_instance, Elementos, (float32_t *)fir_coefficients, fir_state, BLOCK_SIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, Lectura_Datos, 2);
  CANSPI_Initialize();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  Semaforo1 = xSemaphoreCreateBinary();
  if(Semaforo1 == 0)
  {
	  printf("Semaforo1 NO creado\n\r");
  }
  else
  {
  	  printf("Semaforo1 CREADO\n\r");
  }
  Semaforo2 = xSemaphoreCreateBinary();
  if(Semaforo2 == 0)
  {
	  printf("Semaforo2 NO creado\n\r");
  }
  else
  {
	  printf("Semaforo2 CREADO\n\r");
  }
  Semaforo3 = xSemaphoreCreateBinary();
  if(Semaforo3 == 0)
  {
	  printf("Semaforo3 NO creado\n\r");
  }
  else
  {
	  printf("Semaforo3 CREADO\n\r");
  }
  xSemaphoreGive(Semaforo1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  Cola_Datos = xQueueCreate(5, sizeof(float)); // cantidad y tipo/tamaño Cola_Datos
  if(Cola_Datos == 0)
  {
  	printf("Cola Cola_Datos NO creada\n\r");
  }
  else
  {
  	printf("Cola Cola_Datos CREADA\n\r");
  }
  Dato_Procesado = xQueueCreate(1, sizeof(float)); //Dato_Procesado
  if(Dato_Procesado == 0)
  {
	  printf("Cola Dato_Procesado NO creada\n\r");
  }
  else
  {
	  printf("Cola Dato_Procesado CREADA\n\r");
  }
  Alerta = xQueueCreate(1, sizeof(float)); // cantidad y tipo/tamaño Cola_Datos
  if(Alerta == 0)
  {
  	  printf("Cola Alerta NO creada\n\r");
  }
  else
  {
  	  printf("Cola Alerta CREADA\n\r");
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate((void *)Toma_Datos, "tomaDato", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
  xTaskCreate((void *)Procesa_Datos, "procDato", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate((void *)Salida_datos, "saliDato", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate((void *)Sobre_Datos, "sobreDato", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xHandle_Datos);
  vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 3000;
  AnalogWDGConfig.LowThreshold = 200;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_OCTOBER;
  sDate.Date = 0x26;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Toma_Datos(void const * argument)
{
	float Entrada[Elementos];
	while(1)
	{
		xSemaphoreTake(Semaforo1, portMAX_DELAY);
		for(uint32_t i = 0; i<Elementos; i++)
		{
			Entrada[i] = UNSAM_ADC();
			Entrada[i] = Entrada[i] * 7.025/100;	// Del ADC a temperatura
			xQueueSend(Cola_Datos, &Entrada[i], portMAX_DELAY); //Cola de 5 espacios enviados a Procesar
		}
		xSemaphoreGive(Semaforo2);
	}
}

float UNSAM_ADC (void)
{
	float V_Muest1;
	float Temp1;
	float V_Muest2;
	V_Muest1 = Tension_Muestreada;
	Temp1 = ((V_Muest1 - V_25) / Pendiente_Prom) +25;
	V_Muest2 = Tension_Muestreada2;
	return(V_Muest2);
}

void Procesa_Datos(void const * argument)
{
	float datos[Elementos];
	float32_t salida[Elementos];
	float sal_Temp_50Hz[1];

	int uxNumberOfFreeSpaces = 0;
	uint32_t  k = 0;
	float aux, suma = 0, prom;

	while(1)
	{
		if(xSemaphoreTake(Semaforo2, portMAX_DELAY)==pdTRUE)
		{
			for(uint32_t i = 0; i<Elementos; i++) // Recepción de cola de Toma
			{
				xQueueReceive(Cola_Datos, &datos[i], portMAX_DELAY);
				uxNumberOfFreeSpaces = uxQueueSpacesAvailable(Cola_Datos);
				Temp2 = datos[i];	// Visor SW
				fir_in_arm[i] = Temp2; // Datos que entran al filtro
			}
			for (uint32_t n = 0; n < uxNumberOfFreeSpaces; n++) // Filtro Notch que suprime ruido a 50 Hz
			{
				arm_fir_f32(&fir_instance, fir_in_arm + (n * BLOCK_SIZE), fir_out_arm + (n * BLOCK_SIZE), BLOCK_SIZE);
				salida[n] = datos[n] + fir_out_arm[n];
			}
			xSemaphoreGive(Semaforo3);
			if(uxNumberOfFreeSpaces > Elementos-1)
			{
				for(k = 0; k<Elementos; k++)
				{
					for(uint32_t j = 0; j<Elementos-1-k; j++)
					{
						if (salida[j]>salida[j+1])
						{
							aux=salida[j];
							salida[j]=salida[j+1];
							salida[j+1]=aux;
						}
					}
				}
				k=0;
				for(k = 0; k<Elementos-2; k++)
				{
					suma = suma + salida[k+1];
				}
				prom = suma/(Elementos-2); suma = 0;
				sal_Temp_50Hz[0] = prom;
				Temperatura_50Hz = sal_Temp_50Hz[0]; // Visor SW

				xQueueSend(Dato_Procesado, sal_Temp_50Hz, portMAX_DELAY);
			}
		}
	}
}

void Salida_datos(void const * argument)
{
	float datos_in[1];
	float Salida_Final[1];
	const TickType_t xBlockTime = pdMS_TO_TICKS( 1000 );
	UBaseType_t uxPriority_Salida;
	uxPriority_Salida = uxTaskPriorityGet( NULL );
	while(1)
	{
		if(xSemaphoreTake(Semaforo3,portMAX_DELAY)==pdTRUE)
		{
			xQueueReceive(Dato_Procesado, datos_in, portMAX_DELAY);
			Salida_Final[0] = datos_in[0];
			if(Salida_Final[0]<40)
			{
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				vTaskPrioritySet( xHandle_Datos, ( uxPriority_Salida + 1));
			}
			else
			{
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
				UNSAM_CAN(Salida_Final[0]);
			}
			vTaskDelay(xBlockTime);
			xSemaphoreGive(Semaforo1);
		}
	}
}

void UNSAM_CAN (float32_t Salida)
{
	union Temp_Byte Valor;
	int flagMensajeR = 0;
	int uxNumberOfFreeSpaces;
	uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Alerta );
	if(uxNumberOfFreeSpaces < 1)
		xQueueReceive(Alerta, &flagMensajeR, portMAX_DELAY);
	Valor.temperatura = Salida;
	if(flagMensajeR > 0)
	{
		flag0 = 100+flagMensajeR; // Control en Live Expressions

		txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		txMessage.frame.id = 0x01;
		txMessage.frame.dlc = 8;
		txMessage.frame.data0 = Valor.byte_temp[0];
		txMessage.frame.data1 = Valor.byte_temp[1];
		txMessage.frame.data2 = Valor.byte_temp[2];
		txMessage.frame.data3 = Valor.byte_temp[3];
	    txMessage.frame.data4 = 5;
	    txMessage.frame.data5 = 5;
	    txMessage.frame.data6 = 5;
	    txMessage.frame.data7 = Salida;
	    CANSPI_Transmit(&txMessage);
	}
	else{
		flag0 = 200+flagMensajeR; // Control en Live Expressions

		txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		txMessage.frame.id = 0x01;
		txMessage.frame.dlc = 8;
		txMessage.frame.data0 = Valor.byte_temp[0];
		txMessage.frame.data1 = Valor.byte_temp[1];
		txMessage.frame.data2 = Valor.byte_temp[2];
		txMessage.frame.data3 = Valor.byte_temp[3];
	    txMessage.frame.data4 = 0;
	    txMessage.frame.data5 = 0;
	    txMessage.frame.data6 = 0;
	    txMessage.frame.data7 = Salida;
	    CANSPI_Transmit(&txMessage);
	}
}

void Sobre_Datos( void const * argument)
{
	UBaseType_t uxPriority_Sobre;
	int flagMensajeR = 0;
	int uxNumberOfFreeSpaces;
	const TickType_t xBlockTime = pdMS_TO_TICKS( 10 );

	for( ;; )
	{
		uxPriority_Sobre = uxTaskPriorityGet( NULL );
		if(uxPriority_Sobre >1)
		{
			flag0 = 444;
			uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Alerta );
			if(uxNumberOfFreeSpaces < 1)
				xQueueReceive(Alerta, &flagMensajeR, portMAX_DELAY);

			txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
			txMessage.frame.id = 0xAA;
			txMessage.frame.dlc = 8;
			txMessage.frame.data0 = 0;
			txMessage.frame.data1 = 0;
			txMessage.frame.data2 = 0;
			txMessage.frame.data3 = 0;
		    txMessage.frame.data4 = 0x01;
		    txMessage.frame.data5 = 0x01;
		    txMessage.frame.data6 = 0x01;
		    txMessage.frame.data7 = 0x01;
		    CANSPI_Transmit(&txMessage);

			vTaskPrioritySet( xHandle_Datos, (uxPriority_Sobre - 2));
		}
		else
			vTaskDelay(xBlockTime);
	}
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	int flagMensajeT = 10;
	BaseType_t xHigherPriorityTaskWoken;
	get_time();
	flag0 = 12345;
	xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToBackFromISR( Alerta, &flagMensajeT, &xHigherPriorityTaskWoken );
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}
void get_time(void)
{
	//RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	//HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	/* Display time Format: hh:mm:ss */
	printf("Se presionó el botón a los = %02d:%02d:%02d \n\r", gTime.Hours, gTime.Minutes, gTime.Seconds);
	/* Display date Format: dd-mm-yy */
	//printf("Fecha = %02d-%02d-%02d \n\r", gDate.Date, gDate.Month, 2000 + gDate.Year);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void const * argument)
//{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  //for(;;)
  //{
    //osDelay(1);
  //}
  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
