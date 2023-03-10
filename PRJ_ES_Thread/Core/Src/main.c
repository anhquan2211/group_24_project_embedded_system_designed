/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "i2c-lcd.h"
	#include "dht.h"
	#include "delay_timer.h"
	#include <stdio.h>
	#include "string.h"
	//#define DEBUG0
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 900 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for BinarySem_TX_UART */
osSemaphoreId_t BinarySem_TX_UARTHandle;
const osSemaphoreAttr_t BinarySem_TX_UART_attributes = {
  .name = "BinarySem_TX_UART"
};
/* Definitions for BinarySem_IT_UART */
osSemaphoreId_t BinarySem_IT_UARTHandle;
const osSemaphoreAttr_t BinarySem_IT_UART_attributes = {
  .name = "BinarySem_IT_UART"
};
/* Definitions for BinarySem_IT_BUTTON */
osSemaphoreId_t BinarySem_IT_BUTTONHandle;
const osSemaphoreAttr_t BinarySem_IT_BUTTON_attributes = {
  .name = "BinarySem_IT_BUTTON"
};
/* Definitions for CountingSem_Mode2 */
osSemaphoreId_t CountingSem_Mode2Handle;
const osSemaphoreAttr_t CountingSem_Mode2_attributes = {
  .name = "CountingSem_Mode2"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);

/* USER CODE BEGIN PFP */
typedef struct
{
	__IO uint16_t IO_id;
	__IO uint16_t IO_data;
} myQueueData_StructType;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	//::::::::::::::::::::::::::::::::::::::::::: Khai bao lien quan toi UART-Receive
	#define MODE1 "M1:"
	#define MODE2 "M2:"
	#define THRESHOLD "TH:"
	#define key_DHT11_TEMP "TEMP_"
	#define key_DHT11_HUM "HUM_"
	#define key_MQ2_GAS "GAS_"
	uint8_t DATA_RX_1char;	//1 ki tu nhan ve
	uint8_t flag_isr_uart = 0;
	uint8_t index_uart = 0;
	char DATA_RX[50];		//Chuoi ki tu nhan ve

	//::::::::::::::::::::::::::::::::::::::::::: Khai bao cac tham so UART-Transmit
	char DATA_TX[50];		 	 //Chuoi ki tu gui di
	uint16_t T_display_m1 = 3000;    //MODE1: chu ky hien thi ca 3 gia tri len may tinh  [ms]
	uint8_t MODE_DISPLAY = 0;   //MODE_DISPLAY = 0 (MODE1) ... MODE_DISPLAY = 1 (MODE2) [not used]
	//int T_display_m2_DHT11 = 0; //MODE2: chu ky hien thi gia tri nhiet do & do am   [ms]
	//int T_display_m2_MQ2 = 0;   //MODE2: chu ky hien thi gia tri khi gas	[ms]
	
	//::::::::::::::::::::::::::::::::::::::::::: Debug
	unsigned long CheckTime_start = 0;
	
	//::::::::::::::::::::::::::::::::::::::::::::: Khai bao bien luu tru gia tri
	float TEMP = 0;
	uint16_t HUM  = 0, GAS  = 0;
	uint16_t HUM_threshold  = 50, TEMP_threshold = 20, GAS_threshold = 1000;
	char notice[20];
	
	//-------------------------------------------------------------------------------
	//::::::::::::::::::::::::::::::::::::::::::::: Khai bao bien thu vien LCD va DHT11 
	DHT_HandleTypeDef DHT11;
	char ndo[30];
	char doamgas[30];
	
	//::: TASK -- Truyen du lieu [ko can quan tam]
	void SendData()
	{	
		//unsigned long myTick = HAL_GetTick();
		//int SIZE_DATA_TX = sprintf(DATA_TX, "\n[%ld]\n \t TEMP: %.2f*C || HUMD: %d % || GAS:%d %", myTick, 25.02, 85, 10);
		//HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
		//HAL_Delay(500);
		//Thoi gian thuc thi C = 505ms ... Neu khong xet delay(500) thi Ccommand = 5ms
	}


//::: ISR UART Callback : /* Thoi gian xu ly het < 0ms*/

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) 
	{
		//Nhan ki tu xuong dong thi bat co de xu ly chuoi vua duoc nhan
		if(DATA_RX_1char == '\n')
		{
			DATA_RX[index_uart] = '\0';
			//flag_isr_uart = 1;
			osSemaphoreRelease(BinarySem_IT_UARTHandle); //Semaphore: nhả semaphore thực thi ngắt - Task7
		}
		//Chua nhan het mot chuoi
		else
		{
			DATA_RX[index_uart] = DATA_RX_1char;
			index_uart++;
		}
		HAL_UART_Receive_IT(&huart1, &DATA_RX_1char, 1);
	}
	void GetValue(char *pCmd, uint16_t *get_data)
	{
		int T = 0;
		uint8_t flag = 0;
		while(1)
		{
			if(*pCmd >= '0' && *pCmd <= '9') //Chuyen gia tri lay tu may tinh ve kieu so nguyen
			{
				T *= 10;
				T += (*pCmd - '0');
				flag = 1;
			}
			else 
				if(*pCmd == ' ');
			else 
				if(*pCmd == ';') //Ket thuc chuoi
			{
				if(flag == 1)
				{
					*get_data = T;
					flag = 0;
					T = 0;							
				}
				break;	
			}
			else{
				// Neu chua cac ki tu khac la loi, thoat khoi vong lap While(1)
				// ERROR! 
				break;
			}	
			pCmd++; //Tang con tro de kiem tra ki tu tiep theo 			
			}
	}

//::: [TASK] -- Xu ly chuoi nhan ve /*	Thoi gian xu ly het C = 6ms */
	/*
	void ISR_UART_HandleRxData()
	{
		if(flag_isr_uart == 1)  //Ktra flag_isr_uart = 1 thi moi thuc thi task nay
		{
			#ifdef DEBUG0 
				CheckTime_start = HAL_GetTick(); 
			#endif
			char *pCmd1 = strstr(DATA_RX, MODE1); //Kiem tra chuoi nhan ve co chua dau hieu Mode 1 khong
			char *pCmd2 = strstr(DATA_RX, MODE2); //Kiem tra chuoi nhan ve co chua dau hieu Mode 2 khong
			//------------------------------------ TH1: Chuoi nhan ve la MODE1
			if(pCmd1 != NULL)
			{
				pCmd1 += strlen(MODE1); //Tro toi chu so dau tien cua "Chu ky truyen du lieu len may tinh"
				GetValue(pCmd1, &T_display_m1);
				#ifdef DEBUG0 
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: T_display_m1: %dms\n", T_display_m1); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500); 
				#endif
			}
			
			//------------------------------------ TH2: Chuoi nhan ve la MODE2
			if(pCmd2 != NULL)
			{
				pCmd2 += strlen(MODE2); //Tro toi ki tu sau dau ':'
				if(strstr(DATA_RX, key_DHT11) != NULL)
				{
					pCmd2 += strlen(key_DHT11); //Tro toi chu so dau tien cua chu ky truyen du lieu DHT11
					GetValue(pCmd2, &T_display_m2_DHT11);
					#ifdef DEBUG0
						int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: T_display_m2_DHT11: %dms\n", T_display_m2_DHT11); \
						HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
					#endif
				}
				if( strstr(DATA_RX, key_MQ2) != NULL)
				{
					pCmd2 += strlen(key_MQ2); // Tro toi chu so dau tien cua chu ky truyen du lieu MQ2	
					GetValue(pCmd2, &T_display_m2_MQ2);
					#ifdef DEBUG0  
						int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: T_display_m2_MQ2: %dms\n", T_display_m2_MQ2); \
						HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
					#endif	
				}		
			}

			//------------------------------------ Ket thuc xu ly ISR
			index_uart = 0;
			flag_isr_uart = 0;
			#ifdef DEBUG0 
				int SIZE_DATA_TX = sprintf(DATA_TX, "\n ISR_UART_HandleRxData() begin:[%ld] & end: [%d] => Total time = %d\n"\
										  ,CheckTime_start,HAL_GetTick(), HAL_GetTick() - (int)CheckTime_start); \
				HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500); 
			#endif
		}
	}
	*/

//::::
void Task1 (void)
{
//	CheckTime_start = HAL_GetTick();
//	int SIZE_DATA_TX = sprintf(DATA_TX, "\nbegin:[%ld]", CheckTime_start);
//	HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 300);
	DHT_ReadTempHum(&DHT11);
	TEMP = DHT11.Temp;
	HUM = DHT11.Humi;
//	SIZE_DATA_TX = sprintf(DATA_TX, "\ntotal:[%ld]\n", HAL_GetTick()- CheckTime_start);
//	HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 300);
}

void Task2(void)
{ 
//	CheckTime_start = HAL_GetTick();
//	int SIZE_DATA_TX = sprintf(DATA_TX, "\nbegin:[%ld]", CheckTime_start);
//	HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 300);
	
	HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,1000);
  GAS = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
	
//	SIZE_DATA_TX = sprintf(DATA_TX, "\ntotal:[%ld]\n", HAL_GetTick()- CheckTime_start);
//	HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 300);
}

void Task3(void)
{
	sprintf(ndo, "Tem:%3.1f*C", TEMP);
	sprintf(doamgas, "Hum:%d Gas:%d ", HUM, GAS);
  lcd_put_cur(0, 0);
	lcd_send_string(ndo);
	lcd_put_cur(1, 0);
	lcd_send_string(doamgas);
}
//----------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static unsigned long tick_button = 0; 
  if(HAL_GetTick() - tick_button >= 1000)
  {
	tick_button = HAL_GetTick();
	osSemaphoreRelease(BinarySem_IT_BUTTONHandle);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &DATA_RX_1char, 1);
	lcd_init();
	DHT_Init(&DHT11, &htim2, GPIOC, GPIO_PIN_15);
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem_TX_UART */
  BinarySem_TX_UARTHandle = osSemaphoreNew(1, 1, &BinarySem_TX_UART_attributes);

  /* creation of BinarySem_IT_UART */
  BinarySem_IT_UARTHandle = osSemaphoreNew(1, 0, &BinarySem_IT_UART_attributes);

  /* creation of BinarySem_IT_BUTTON */
  BinarySem_IT_BUTTONHandle = osSemaphoreNew(1, 0, &BinarySem_IT_BUTTON_attributes);

  /* creation of CountingSem_Mode2 */
  CountingSem_Mode2Handle = osSemaphoreNew(5, 0, &CountingSem_Mode2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(myQueueData_StructType), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myTask01 */
  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* creation of myTask08 */
  myTask08Handle = osThreadNew(StartTask08, NULL, &myTask08_attributes);

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
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//ISR_UART_HandleRxData();
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function: LAY DU LIEU NHIET DO
  * @WCET (Wrost-case execution time): 22 ms 
  * @priority		 : 25
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN 5 */
	myQueueData_StructType msg;	
  /* Infinite loop */
  for(;;)
  {
		Task1();
		osDelay(5000);
  }
  /*
  	int SIZE_DATA_TX = sprintf(DATA_TX, "\n%s Doc nhiet do, begin [%lu] \n", myTask01_attributes.name, osKernelGetTickCount());
	HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
	*/
	//TEMP += 2;
	//osMessageQueueGet(myQueue01Handle, &msg, 0, osWaitForever);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function: LAY DU LIEU DO AM
* @execution time: ? ms
* @priority		 : ?
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		Task2();
		osDelay(6000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function: HIEN THI LCD 16x2 
* @execution time: 14 ms
* @priority: 
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	Task3();
	osDelay(3000);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function: HIEN THI NHIET DO
* @execution time: 4 ms 
* @priority		 : 33
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {	
		CheckTime_start = HAL_GetTick();
		if(TEMP > TEMP_threshold) strcpy(notice, "Above threshold");
		else strcpy(notice, "Below threshold");
		int SIZE_DATA_TX = sprintf(DATA_TX, "\n[%ld] \t TEMP: %4.1f*C___Notification: %s ", CheckTime_start, TEMP, notice);
  		if(MODE_DISPLAY == 0)
		{
			osSemaphoreAcquire(BinarySem_TX_UARTHandle, osWaitForever); //Semaphore: Chiem lay semaphore cho phep truyen UART
			HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);		
			osSemaphoreRelease(BinarySem_TX_UARTHandle);
			//osDelay(T_display_m1);
		}
		else
		{
			osSemaphoreAcquire(CountingSem_Mode2Handle, osWaitForever);
			HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
		}
	osDelay(T_display_m1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function: HIEN THI DO AM
* @execution time: 4  ms
* @priority		 : 32
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {		
	CheckTime_start = HAL_GetTick();
	if(HUM > HUM_threshold)strcpy(notice, "Above threshold");
	else strcpy(notice, "Below threshold");
	int SIZE_DATA_TX = sprintf(DATA_TX, "\n[%ld] \t HUM : %4d%% ___Notification: %s\n", CheckTime_start, HUM, notice );
	if(MODE_DISPLAY == 0)
	{
		osSemaphoreAcquire(BinarySem_TX_UARTHandle, osWaitForever); //Semaphore: Ch�? lấy được semaphore cho phép truy�?n UART				 
		HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);		
		osSemaphoreRelease(BinarySem_TX_UARTHandle);
		//osDelay(T_display_m1);
	}
	else
	{		
		osSemaphoreAcquire(CountingSem_Mode2Handle, osWaitForever);
		HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
	}
	osDelay(T_display_m1);	
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function: HIEN THI NONG DO KHI GAS
* @execution time: 4 ms
* @priority		 : 34
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
			CheckTime_start = HAL_GetTick();
			if(GAS > GAS_threshold)strcpy(notice, "Above threshold");
			else strcpy(notice, "Below threshold");
			int SIZE_DATA_TX = sprintf(DATA_TX, "\n[%ld] \t GAS : %4d  ___Notification: %s ", CheckTime_start, GAS, notice);
		if(MODE_DISPLAY == 0)
		{
			osSemaphoreAcquire(BinarySem_TX_UARTHandle, osWaitForever); //Semaphore: Ch�? lấy được semaphore cho phép truy�?n UART			 
			HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);		
			osSemaphoreRelease(BinarySem_TX_UARTHandle);
			//osDelay(T_display_m1);
		}
		else
		{
			osSemaphoreAcquire(CountingSem_Mode2Handle, osWaitForever);
			HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
		}
		
	osDelay(T_display_m1);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
  		osSemaphoreAcquire(BinarySem_IT_UARTHandle, osWaitForever);
		#ifdef DEBUG0 
			CheckTime_start = HAL_GetTick(); 
		#endif
		char *pCmd1 = strstr(DATA_RX, MODE1); //Kiem tra chuoi nhan ve co chua dau hieu Mode 1 khong
		char *pCmd2 = strstr(DATA_RX, MODE2); //Kiem tra chuoi nhan ve co chua dau hieu Mode 2 khong
		char *pCmd3 = strstr(DATA_RX, THRESHOLD); //Kiem tra chuoi nhan ve co dau hieu THRESHOLD khong
		//------------------------------------ TH1: Chuoi nhan ve la MODE1
		if(pCmd1 != NULL)
		{
			MODE_DISPLAY = 0;
			osSemaphoreRelease(BinarySem_TX_UARTHandle);
			osSemaphoreRelease(CountingSem_Mode2Handle); //Neu ko nha Semaphore co the dan toi truong hop task vua thuc hien mode 2 se treo vi dang cho semaphore counting
			osThreadResume(myTask06Handle);
			osThreadResume(myTask04Handle);
			osThreadResume(myTask05Handle);
			pCmd1 += strlen(MODE1); //Tro toi chu so dau tien cua "Chu ky truyen du lieu len may tinh"
			GetValue(pCmd1, &T_display_m1);
			int SIZE_DATA_TX = sprintf(DATA_TX, "\n"); \
			HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
			#ifdef DEBUG0 
				int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: T_display_m1: %dms\n", T_display_m1); \
				HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500); 
			#endif
		}
		
		//------------------------------------ TH2: Chuoi nhan ve la MODE2
		if(pCmd2 != NULL)
		{
			MODE_DISPLAY = 1;
			/*MODE2: Che do hien thi TEMP*/
			if( (pCmd2 = strstr(DATA_RX, key_DHT11_TEMP)) != NULL)
			{
				//pCmd2 += strlen(key_DHT11_TEMP); //Tro toi chu so dau tien cua chu ky truyen du lieu DHT11
				//GetValue(pCmd2, &T_display_m2_DHT11);
				osThreadResume(myTask04Handle);
				osThreadSuspend(myTask05Handle);
				osThreadSuspend(myTask06Handle);
				osSemaphoreRelease(CountingSem_Mode2Handle);				
				#ifdef DEBUG0
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: DHT11 (TEMP)\n"); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
				#endif
			}
			//--------------------------------------------------------------------------
			/*MODE2: Che do hien thi HUM*/
			if( (pCmd2 = strstr(DATA_RX, key_DHT11_HUM)) != NULL)
			{
				
				osThreadResume(myTask05Handle);
				osThreadSuspend(myTask04Handle);
				osThreadSuspend(myTask06Handle);
				osSemaphoreRelease(CountingSem_Mode2Handle);
				#ifdef DEBUG0
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: DHT11 (HUM)\n"); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
				#endif
			}
			//--------------------------------------------------------------------------
			/*MODE2: Che do hien thi GAS*/
			if( (pCmd2 = strstr(DATA_RX, key_MQ2_GAS)) != NULL)
			{
				
				osThreadResume(myTask06Handle);
				osThreadSuspend(myTask05Handle);
				osThreadSuspend(myTask04Handle);
				osSemaphoreRelease(CountingSem_Mode2Handle);
				#ifdef DEBUG0
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: MQ2 (GAS)\n"); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
				#endif
			}
		}
		//-------------------------------- TH3: Chuoi nhan ve de cap nhat nguong
		if(pCmd3 != NULL)
		{
			if( (pCmd3 = strstr(DATA_RX, key_DHT11_TEMP)) != NULL)
			{
				pCmd3 += strlen(key_DHT11_TEMP); //Tro toi chu so dau tien trong chuoi du lieu
				GetValue(pCmd3, &TEMP_threshold);
				#ifdef DEBUG0
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: DHT11 (TEMP) = %d\n",TEMP_threshold); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
				#endif
			}
			if( (pCmd3 = strstr(DATA_RX, key_DHT11_HUM)) != NULL)
			{
				pCmd3 += strlen(key_DHT11_HUM); //Tro toi chu so dau tien trong chuoi du lieu
				GetValue(pCmd3, &HUM_threshold);
				#ifdef DEBUG0
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: DHT11 (HUM) = %d\n",HUM_threshold); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
				#endif
			}
			if( (pCmd3 = strstr(DATA_RX, key_MQ2_GAS)) != NULL)
			{
				pCmd3 += strlen(key_MQ2_GAS); //Tro toi chu so dau tien trong chuoi du lieu
				GetValue(pCmd3, &GAS_threshold);
				#ifdef DEBUG0
					int SIZE_DATA_TX = sprintf(DATA_TX, "\n Update: MQ2 (GAS) = %d\n",GAS_threshold); \
					HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500);
				#endif
			}
		}
		
		
		
		//------------------------------------ Ket thuc xu ly ISR
		index_uart = 0;
		//flag_isr_uart = 0;
		#ifdef DEBUG0 
			int SIZE_DATA_TX = sprintf(DATA_TX, "\n ISR_UART_HandleRxData(), end: %ld => Total time = d\n"\
										,HAL_GetTick()-CheckTime_start); \
			HAL_UART_Transmit(&huart1, (uint8_t*)DATA_TX, SIZE_DATA_TX, 500); 				
		#endif
	//osDelay(1);
  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
  for(;;)
  {
  	osSemaphoreAcquire(BinarySem_IT_BUTTONHandle, osWaitForever);
	if(T_display_m1 >= 5000)
		T_display_m1 = 1000;
	else
		T_display_m1 += 1000;	
	/*while(1)
  	{
  		uint8_t sta = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		if(sta != btn_filter)
		{
			btn_filter = sta;
			is_debouncing = 1;
			time_deboune = HAL_GetTick();
		}
		if( (is_debouncing == 1) && (HAL_GetTick()-time_deboune >= 15)) //Nguoi ta quy uoc: neu trong khoang thoi gian 15ms btn_filter khong thay doi tuc la button da o trang thai on dinh
		{
			btn_current = btn_filter;  	//--btn_filter da on dinh, co the voi gan btn_current
			is_debouncing = 0;					//--Het trang thai nhieu
		}
		
		if(btn_current != btn_last)
		{
			if(btn_current == 0) 			//--Press button
			{
				//is_press_timeout = 1;
				//time_startpress_btn = HAL_GetTick();
				//btn_press_callback(); 	//--Ham xu ly nut nhan
				if(T_display_m1 >= 5000) T_display_m1 = 1000;
				else T_display_m1 += 1000;
			}
			btn_last = btn_current; //Ket thuc viec thuc hien chung nang button
			flag_button = 0;
		}
		
	}*/	
	osDelay(1000);
	}
	
  /* USER CODE END StartTask08 */
}

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
