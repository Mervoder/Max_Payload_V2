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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwgps/lwgps.h"
#include "LSM6DSLTR.h"
#include "./BME280/bme280.h"
#include "w25q.h"
#include <math.h>
#include "FIR_FILTER.h"
#include "string.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_TX_BUFFER_SIZE 70
#define RX_BUFFER_SIZE 128
#define DEVICE_ID 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index=0;
uint8_t rx_data=0;


float adc_pil_val=0;
float adc =0;
uint8_t counter=0;
uint8_t adc_flag=0;

uint8_t v4_battery=0;
uint8_t v4_mod=0;


uint8_t BUTTON_STATE=0;


uint8_t writeData[50] = {0,1,1,1};
uint8_t readData[50] = {0};

uint8_t loratx[LORA_TX_BUFFER_SIZE];
uint8_t lora_flag=0;
uint8_t sensor_flag=0;
uint8_t buzzer_flag=0;

float temperature=0;
float humidity=0;
float altitude=0;
float offset_altitude=0;
float pressure=0;
float alt=0;
float P0 = 1013.25;
float prev_alt=0;
float speed=0;
float altitude_max , speed_max, x_max;

float real_pitch, real_roll , toplam_pitch,toplam_roll;
float altitude_kalman;
uint8_t  sensor_counter =0, altitude_rampa_control=0;
uint16_t stage_sayac =0;
KalmanFilter kf;


typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;

enum ZORLU2024
{
	RAMPA,UCUS,BURNOUT,AYIR,AYRILDI_MI,AYRILDI,AYRILMADI,FINISH
};
enum ZORLU2024 BOOSTER;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
int16_t I2C_Testsensor(void);
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
float BME280_Get_Altitude(void);
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);
void union_converter();
void Altitude_Offset();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
lwgps_t gps;
LSM6DSLTR Lsm_Sensor;
FIRFilter accx;
FIRFilter IMU_GYROX;
FIRFilter IMU_GYROY;
FIRFilter IMU_GYROZ;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rx_data != '\n' && rx_index < RX_BUFFER_SIZE)
	{
		rx_buffer[rx_index++] = rx_data;
	}
	else {
		lwgps_process(&gps, rx_buffer, rx_index+1);
		rx_index = 0;
		rx_data = 0;
	}
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim11){ // 1.5sn
		lora_flag=1;

		counter++;
		if(counter == 2)
		{
			adc_flag=1;
			counter =0;
			HAL_ADC_Start_IT(&hadc1);
		}



	}

	if(htim==&htim10){ //50 ms timer
		sensor_flag=1;

	}

	if(htim==&htim6){
    //  motor_burnout++;
	}
	if(htim==&htim7){
     // kesin_burnout++;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1 )
	{
		adc= HAL_ADC_GetValue(&hadc1);


		adc_flag = 1;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  ///KURTARMA PORTLARI KAPALI EMIN OL
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);//A
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);//B
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);//C
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);//D



  HAL_UART_Receive_IT(&huart2,&rx_data,1);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_ADC_Start_IT(&hadc1);


  MAFilter_Init(&accx);
  FIRFilter_Init(&IMU_GYROY);
  FIRFilter_Init(&IMU_GYROX);
  FIRFilter_Init(&IMU_GYROZ);

  lwgps_init(&gps);
  LSM6DSLTR_Init();
  E220_CONFIG(0x6,0x4A,0X10,1); // 0x10 ch



   dev.dev_id = BME280_I2C_ADDR_PRIM;
   dev.intf = BME280_I2C_INTF;
   dev.read = user_i2c_read;
   dev.write = user_i2c_write;
   dev.delay_ms = user_delay_ms;

   rslt = bme280_init(&dev);

   dev.settings.osr_h = BME280_OVERSAMPLING_1X;
   dev.settings.osr_p = BME280_OVERSAMPLING_4X;
   dev.settings.osr_t = BME280_OVERSAMPLING_2X;
   dev.settings.filter = BME280_FILTER_COEFF_16;
   rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

   ////ALTITUDE OFFSET


    Altitude_Offset();
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
	HAL_Delay(100);
	// KalmanFilter_Init(&kf, 0.005, 0.1, 0.0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		   if(sensor_flag==1)
		   {
					sensor_flag=0;
					prev_alt=altitude;
					rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);

					rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
					if(rslt == BME280_OK)
					{
					  temperature = comp_data.temperature/100.00;
					  humidity = comp_data.humidity;
					  pressure = comp_data.pressure;
					  altitude=BME280_Get_Altitude()-offset_altitude;
					//  altitude_kalman= KalmanFilter_Update(&kf, altitude);
					  speed=(altitude-prev_alt)*20;


					}
					 LSM6DSLTR_Read_Accel_Data(&Lsm_Sensor);
					 calculate_roll_pitch(&Lsm_Sensor);
					 LSM6DSLTR_Read_Gyro_Data(&Lsm_Sensor);
					 update_angles(&Lsm_Sensor);

					 Lsm_Sensor.Accel_X=FIRFilter_Update(&accx,  Lsm_Sensor.Accel_X);

					 Lsm_Sensor.Gyro_X=FIRFilter_Update(&IMU_GYROX,  Lsm_Sensor.Gyro_X);
					 Lsm_Sensor.Gyro_Y=FIRFilter_Update(&IMU_GYROY, Lsm_Sensor.Gyro_Y);
					 Lsm_Sensor.Gyro_Z=FIRFilter_Update(&IMU_GYROZ, Lsm_Sensor.Gyro_Z);

					 toplam_pitch+= Lsm_Sensor.Pitch;
					 toplam_roll+= Lsm_Sensor.Roll;

					 sensor_counter++;
					 if(sensor_counter == 6)
					 {
						 real_pitch = toplam_pitch/6;
						 real_roll = toplam_roll/6;
						 toplam_roll=0;
						 toplam_pitch=0;
						 sensor_counter =0;
					 }


					 BUTTON_STATE=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);

		   }


   if(lora_flag==1)
   {
	   lora_flag=0;


		loratx[0]=0x8;
		loratx[1]=0x2A;
		loratx[2]=0x10;
		loratx[3]=DEVICE_ID;
		loratx[4]=gps.sats_in_view;

		union_converter();

		loratx[49]=v4_battery;
		loratx[50]=0x33; // v4mod
		loratx[51]=0;
		 for(uint8_t i=52;i<69;i++)
		 {
			loratx[i]='0';
		 }

		loratx[69]='\n';

    	HAL_UART_Transmit_IT(&huart3,loratx,LORA_TX_BUFFER_SIZE );

         }


/************************************************************************************/

/*************************************************************************************/
		  if(altitude>altitude_max) altitude_max = altitude;

		  if(speed>speed_max) speed_max = speed;

		  if( Lsm_Sensor.Accel_X> x_max) x_max =  Lsm_Sensor.Accel_X;

		  if(adc_flag ==1)
		  {
			  if(adc > 2476) adc = 2234;
			  if(adc < 1755) adc = 1755;
			  // 6V = 1755 adc val 1,41V
			  // 8.4V = 2476 adc val 1,99V 0,58V

		  adc_pil_val=(float)( ( ( (adc/4095)*3.3)-1.41) / (1.99-1.41) ) *100  ;
		  v4_battery= adc_pil_val;
		  adc_flag=0;
			}




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 34999;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65000-1;
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
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1680;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 16800;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 64999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|CS_Pin|BUZZER_Pin|GATE_D_Pin
                          |GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin|LED2_Pin|LED1_Pin
                          |GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FN_GPIO_Port, FN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 CS_Pin BUZZER_Pin GATE_D_Pin
                           GATE_C_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|CS_Pin|BUZZER_Pin|GATE_D_Pin
                          |GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin FN_Pin LED2_Pin
                           LED1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SECINP_Pin */
  GPIO_InitStruct.Pin = SECINP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SECINP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int16_t I2C_Testsensor(void){

	HAL_StatusTypeDef status;

	status=HAL_I2C_IsDeviceReady(&hi2c1,0x6A <<1, 4, 100);


	if(HAL_OK==status){

		return 1;
		}
	else {
		return 0;
		}


}
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
    HAL_Delay(50);

    char cfg_buff[8] = {0}; // E220 için 8 elemanlı bir dizi kullanıyoruz
    enum {Transparent, Fixed} mode;
    mode = MODE;

    cfg_buff[0] = ADDH;
    cfg_buff[1] = ADDL;
    cfg_buff[2] = 0x62;
    cfg_buff[3] = 0x00;
    cfg_buff[4] = CHN;

    switch(mode){
        case Transparent:
            cfg_buff[5] = 0x00;  // opsiyon
            break;
        case Fixed:
            cfg_buff[5] = 0x11;
            break;
        default:
            cfg_buff[5] = 0x11;
     }

     cfg_buff[6] = 0x00;
     cfg_buff[7] = 0x00;


    HAL_UART_Transmit(&huart3, (uint8_t*) cfg_buff, 8, 1000);

    HAL_Delay(25);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
    HAL_Delay(25);
}

float BME280_Get_Altitude(void)
{
	float press = comp_data.pressure / 10000.0;
	float temp = comp_data.temperature / 100.0;
	alt = 44330 * (1 - pow((press / 1013.25),(1/5.255)));
	//alt = ((pow((P0/press), (1/5.257))-1) * (temp + 273.15)) / 0.0065;

	return (alt);
}
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

void union_converter()
{
	 float2unit8 f2u8_gpsalt;
    f2u8_gpsalt.fVal=gps.altitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+5]=f2u8_gpsalt.array[i];
		 }

	 float2unit8 f2u8_latitude;
	 f2u8_latitude.fVal=gps.latitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+9]=f2u8_latitude.array[i];
		 }

	 float2unit8 f2u8_longitude;
	 f2u8_longitude.fVal=gps.longitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+13]=f2u8_longitude.array[i];
		 }

	 float2unit8 f2u8_altitude;
	 f2u8_altitude.fVal=altitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+17]=f2u8_altitude.array[i];
		 }
	 float2unit8 f2u8_speed;
	 f2u8_speed.fVal=speed;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+21]=f2u8_speed.array[i];
		 }

	 float2unit8 f2u8_temp;
	 f2u8_temp.fVal=temperature;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+25]=f2u8_temp.array[i];
		 }

	 float2unit8 f2u8_accx;
	 f2u8_accx.fVal=Lsm_Sensor.Accel_X;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+29]=f2u8_accx.array[i];
		 }

	 float2unit8 f2u8_accy;
	 f2u8_accy.fVal=Lsm_Sensor.Accel_Y;
	 	 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+33]=f2u8_accy.array[i];
		 }

	 float2unit8 f2u8_accz;
	 f2u8_accz.fVal=Lsm_Sensor.Accel_Z;
	 	 for(uint8_t i=0;i<4;i++)
		 {
		    loratx[i+37]=f2u8_accz.array[i];
		 }

	 float2unit8 f2u8_roll;
	 f2u8_roll.fVal=Lsm_Sensor.Roll;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+41]=f2u8_roll.array[i];
		 }

	 float2unit8 f2u8_pitch;
	 f2u8_pitch.fVal=real_pitch;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+45]=f2u8_pitch.array[i];
		 }
}

void Altitude_Offset()
{
	for(uint8_t i=0;i<5;i++)
	{
		HAL_Delay(50);
	  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	  if(rslt == BME280_OK)
	  { pressure = comp_data.pressure;
	    offset_altitude=BME280_Get_Altitude();
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
