/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "circular_q.h"
#include "kalman.h"

#include "max30102_for_stm32_hal.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "pulse_detection_algorithm.h"

#define ARM_MATH_CM4
#include "arm_math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define buffer_size 10

#define VREF_INT_ADDR (uint16_t*)(uint32_t) 0x1FFF75AA //length 2 bytes

#define TS_CAL1_ADDR (uint16_t*)(uint32_t) 0x1FFF75A8
#define TS_CAL2_ADDR (uint16_t*)(uint32_t) 0x1FFF75CA

#define PI 3.14f
#define MAX_VALUE 3276 //12 bit DAC 80% of 4095

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

max30102_t max30102;
// Heart rate calculation variables
#define RATE_SIZE 4  // Size of the array for storing heartbeat rates
uint32_t lastBeat = 0;  // Time of the last detected heartbeat
float beatsPerMinute = 0;
uint8_t beatAvg = 0;


int tone; //keeps track of tone
int C6_size = 44; // C6 is 1 kHz, so 44 samples per period
int E6_size = 33; // E6 is 1.3 kHz, so 33 samples per period
int G6_size = 28; // G6 is 1.57 kHz, so 28 samples per period
uint16_t C6_data[44];
uint16_t E6_data[33];
uint16_t G6_data[28];


//accelero

//store kalman state
struct kalman_state x_state;
struct kalman_state y_state;
struct kalman_state z_state;

//store accelerometer values
int16_t aDataXYZ[3];
float aDataXYZ_norm[3];
float ax, ay, az;
//store tilt values
float tilt_x, tilt_y, tilt_z;
//store square roots of denominators in arctan
float root_x, root_y, root_z;

//buffer for last 20 values
struct queue buffer_x;
struct queue buffer_y;
struct queue buffer_z;
struct queue buffer_heart;

float array_x[buffer_size];
float array_y[buffer_size];
float array_z[buffer_size];
float array_heart[4];

//alarm flag, checked to sound alarm thru DAC speaker
uint8_t alarm = 0;
uint8_t temp_alarm = 0;

//UART output
char output[100];


//adc
int CHANNEL = ADC_CHANNEL_VREFINT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//overwrite the write fucntion to use printf with UART transmit
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

void generate_sine_wave(uint16_t wave[], int num_samples){
	float step_angle = 2.0f * PI / num_samples;
	int max_amplitude = MAX_VALUE / 2;

	for (int i = 0; i < num_samples; i++) {
		wave[i] = max_amplitude * (1 + arm_sin_f32(step_angle* i)); //sine values are between -1 and 1, add 1 to make them between 0 and 2
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //generate DAC waves
  generate_sine_wave(C6_data, C6_size);
  generate_sine_wave(G6_data, G6_size);
  generate_sine_wave(E6_data, E6_size);

  //setup temperature adc
  uint16_t vref_int = *VREF_INT_ADDR;
  float vref_cal = 3.0;

  float ts_cal2_temp = 130.0;

  float ts_cal1_temp = 30.0;
  uint16_t ts_cal1 = *TS_CAL1_ADDR;
  uint16_t ts_cal2 = *TS_CAL2_ADDR;
  uint16_t adc_temp_value;
  float temp_value;

  HAL_Delay(1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t adc_volt_value = HAL_ADC_GetValue(&hadc1);

  float vref_value = (vref_cal * vref_int) / adc_volt_value;
  CHANNEL = ADC_CHANNEL_TEMPSENSOR;
  MX_ADC1_Init();

  //setup heart sensor
  lastBeat = HAL_GetTick();
  beatsPerMinute = 0;
  for (uint8_t i = 0; i < RATE_SIZE; i++) {
      rates[i] = 0;
  }
  beatAvg = 0;
 // Initialize sensor
  max30102_init(&max30102, &hi2c1);


  // Reset sensor and clear FIFO
  max30102_reset(&max30102);
  max30102_clear_fifo(&max30102);

    // Configure FIFO settings
  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

  // Configure LED settings
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  max30102_set_sampling_rate(&max30102, max30102_sr_800);
  max30102_set_led_current_1(&max30102, 6.2);
  max30102_set_led_current_2(&max30102, 6.2);

  // Enter SpO2 measurement mode
  max30102_set_mode(&max30102, max30102_heart_rate);

  // Enable interrupts
  max30102_set_a_full(&max30102, 1);
  max30102_set_die_temp_en(&max30102, 1);
  max30102_set_die_temp_rdy(&max30102, 1);

  printf("MAX30102 initialized and configured\r\n");


  //start timer 2, used by dma
  HAL_TIM_Base_Start_IT(&htim2);

  //init accelerometer
  BSP_ACCELERO_Init();
  buffer_x.start = 0;
  buffer_x.size = buffer_size;
  buffer_x.end = buffer_size- 1;
  buffer_x.array = array_x;

  buffer_y.start = 0;
  buffer_y.size = buffer_size;
  buffer_y.end = buffer_size- 1;
  buffer_y.array = array_y;

  buffer_z.start = 0;
  buffer_z.size = buffer_size;
  buffer_z.end = buffer_size- 1;
  buffer_z.array = array_z;

  buffer_heart.start = 0;
  buffer_heart.size = 4;
  buffer_heart.end = RATE_SIZE - 1;
  buffer_heart.array = array_heart;


  //init kalman filter for accelerometer values
  BSP_ACCELERO_AccGetXYZ(aDataXYZ);

  x_state.x = aDataXYZ[0];
  x_state.p = 0.1;
  x_state.q = 0.1;
  x_state.k = 0.0;
  x_state.r = 0.1;

  y_state.x = aDataXYZ[1];
  y_state.p = 0.1;
  y_state.q = 0.1;
  y_state.k = 0.0;
  y_state.r = 0.1;

  z_state.x = aDataXYZ[2];
  z_state.p = 0.1;
  z_state.q = 0.1;
  z_state.k = 0.0;
  z_state.r = 0.1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint16_t len;

	//get temperature values
	HAL_ADC_Start(&hadc1);
	HAL_Delay(1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_temp_value = HAL_ADC_GetValue(&hadc1);
	temp_value = ((ts_cal2_temp - ts_cal1_temp) / (float)(ts_cal2 - ts_cal1)) * (adc_temp_value * (vref_value / vref_cal) - (float)ts_cal1) + 30.0;

	if(temp_value >= 38 && alarm == 0) { //sound alarm if temperature becomes too high
		alarm = 1;
	}

	//read accelerometer raw values
	BSP_ACCELERO_AccGetXYZ(aDataXYZ);
	// normalize the values
	aDataXYZ_norm[0] = (aDataXYZ[0] / 1024.0f);
	aDataXYZ_norm[1] = (aDataXYZ[1] / 1024.0f);
	aDataXYZ_norm[2] = (aDataXYZ[2] / 1024.0f);

	ax = aDataXYZ_norm[0];
	ay = aDataXYZ_norm[1];
	az = aDataXYZ_norm[2];

	//get square root of denominators in arctan
	arm_sqrt_f32(ay*ay + az*az, &root_x);
	arm_sqrt_f32(ax*ax + az*az, &root_y);
	arm_sqrt_f32(ay*ay + ax*ax, &root_z);

	//get tilt values in degrees
	tilt_x = atan2(ax,root_x) * (180.0f / 3.14f);
	tilt_y = atan2(ay,root_y) * (180.0f / 3.14f);
	tilt_z = atan2(az,root_z) * (180.0f / 3.14f);

	//apply kalman filter on each axis value
	tilt_x = update_kalman_c(&x_state, tilt_x);
	tilt_y = update_kalman_c(&y_state, tilt_y);
	tilt_z = update_kalman_c(&z_state, tilt_z);

	//add the values read to the circular buffers
	queue_add(&buffer_x, tilt_x);
	queue_add(&buffer_y, tilt_y);
	queue_add(&buffer_z, tilt_z);

	//get the new averages
	float avg_x = queue_average(&buffer_x);
	float avg_y = queue_average(&buffer_y);
	float avg_z = queue_average(&buffer_z);

	if(tilt_z >= -90 && tilt_z <= -80 && alarm == 0) { //if 90 degrees rotation about z axis, sound alarm
		alarm = 1;
	}



	//read heart sensor IR values and calculate heart rate

  // Read data from FIFO
  max30102_read_fifo(&max30102);

  // Access the data from the max30102 object
  // The first sample in each array is the most recent
  uint32_t ir_sample = max30102._ir_samples[0];
  uint32_t red_sample = max30102._red_samples[0];

  // Check for heartbeat

  if (checkForBeat(ir_sample) == true)
  {
	  // We sensed a beat!
	  uint32_t currentTime = HAL_GetTick();
	  uint32_t delta = currentTime - lastBeat;
	  lastBeat = currentTime;

	  beatsPerMinute = 60 / (delta / 1000.0);

	  if (beatsPerMinute < 255 && beatsPerMinute > 20)
	  {
		  queue_add(&buffer_heart, beatsPerMinute);

		  beatAvg = queue_average(&buffer_heart);

		  // Print heart rate information
		sprintf(output, "\r\n================= SENSOR READINGS =================\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), 200);

		sprintf(output, "💓 Heart Rate : %.1f BPM\r\n", beatsPerMinute);
		HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), 200);

		sprintf(output, "🌡 Temperature : %.1f °C\r\n", temp_value);
		HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), 200);

		sprintf(output, "📐 Tilt Angles : X = %.1f°, Y = %.1f°, Z = %.1f°\r\n", tilt_x, tilt_y, tilt_z);
		HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), 200);

		sprintf(output, "==================================================\r\n\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), 200);

		if(beatAvg > 170 && alarm == 0) { //if higher than 170, sound alarm
			alarm = 1;
		}
	  }
  }

  if(alarm == 1) {
	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, G6_data, G6_size, DAC_ALIGN_12B_R);
	  alarm = 2; //alarm is playing is 2
  }

	//HAL_Delay(10);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x30A175AB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2721;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(alarm == 2) {
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
		alarm = 0;
	}
}

//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
//	if(htim == &htim2) {
////		index_v = (index_v + 1) % c6_samples;
////		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, c6_sine_wave[index_v]);
//	}
//}

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
