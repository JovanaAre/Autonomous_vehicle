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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include <stdlib.h>

#include "stm32f4xx_it.h"

#include "wifi_esp.h"
#include "light_ranger.h"
#include "color_10_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_SIZE		512
#define MSG_SIZE_SMALL  32
#define TIMEOUT			100

#define SSID			"etfbl.net"
#define PASSWORD		""
#define CONNECTION_TYPE	"TCP"
#define HOST			"10.99.175.169"
#define PORT			80

#define MESSAGE_START 	"start"
#define MESSAGE_STOP	"stop"

#define DATA_FORMAT_1 	      "{\"color_buff\" : \"%s\", \"hall_int\" : %d, \"x_pos\" : %.1f, \"y_pos\" : %.1f, \"distance\" : %.1f}\r\n"
#define DATA_FORMAT_2 	      "{\"color_buff\" : \"%s\", \"hall_int\" : %d, \"x_pos\" : %.1f, \"y_pos\" : %.1f}\r\n"
#define MESSAGE_FORMAT	      "{\"type\" : \"message\", \"message\" : \"%s\"}\r\n"
#define MESSAGE_READY_FORMAT  "{\"ready\" : \"true\"}\r\n"
#define MESSAGE_START_FORMAT  "{\"started\" : \"true\"}\r\n"
#define MESSAGE_STOP_FORMAT	  "{\"stopped\" : \"true\"}\r\n"

// LightRanger 8 measurement ready or not
//
#define LR_NO_NEW_MEASUREMENT		0
#define LR_NEW_MEASUREMENT_READY	1

// Accel 13 Click registers
//
#define CTRL1_REG_ADDR 		0x20 // Control register 1 address
#define CTRL6_REG_ADDR		0x25 // Control register 6 address
#define STATUS_REG_ADDR		0x27 // Status register (RO) for checking DRDY bit
#define OUT_X_L_REG_ADDR	0x28 // X axis accel low byte address

#define CTRL1_REG_VALUE		0x74  	// 0111 0100
// High-Performance / Low-Power mode 400/200 Hz, High-Performance Mode (14-bit resolution), Low-Power Mode 1 (12-bit resolution)
#define CTRL6_REG_VALUE		0x04    // 0000 0100
// +- 2g data format, low noise

#define CONV_CONST		    0.061f // 4,000 MilliGs / 65,535 = 0.061

// The limit at which the car stops before an obstacle
//
#define DISTANCE_LIMIT  30

// Front motor control pins (connected to GPIO pins)
// 
#define MOTOR_LEFT_PIN GPIO_PIN_3
#define MOTOR_LEFT_PORT GPIOE
#define MOTOR_RIGHT_PIN GPIO_PIN_4
#define MOTOR_RIGHT_PORT GPIOE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

// TIM_HandleTypeDef htim2;
// TIM_HandleTypeDef htim3;
// TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

HAL_StatusTypeDef status;

/** Sensor modules objects **/
wifi_esp_t wifi_esp;
static light_ranger_8_t light_ranger_8;
static color_10_t color_10;

/** Wifi arrays **/
char msg_buff[MSG_SIZE];
char data[MSG_SIZE * 4];

/** Accel 13 variables **/
uint8_t acc_rx_data[6];

// Acceleration variables
//
float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

// Speed variables
//
float vx = 0.0;
float vy = 0.0;

// Position variables
//
float x_pos = 0.0;
float y_pos = 0.0;

float dt = 0.005;

/** Color 10 variables **/
uint8_t color;

/** LightRanger 8 variables **/
int16_t offset;
static uint16_t volatile distance;

static uint16_t period_ms              	= 10;
static uint32_t budget_us              	= 100000;
static int16_t calibration_distance_mm 	= 100;

static uint8_t volatile new_range_measurement;
static uint8_t volatile range_status;

/** Number of Hall sensor interrupts**/
static int hall_int = 0;

/** Start/stop from application **/
static uint8_t volatile start_stop	= 0;

/** Motors variables **/
static uint32_t forward = TIM_CHANNEL_1;
static uint32_t back = TIM_CHANNEL_2;

/** Timers control variables **/
static uint8_t tim_flag = 0;
static uint8_t tim2_flag = 0;
static uint8_t wait_red = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// Accelerometer reading function
//
void accel_read_data (uint8_t address);

// UART print function
//
void uart_print();

// Light Ranger function
//
void light_ranger_calibration();

// Color sensor functions
//
void write_color (uint8_t color, uint8_t * color_buff);
void get_color_id();

// Control motors function
//
//void control_motors();

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /** Initialize structure LightRanger 8
  */
  light_ranger_8.i2c_handle 	= &hi2c3;
  light_ranger_8.uart_handle 	= &huart4;
  light_ranger_8.en_pin_base  	= GPIOE;
  light_ranger_8.int_pin_base 	= GPIOE;
  light_ranger_8.scl_pin_base 	= GPIOA;
  light_ranger_8.sda_pin_base 	= GPIOC;
  light_ranger_8.en      		= GPIO_PIN_8;
  light_ranger_8.int_pin 		= GPIO_PIN_10;
  light_ranger_8.scl     		= GPIO_PIN_8;
  light_ranger_8.sda     		= GPIO_PIN_9;
  light_ranger_8.i2c_speed   	= I2C_MASTER_SPEED_STANDARD;
  light_ranger_8.i2c_address 	= LIGHTRANGER8_SET_DEV_ADDR;

  /** Initialize structure Color 10
  */
  color_10.i2c_handle 	= &hi2c2;
  color_10.uart_handle 	= &huart4;
  color_10.scl_pin_base = GPIOB;
  color_10.sda_pin_base = GPIOB;
  color_10.scl 			= GPIO_PIN_10;
  color_10.sda 			= GPIO_PIN_11;
  color_10.i2c_speed 	= COLOR10_I2C_SPEED;
  color_10.i2c_address 	= COLOR10_SLAVE_ADDR;

  /** Initialize structure WiFi ESP
  */ 
  wifi_esp.uart_wifi    = &huart3;
  wifi_esp.uart_prnt    = &huart4;
  wifi_esp.rst_pin_base = GPIOE;
  wifi_esp.ssid      	= (char *) SSID;
  wifi_esp.password 	= (char *) PASSWORD;
  wifi_esp.mode			= MODE_STATION;
  wifi_esp.echo			= ECHO_OFF;
  wifi_esp.mux_conn		= MUX_DISABLED;
  wifi_esp.rst_pin		= GPIO_PIN_13;
  wifi_esp.wifi_status	= WIFI_STATUS_NOT_CONNECTED;
  
  /** Accel 13 Configuration
  */ 
  uint8_t write_array[2]; // address and data
  uint8_t addr = 0x80 | CTRL1_REG_ADDR; // R/W bit = 1 for read
  uint8_t accel_data = 0x00;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, (uint8_t *) &addr, sizeof(addr), 100);
  HAL_SPI_Receive(&hspi2, (uint8_t *) &accel_data, sizeof(accel_data), 100);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  /** Write CTRL1 register
  */
  while (accel_data != CTRL1_REG_VALUE)
  {
       write_array[0] = CTRL1_REG_ADDR; //0x7f & CTRL1_REG_ADDR; R/W bit = 0 for write
       write_array[1] = CTRL1_REG_VALUE;

       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
       HAL_SPI_Transmit(&hspi2, write_array, sizeof(write_array), 100);
       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
       HAL_SPI_Transmit(&hspi2, (uint8_t *) &addr, sizeof(addr), 100);
       HAL_SPI_Receive(&hspi2, (uint8_t *) &accel_data, sizeof(accel_data), 100);
       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
  }

  /** Write CTRL6 register
  */
  write_array[0] = CTRL6_REG_ADDR;
  write_array[1] = CTRL6_REG_VALUE;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, write_array, sizeof(write_array), 100);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);


  /** Read CTRL1 register
  */
  addr = 0x80 | CTRL1_REG_ADDR;
  accel_data = 0x00;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, (uint8_t *) &addr, sizeof(addr), 100);
  HAL_SPI_Receive(&hspi2, (uint8_t *) &accel_data, sizeof(accel_data), 100);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  /** Read CTRL6 register
  */
  addr = 0x80 | CTRL6_REG_ADDR;
  accel_data = 0x00;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, (uint8_t *) &addr, sizeof(addr), 100);
  HAL_SPI_Receive(&hspi2, (uint8_t *) &accel_data, sizeof(accel_data), 100);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  /** Color 10 configuration
  */
  color10_config(&color_10, COLOR10_CFG_HIGH_DYNAMIC_RANGE_1 	|
				 COLOR10_CFG_INTEGRATION_TIME_SETT_50_MS |
				 COLOR10_CFG_AUTO_MODE |
				 COLOR10_CFG_TRIGGER_NO |
				 COLOR10_CFG_POWER_ON |
				 COLOR10_CFG_GAIN_1_X1 |
				 COLOR10_CFG_GAIN_2_X1);


  /** WiFi hardware reset
  */ 
  wifi_rst(&wifi_esp);

  /** Initialization
  */ 
  wifi_init(&wifi_esp);

  /** Get hardware version
  */ 
  wifi_hw_version(&wifi_esp);

  /** List AP
  */
  wifi_list_ap(&wifi_esp);

  /** Connect
  */
  status = wifi_connect(&wifi_esp);

  if (status == HAL_OK)
  {
	  sprintf(msg_buff, "[OK] WiFi connection\r\n");
	  uart_print();
  }
  else
  {
	  sprintf(msg_buff, "[ERROR] WiFi connection\r\n");
	  uart_print();

	  for (; ;);
  }

  /** Get IP
  */
  wifi_get_ip(&wifi_esp);

  /** TCP connect
  */ 
  wifi_tcp_connect(&wifi_esp, HOST, PORT);
  wifi_esp.wifi_status = WIFI_STATUS_CONNECTED;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /** Check color ID
  */
  get_color_id();

  /** LightRanger 8 Configuration
  */
  lightranger8_power_on(&light_ranger_8);
  if (lightranger8_default_cfg(&light_ranger_8) != 0)
  {
	  char msg[MSG_SIZE];
      sprintf(msg, " -Light Ranger 8: Sensor configuration error");
      sprintf(data, MESSAGE_FORMAT, msg);
      wifi_send_data(&wifi_esp, data, strlen(data));
  }

  lightranger8_set_distance_mode(&light_ranger_8, LIGHTRANGER8_DISTANCE_MODE_MEDIUM);
  lightranger8_set_measurement_timing_budget(&light_ranger_8, budget_us);

  HAL_Delay(1000);

  /** LightRanger 8 Calibration
  */
  light_ranger_calibration();

   /** Say that device is ready
   */ 
  sprintf(data, MESSAGE_READY_FORMAT);
  wifi_send_data(&wifi_esp, data, strlen(data));

  /** Wait for START
  */
  char *res = NULL;
  do
  {
	  wifi_get_data(&wifi_esp, data, MSG_SIZE * 4);
	  res = strstr((char *) data, MESSAGE_START);
  } while (res == NULL);

  /** Say that device has started
  */
  sprintf(data, MESSAGE_START_FORMAT);
  wifi_send_data(&wifi_esp, data, strlen(data));

  res = NULL;
  start_stop = 1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (start_stop == 1)
	  {
	 	 wifi_get_data(&wifi_esp, data, MSG_SIZE * 4);
	 	 res = strstr((char *) data, MESSAGE_STOP);

	 	 if (res != NULL)
	 	 {
	 	   start_stop = 0;
	 	   res = NULL;
	 	 }
	  }
	  else if (start_stop == 0)
	  {
	 	 wifi_get_data(&wifi_esp, data, MSG_SIZE * 4);
	 	 res = strstr((char *) data, MESSAGE_START);

	 	 if (res != NULL)
	 	 {
	 		 start_stop = 1;
	 		 res = NULL;
	 	 }
	  }

	  if (start_stop == 1)
	  {
		  // Local varisables for Color 10
		  //
		  uint8_t color_buff[MSG_SIZE_SMALL];
		  uint16_t read_data;
		  float color_data;
		  
		  // Local varisables for Accel 13
		  //
		  uint8_t addr;
		  uint8_t accel_data;
		  int16_t x, y, z;
		  
		   /** Accel 13 reading
		   */
		  addr = 0x80 | STATUS_REG_ADDR;
		  accel_data = 0x00;

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_SPI_Transmit(&hspi2, (uint8_t *) &addr, sizeof(addr), 100);
		  HAL_SPI_Receive(&hspi2, (uint8_t *) &accel_data, sizeof(accel_data), 100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

		  if (accel_data & 0x01) // DRDY bit checking (if DRDY == 1 read data)
		  {
		    addr = OUT_X_L_REG_ADDR;
		    accel_read_data(addr);

		    // Raw values from the accelerometer
		    x = (acc_rx_data[1] << 8) | acc_rx_data[0];
		    y = (acc_rx_data[3] << 8) | acc_rx_data[2];
		    z = (acc_rx_data[5] << 8) | acc_rx_data[4];

		    // Convert into g
		    ax = (x * CONV_CONST) / 1000;
		    ay = (y * CONV_CONST) / 1000;
		    az = (z * CONV_CONST) / 1000;

		    ax *= 9.81;
		    ay *= 9.81;
		    az *= 9.81;

		    vx = vx + (ax * dt);
		    vy = vy + (ay * dt);
		    x_pos = x_pos + (vx * dt);
		    y_pos = y_pos + (vy * dt);

		  }

		  /** Color 10 reading
		  */
		  read_data = color10_generic_read(&color_10, COLOR10_CMD_REG_IR); // IR value
		  color_data = color10_get_color_value(&color_10);
		  color = color10_get_color(color_data);
		  write_color(color, color_buff);

		  if (new_range_measurement == LR_NEW_MEASUREMENT_READY)
		  {
			  sprintf(data, DATA_FORMAT_1, color_buff, hall_int,
					  x_pos, y_pos, ((float) distance) / 10);
			  new_range_measurement = LR_NO_NEW_MEASUREMENT;
		  }
		  else
		  {
			  sprintf(data, DATA_FORMAT_2, color_buff, hall_int,
					  x_pos, y_pos);
		  }

		  wifi_send_data(&wifi_esp, data, strlen(data));
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE12 PE13
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief Accel 13 reading Function
  * @details This function reads Accel 13 data from passed address.
  * @param[in] color : Data address
  * @retval None
  */
void accel_read_data (uint8_t address)
{
	uint8_t addr = 0x80 | address;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *) &addr, sizeof(addr), 100);
	HAL_SPI_Receive(&hspi2, acc_rx_data, sizeof(acc_rx_data), 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
}

/**
  * @brief UART Print Function
  * @param None
  * @retval None
  */
void uart_print()
{
	HAL_UART_Transmit(&huart4, (uint8_t *) msg_buff, strlen(msg_buff), TIMEOUT);
}

/**
  * @brief LightRanger 8 Calibration Function
  * @details This function implements LightRanger 8
  * calibration.
  * @param None
  * @retval None
  */
void light_ranger_calibration()
{
	  char msg[MSG_SIZE];
	  sprintf(msg, " -Light Ranger 8: Calibration; Place an object at %.1f cm distance from sensor.", ((float) calibration_distance_mm) / 10);
	  sprintf(data, MESSAGE_FORMAT, msg);
	  wifi_send_data(&wifi_esp, data, strlen(data));

	  HAL_Delay(5000);

	  sprintf(msg, " -Light Ranger 8: Sensor calibration is in progress...");
	  sprintf(data, MESSAGE_FORMAT, msg);
	  wifi_send_data(&wifi_esp, data, strlen(data));

	  HAL_Delay(5000);

	  /** Calibration
	  */
	  lightranger8_calibrate_offset(&light_ranger_8, calibration_distance_mm, period_ms, &offset);

	  HAL_Delay(500);

	  lightranger8_start_measurement(&light_ranger_8, period_ms);

	  sprintf(msg, " -Light Ranger 8: Calibration done");
	  sprintf(data, MESSAGE_FORMAT, msg);
	  wifi_send_data(&wifi_esp, data, strlen(data));

	  HAL_Delay(50);

	  sprintf(msg, " -Light Ranger 8: Device ready, waiting for START");
	  sprintf(data, MESSAGE_FORMAT, msg);
	  wifi_send_data(&wifi_esp, data, strlen(data));

	  HAL_Delay(100);
}

/**
  * @brief Color Writing Function
  * @details This function returns the color name based on the
  * corresponding color value.
  * @param[in] color : Color value
  * @param[out] color_buff : Output color string
  * @retval None
  */
void write_color (uint8_t color, uint8_t * color_buff)
{

    switch ( color )
    {
    	case COLOR10_COLOR_ORANGE:
    		 sprintf((char *) color_buff, "ORANGE");
    		 break;
    	case COLOR10_COLOR_RED:
    		 sprintf((char *) color_buff, "RED");
    		 break;
    	case COLOR10_COLOR_PINK:
    		 sprintf((char *) color_buff, "PINK");
    		 break;
    	case COLOR10_COLOR_PURPLE:
    		 sprintf((char *) color_buff, "PURPLE");
    		 break;
    	case COLOR10_COLOR_BLUE:
    		 sprintf((char *) color_buff, "BLUE");
    		 break;
    	case COLOR10_COLOR_CYAN:
    		 sprintf((char *) color_buff, "CYAN");
    		 break;
    	case COLOR10_COLOR_GREEN:
    		 sprintf((char *) color_buff, "GREEN");
    		 break;
    	case COLOR10_COLOR_YELLOW:
    		 sprintf((char *) color_buff, "YELLOW");
    		 break;
    	default:
    		 sprintf((char *) color_buff, "NOT DEFINED");
    }
}

/**
  * @brief Color 10 Checking device ID Function
  * @details This function checks if the device ID is OK
  * and prints a corresponding message.
  * @param None
  * @retval None
  */
void get_color_id()
{
    if ( color10_get_id(&color_10) == COLOR10_DEVICE_ID )
    {
	   char msg[MSG_SIZE];
	   sprintf(msg, " - Color 10: Device ID OK");
	   sprintf(data, MESSAGE_FORMAT, msg);
	   wifi_send_data(&wifi_esp, data, strlen(data));
    }
    else
    {
	   char msg[MSG_SIZE];
       sprintf(data, " - Color 10: Device ID error");
	   sprintf(data, MESSAGE_FORMAT, msg);
	   wifi_send_data(&wifi_esp, data, strlen(data));
    }
}

/**
  * @brief EXTI External Interrupt ISR Handler CallBackFun
  * @param[in] GPIO_Pin: External interrupt pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_1) // If the INT source is EXTI Line1 (PD1 Pin)
    {
    	/** Toggle the output (LED) Pin
		*/
    	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		
		/** Increment number of hall sensor interrupts
		*/
    	hall_int++;
    	// distance_traveled_cm = hall_int * CIRCUMFERENCE;
    }
	if (GPIO_Pin == GPIO_PIN_10) // If the INT source is EXTI Line10 (PE10 Pin)
	{
		new_range_measurement = LR_NEW_MEASUREMENT_READY;
		distance = lightranger8_get_distance(&light_ranger_8);
		range_status = lightranger8_get_range_status(&light_ranger_8);
		
		if (start_stop == 1)
		{
			sprintf(msg_buff, "-Light Ranger 8: Interrupt detected. Distance is %d Range status is %d\r\n", distance, range_status);
			uart_print();

			/** Motor control
			*/
			//control_motors();

			if (distance < DISTANCE_LIMIT * 10)
			{
				/** Set the output (LED) Pin
				*/
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
			}
			else
			{	    					
				/** Set the output (LED) Pin
				*/
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
			}
		}
		else
		{
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
		}
			
		/** Clear interrupt
		*/
		lightranger8_system_interrupt_clear(&light_ranger_8);
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
