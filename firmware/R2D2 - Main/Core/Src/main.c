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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "ili9341.h"
#include "ili9341_gfx.h"
#include "bmp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SECTOR_TOP 0;
#define SECTOR_RIGHT 1;
#define SECTOR_BOTTOM 2;
#define SECTOR_LEFT 3;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

extern const unsigned char bmp[];

// FDCAN2 Defines
FDCAN_TxHeaderTypeDef   TxHeaderDriveLeft;
FDCAN_TxHeaderTypeDef   TxHeaderDriveRight;
FDCAN_RxHeaderTypeDef   RxHeader1;
uint8_t               TxDataDriveLeft[2];
uint8_t               TxDataDriveRight[2];

uint8_t               RxData1[2];

uint32_t indx;

uint8_t speed;
volatile uint8_t uartRxData[10];
bool uartReceiveEvent;

int8_t joystickX;
int8_t joystickY;

uint8_t pwmDutyCycleLeft;
uint8_t pwmDutyCycleRight;

ili9341_t *_lcd;
ili9341_text_attr_t text;
ili9341_text_attr_t text_connected;
ili9341_text_attr_t text_disconnected;
ili9341_text_attr_t text_battery;

uint16_t lastPointerPositionLeft = 1;
int8_t lastPointerPositionDirectionLeft;
uint16_t lastPointerPositionRight = 1;
int8_t lastPointerPositionDirectionRight;

bool websocketConnected;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float calculate_differential_drive_factor(int8_t joystickXpos, int8_t joystickYpos){
	if(joystickXpos > joystickYpos){
		return (1 - (float)joystickYpos / (float)joystickXpos);
	}
	else if(joystickYpos > joystickXpos){
		return (1 - (float)joystickXpos / (float)joystickYpos);
	}
	else{
		return 0;
	}
}

int8_t get_joystick_sector(int8_t joystickXpos, int8_t joystickYpos){
	if(joystickY > 0 && joystickYpos > joystickXpos){
		return SECTOR_BOTTOM;
	}
	else if(joystickX > 0 && joystickXpos > joystickYpos){
		return SECTOR_RIGHT;
	}
	else if(joystickY < 0 && joystickYpos > joystickXpos){
		return SECTOR_TOP;
	}
	else if(joystickX < 0 && joystickXpos > joystickYpos){
		return SECTOR_LEFT;
	}
	else{
		return -1;
	}
}

void calculate_pwm_values(int8_t sector, float differentialDriveFactor){
	if(sector == -1){
		pwmDutyCycleLeft = 0;
		pwmDutyCycleRight = 0;
		TxDataDriveLeft[0] = 'S';
		TxDataDriveRight[0] = 'S';
	}
	else{
		switch(sector){
		case 0: // move forward with steering
			TxDataDriveLeft[0] = 'F';
			TxDataDriveRight[0] = 'F';
			if(joystickX > 0){
				pwmDutyCycleLeft = speed;
				pwmDutyCycleRight = speed * differentialDriveFactor;
			}
			else{
				pwmDutyCycleLeft = speed * differentialDriveFactor;
				pwmDutyCycleRight = speed;
			}
			break;
		case 1: // rotate right
			TxDataDriveLeft[0] = 'F';
			TxDataDriveRight[0] = 'B';
			pwmDutyCycleLeft = speed;
			pwmDutyCycleRight = speed;
			break;
		case 2: // move backward with steering
			TxDataDriveLeft[0] = 'B';
			TxDataDriveRight[0] = 'B';
			if(joystickX > 0){
				pwmDutyCycleLeft = speed * differentialDriveFactor;
				pwmDutyCycleRight = speed;
			}
			else{
				pwmDutyCycleLeft = speed;
				pwmDutyCycleRight = speed * differentialDriveFactor;
			}
			break;
		case 3: // rotate left
			TxDataDriveLeft[0] = 'B';
			TxDataDriveRight[0] = 'F';
			pwmDutyCycleLeft = speed;
			pwmDutyCycleRight = speed;
			break;
		}
	}
}

void display_telemetry(void){
	uint16_t pointerPositionLeft = (pwmDutyCycleLeft - 50) * 1.6;
	if(lastPointerPositionLeft != pointerPositionLeft){
		if(lastPointerPositionDirectionLeft == 'F'){
			ili9341_fill_rect(_lcd, ILI9341_NAVY, 245, 97 - lastPointerPositionLeft, 15, 6);
		}
		else{
			ili9341_fill_rect(_lcd, ILI9341_NAVY, 245, 97 + lastPointerPositionLeft, 15, 6);
		}
		if(TxDataDriveLeft[0] == 'F'){
			ili9341_fill_rect(_lcd, ILI9341_RED, 245, 97 - pointerPositionLeft, 15, 6);
		}
		else{
			ili9341_fill_rect(_lcd, ILI9341_RED, 245, 97 + pointerPositionLeft, 15, 6);
		}
 		lastPointerPositionLeft = pointerPositionLeft;
 		lastPointerPositionDirectionLeft = TxDataDriveLeft[0];
	}

	uint16_t pointerPositionRight = (pwmDutyCycleRight - 50) * 1.6;
	if(lastPointerPositionRight != pointerPositionRight){
		if(lastPointerPositionDirectionRight == 'F'){
			ili9341_fill_rect(_lcd, ILI9341_NAVY, 275, 97 - lastPointerPositionRight, 15, 6);
		}
		else{
			ili9341_fill_rect(_lcd, ILI9341_NAVY, 275, 97 + lastPointerPositionRight, 15, 6);
		}
		if(TxDataDriveRight[0] == 'F'){
			ili9341_fill_rect(_lcd, ILI9341_RED, 275, 97 - pointerPositionRight, 15, 6);
		}
		else{
			ili9341_fill_rect(_lcd, ILI9341_RED, 275, 97 + pointerPositionRight, 15, 6);
		}
		lastPointerPositionRight = pointerPositionRight;
		lastPointerPositionDirectionRight = TxDataDriveRight[0];
	}

	if(websocketConnected){
		ili9341_draw_string(_lcd, text_connected, "Connected    ");
	}
	else{
		ili9341_draw_string(_lcd, text_disconnected, "Disconnected!");
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
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_USB_Device_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK)
  {
   Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  // Configure TX Header for FDCAN1
  TxHeaderDriveLeft.Identifier = 0x12;
  TxHeaderDriveLeft.IdType = FDCAN_STANDARD_ID;
  TxHeaderDriveLeft.TxFrameType = FDCAN_DATA_FRAME;
  TxHeaderDriveLeft.DataLength = FDCAN_DLC_BYTES_2;
  TxHeaderDriveLeft.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeaderDriveLeft.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeaderDriveLeft.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeaderDriveLeft.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeaderDriveLeft.MessageMarker = 0;

  TxHeaderDriveRight.Identifier = 0x13;
  TxHeaderDriveRight.IdType = FDCAN_STANDARD_ID;
  TxHeaderDriveRight.TxFrameType = FDCAN_DATA_FRAME;
  TxHeaderDriveRight.DataLength = FDCAN_DLC_BYTES_2;
  TxHeaderDriveRight.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeaderDriveRight.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeaderDriveRight.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeaderDriveRight.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeaderDriveRight.MessageMarker = 0;

  HAL_Delay(1000); // give the ESP01 time to boot (the ESP sends unknown data over UART on boot)
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)uartRxData, sizeof(uartRxData));

  _lcd = ili9341_new(
        &hspi1,
        RST_GPIO_Port, RST_Pin,
        SPI_NSS_GPIO_Port,    SPI_NSS_Pin,
        DC_GPIO_Port,    DC_Pin,
        isoLandscape,
        TOUCH_CS_GPIO_Port,  TOUCH_CS_Pin,
        TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin,
        itsSupported,
        itnNormalized);

  text.bg_color = ILI9341_NAVY;
  text.fg_color = ILI9341_WHITE;
  text.font = &ili9341_font_16x26;
  text.origin_x = 30;
  text.origin_y = 210;

  text_connected.bg_color = ILI9341_NAVY;
  text_connected.fg_color = ILI9341_GREEN;
  text_connected.font = &ili9341_font_7x10;
  text_connected.origin_x = 140;
  text_connected.origin_y = 10;

  text_disconnected.bg_color = ILI9341_NAVY;
  text_disconnected.fg_color = ILI9341_RED;
  text_disconnected.font = &ili9341_font_7x10;
  text_disconnected.origin_x = 140;
  text_disconnected.origin_y = 10;

  text_battery.bg_color = ILI9341_NAVY;
  text_battery.fg_color = ILI9341_GREEN;
  text_battery.font = &ili9341_font_7x10;
  text_battery.origin_x = 165;
  text_battery.origin_y = 60;

  ili9341_fill_screen(_lcd, ILI9341_NAVY);
  ili9341_draw_string(_lcd, text, "R2D2 Version 0.1");
  ili9341_draw_string(_lcd, text_disconnected, "Disconnected!");
  ili9341_draw_rect(_lcd, ILI9341_WHITE, 220, 20, 25, 160);
  ili9341_draw_rect(_lcd, ILI9341_WHITE, 290, 20, 25, 160);

  ili9341_draw_rect(_lcd, ILI9341_BLACK, 160, 80, 40, 100);
  ili9341_fill_rect(_lcd, ILI9341_BLACK, 170, 71, 20, 10);
  ili9341_draw_string(_lcd, text_battery, "100%");

//  ili9341_draw_bitmap_1b(_lcd, ILI9341_WHITE, ILI9341_NAVY, 100, 100, 4, 4, bmp);
  ili9341_draw_bitmap_1b(_lcd, ILI9341_NAVY, ILI9341_WHITE,
          1, 3, 130, 200, (uint8_t *)bmp);

  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   if(uartReceiveEvent){
		   if(uartRxData[0] == 'M'){
			   joystickX = atoi((uint8_t *)&uartRxData[1]);
			   joystickY = atoi((uint8_t *)&uartRxData[5]);

			   speed = 0.393700787 * (float)sqrt(joystickX * joystickX + joystickY * joystickY) + 50.0;
			   if(speed > 100){
				   speed = 100;
			   }

			   uint8_t joystickXpos = (joystickX < 0) ? -joystickX : joystickX;
			   uint8_t joystickYpos = (joystickY < 0) ? -joystickY : joystickY;
			   float differentialDriveFactor = calculate_differential_drive_factor(joystickXpos, joystickYpos);
			   int8_t sector = get_joystick_sector(joystickXpos, joystickYpos);
			   calculate_pwm_values(sector, differentialDriveFactor);

			   TxDataDriveLeft[1] = pwmDutyCycleLeft;
			   TxDataDriveRight[1] = pwmDutyCycleRight;

			   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeaderDriveLeft, TxDataDriveLeft);
			   HAL_Delay(2);
			   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeaderDriveRight, TxDataDriveRight);
		   }
		   else if(uartRxData[0] == 'L'){

		   }
		   else if(uartRxData[0] == 'R'){

		   }
		   else if(uartRxData[0] == 'S'){

		   }
		   else if(strstr(uartRxData, 'connect')){
			   if(uartRxData[0] == '!'){
				   websocketConnected = false;
			   }
			   else{
				   websocketConnected = true;
			   }
		   }
		   uartReceiveEvent = false;
	   }

	   display_telemetry();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 100;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 18;
  hfdcan2.Init.NominalTimeSeg2 = 15;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x10;
  sFilterConfig.FilterID2 = 0x10;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  /* USER CODE END FDCAN2_Init 2 */

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
  hi2c1.Init.Timing = 0x40B285C2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 42500-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TOUCH_CS_Pin|TOUCH_IRQ_Pin|RST_Pin|SPI_NSS_Pin
                          |DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_MODE_SELECT_GPIO_Port, CAN_MODE_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TOUCH_CS_Pin TOUCH_IRQ_Pin RST_Pin SPI_NSS_Pin
                           DC_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin|TOUCH_IRQ_Pin|RST_Pin|SPI_NSS_Pin
                          |DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_MODE_SELECT_Pin */
  GPIO_InitStruct.Pin = CAN_MODE_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_MODE_SELECT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uartReceiveEvent = true;
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	websocketConnected = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	websocketConnected = false;
}

// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData1) != HAL_OK)
    {
    /* Reception Error */
    Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
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
