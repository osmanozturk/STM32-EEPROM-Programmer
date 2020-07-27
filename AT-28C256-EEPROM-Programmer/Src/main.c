/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd_i2cModule.h"
#include "lcd_userConf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef* name;
	uint16_t number;
} CertainPin;


typedef enum {
	GPIOINPUT = 0,
	GPIOOUTPUT
} GPIODIRECTION;

typedef enum {
	MSBFIRST = 0,
	LSBFIRST
} BITORDER;

typedef enum {
	READ = 0,
	WRITE,
	UNKNOWN
} SELECTEDMODE;

typedef enum {
	PREBEGIN = 0,
	BEGINNING,
	GETADDRESS,
	GETINPUTDATA,
	PERFORM,
	END
} OPERATIONSTATUS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDRESSLENGTH 15
#define OUTPUTLENGTH 8
#define SELECTREAD 7
#define SELECTWRITE 9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//There is no need for seperate address pins since we are using shift registers now
//CertainPin addressPins[15] = {{GPIOC, GPIO_PIN_0}, {GPIOC, GPIO_PIN_1}, {GPIOC, GPIO_PIN_2}, {GPIOC, GPIO_PIN_3},
//		{GPIOC, GPIO_PIN_4}, {GPIOC, GPIO_PIN_5}, {GPIOC, GPIO_PIN_6}, {GPIOC, GPIO_PIN_7}, {GPIOC, GPIO_PIN_8},
//		{GPIOC, GPIO_PIN_9}, {GPIOC, GPIO_PIN_10}, {GPIOC, GPIO_PIN_11}, {GPIOC, GPIO_PIN_12}, {GPIOB, GPIO_PIN_13}, {GPIOB, GPIO_PIN_14}};

CertainPin outputPins[8] = {{GPIOB, GPIO_PIN_0}, {GPIOB, GPIO_PIN_1}, {GPIOB, GPIO_PIN_2}, {GPIOB, GPIO_PIN_3},
		{GPIOB, GPIO_PIN_4}, {GPIOB, GPIO_PIN_5}, {GPIOB, GPIO_PIN_6}, {GPIOB, GPIO_PIN_7}};

CertainPin chipEnable = {GPIOA, GPIO_PIN_12}; //yesil
CertainPin outputEnable = {GPIOA, GPIO_PIN_11}; //sari
CertainPin writeEnable = {GPIOA, GPIO_PIN_15}; //mavi
CertainPin addressRegisterLatchPin = {GPIOA, GPIO_PIN_6};

char dataOutBuff[9];
char uartAddrReceiveBuff[5];
char uartOpReceiveBuff;
char uartInputReceiveBuff[3]; //legths are determined by char length
int messagePrinted = 0;
int startup = 1;

OPERATIONSTATUS status = PREBEGIN;
SELECTEDMODE mode = UNKNOWN;
uint16_t selectedAddr = 0;
uint8_t selectedInput = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void setAddress(uint16_t address);
int readOutput(char* buff, uint16_t address);
int writeData(uint16_t address, int8_t data);
void setPinDirection(GPIODIRECTION direction);
void delay_microsecond(uint16_t delay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Will shift given data to the shift register but won't latch it to the output
 * It won't use SPI and manually transfer the bits
 * **/
void shiftOut(CertainPin dataPin, CertainPin clockPin, BITORDER bitOrder, uint8_t data) {
	HAL_GPIO_WritePin(clockPin.name, clockPin.number, GPIO_PIN_RESET); //ensuring clock direction by setting it beforehand

	if (bitOrder == LSBFIRST) {
		for (int i = 0; i < 8; i++) {
			HAL_GPIO_WritePin(dataPin.name, dataPin.number, data & 1); //taking lsb
			HAL_GPIO_TogglePin(clockPin.name, clockPin.number); //giving clock pulse
			HAL_GPIO_TogglePin(clockPin.name, clockPin.number);
			data = (data >> 1);
		}
	}

	else {
		for (int i = 0; i < 8; i++) {
			HAL_GPIO_WritePin(dataPin.name, dataPin.number, data & 128); //taking msb
			HAL_GPIO_TogglePin(clockPin.name, clockPin.number); //giving clock pulse
			HAL_GPIO_TogglePin(clockPin.name, clockPin.number);
			data = (data << 1);
		}
	}
}



void delay_microsecond(uint16_t delay) {

	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delay) {
	}
}

/**
 * Sets EEPROM address lines to given address via SPI(Shift registers)
 * */
void setAddress(uint16_t address) {
	uint8_t msb = address >> 8; //storing it is not necessary especially with manual shiftOut function
	HAL_GPIO_WritePin(addressRegisterLatchPin.name, addressRegisterLatchPin.number, GPIO_PIN_RESET); //keeping latch clock low before sending data
	HAL_SPI_Transmit(&hspi4, &msb, 1, 0xFF);
	HAL_SPI_Transmit(&hspi4, &address, 1, 0xFF);
	HAL_GPIO_TogglePin(addressRegisterLatchPin.name, addressRegisterLatchPin.number);
	delay_microsecond(10);
	HAL_GPIO_TogglePin(addressRegisterLatchPin.name, addressRegisterLatchPin.number);
}

/**
 * Outputs to given buffer as binary string with trailing null if buff is not null
 * **/
int readOutput(char* buff, uint16_t address) {
	int data = 0;
	GPIO_PinState bit = 0;
	setAddress(address);
	setPinDirection(GPIOINPUT);
	HAL_Delay(10);
	HAL_GPIO_WritePin(writeEnable.name, writeEnable.number, GPIO_PIN_SET); //not programming active low
	HAL_GPIO_WritePin(outputEnable.name, outputEnable.number, GPIO_PIN_RESET); //active low
	HAL_GPIO_WritePin(chipEnable.name, chipEnable.number, GPIO_PIN_RESET); //enabled


	for (int i = OUTPUTLENGTH-1; i >= 0; i--) {
		 bit =  HAL_GPIO_ReadPin(outputPins[i].name, outputPins[i].number);
		 data = (data << 1) + bit;
		 if (buff != NULL) { // converting to characters
			buff[OUTPUTLENGTH - i - 1] = bit + 48; //converting to ascii
		 }
	}

	buff[OUTPUTLENGTH] = '\0';
	HAL_GPIO_WritePin(chipEnable.name, chipEnable.number, GPIO_PIN_SET); //enabled
	return data;
}

/**
 * @brief Sets PB0-7 GPIO Directions (EEPROM Out Pins)
 * **/
void setPinDirection(GPIODIRECTION direction) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (direction == GPIOOUTPUT) {
		  /*Configure GPIO pin Output Level */
		  HAL_GPIO_WritePin(GPIOB, OUT_Pin|OUTB1_Pin|OUTB2_Pin|OUTB3_Pin|OUTB4_Pin|OUTB5_Pin
		                          |OUTB6_Pin|OUTB7_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pins : OUT_Pin OUTB1_Pin OUTB2_Pin OUTB3_Pin
		 * OUTB4_Pin OUTB5_Pin OUTB6_Pin OUTB7_Pin */
		GPIO_InitStruct.Pin = OUT_Pin|OUTB1_Pin|OUTB2_Pin|OUTB3_Pin
				|OUTB4_Pin|OUTB5_Pin|OUTB6_Pin|OUTB7_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	}

	else {
		  /*Configure GPIO pins : OUT_Pin OUTB1_Pin OUTB2_Pin OUTB3_Pin
		                           OUTB4_Pin OUTB5_Pin OUTB6_Pin OUTB7_Pin */
		  GPIO_InitStruct.Pin = OUT_Pin|OUTB1_Pin|OUTB2_Pin|OUTB3_Pin
		                          |OUTB4_Pin|OUTB5_Pin|OUTB6_Pin|OUTB7_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}



int writeData(uint16_t address, int8_t data) {
	uint8_t dataToShift = data;

	//PROGRAMMING

	setAddress(address);
	setPinDirection(GPIOOUTPUT);
	HAL_Delay(10);

	HAL_GPIO_WritePin(writeEnable.name, writeEnable.number, GPIO_PIN_SET); //not programming yet
	HAL_GPIO_WritePin(outputEnable.name, outputEnable.number, GPIO_PIN_SET); //active low
	HAL_GPIO_WritePin(chipEnable.name, chipEnable.number, GPIO_PIN_RESET); //Active low


	for (int i = 0; i < OUTPUTLENGTH; i++) {
		if (dataToShift & 1) {
			HAL_GPIO_WritePin(outputPins[i].name, outputPins[i].number, GPIO_PIN_SET);
		}

		else {
			HAL_GPIO_WritePin(outputPins[i].name, outputPins[i].number, GPIO_PIN_RESET);
		}

		dataToShift = dataToShift >> 1;
	}

	HAL_GPIO_TogglePin(writeEnable.name, writeEnable.number); //Active low
	//delay_microsecond(2);
	HAL_GPIO_TogglePin(writeEnable.name, writeEnable.number); //Active low
	HAL_Delay(20); //waiting for it to finish
	HAL_GPIO_WritePin(chipEnable.name, chipEnable.number, GPIO_PIN_SET); //disabling chip


}

/*
 * Converts the non null terminated number (max length 256) inside of buf to real number useful for UART
 * */
uint16_t convertToNumber(char* buf, int nullPos) {
	char terminated[256];
	for (int i = 0; i < nullPos; i++) {
		terminated[i] = buf[i];
	}
	terminated[nullPos] = '\0';
	return ((uint16_t) atoi(terminated));
}

void sendEEPROMValToLCD(char* binaryRepBuff, int8_t value, unsigned int line_x, unsigned int chr_x) {
	LCD_SetCursor(line_x, chr_x);
	char lcdBuff[17];
	int len = sprintf(lcdBuff, "%s x%X %d", binaryRepBuff, value, value);
	if (len <= 16) {
		LCD_Send_String(lcdBuff, STR_NOSLIDE);
	}

	else {
		sprintf(lcdBuff, "x%X %d", value, value);
		LCD_Send_String(lcdBuff, STR_NOSLIDE);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (status == PREBEGIN) {
		messagePrinted = 0;
		status = BEGINNING;
		mode = UNKNOWN;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	switch (status) {
		case PREBEGIN:
		case BEGINNING:
			if(mode == UNKNOWN && convertToNumber(&uartOpReceiveBuff, 1) == SELECTREAD) {
				mode = READ;
				messagePrinted = 0;
				HAL_UART_Receive_IT(&huart2, uartAddrReceiveBuff, 5);

			}

			else if (mode == UNKNOWN && convertToNumber(&uartOpReceiveBuff, 1) == SELECTWRITE) {
				mode = WRITE;
				messagePrinted = 0;
				HAL_UART_Receive_IT(&huart2, uartAddrReceiveBuff, 5);
			}

			status = GETADDRESS;
			break;


		case GETADDRESS:
			selectedAddr = convertToNumber(uartAddrReceiveBuff, 6);
			if (mode == READ) {
				messagePrinted = 0;
				status = PERFORM;
			}

			else if (mode == WRITE) {
				status = GETINPUTDATA;
				messagePrinted = 0;
				HAL_UART_Receive_IT(&huart2, uartInputReceiveBuff, 3);

			}

			break;
		case GETINPUTDATA:
			messagePrinted = 0;
			selectedInput = convertToNumber(uartInputReceiveBuff, 4);
			status = PERFORM;
			break;
		case PERFORM:

			break;

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

	char buff[255];
	int len = 0;
	int readData = 0;



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
  MX_TIM1_Init();
  MX_SPI4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(chipEnable.name, chipEnable.number, GPIO_PIN_SET); //active low disabling


  LCD_i2cDeviceCheck();
  LCD_Init();
  LCD_BackLight(LCD_BL_ON);
  LCD_SetCursor(1,1);


//  writeData(0, 1);
//  readData = readOutput(dataOutBuff, 0);
//  HAL_Delay(1000);
//  LCD_Send_String(dataOutBuff, STR_NOSLIDE); //Example of sending string value.
//  LCD_SetCursor(2,1);
//  writeData(1, 0xAA);
//  readData = readOutput(dataOutBuff, 1);
//  HAL_Delay(1000);
//  LCD_Send_String(dataOutBuff, STR_NOSLIDE); //Example of sending string value.
//  HAL_GPIO_WritePin(chipEnable.name, chipEnable.number, GPIO_PIN_SET); //active low disabling




  //LCD_Clear();
  //LCD_Send_String("LCD TEST", STR_SLIDE); //Example of sending string value.
  //HAL_Delay(800);
  //HAL_Delay(20);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//len = sprintf(buff, "output from 1 %s is %d in binary\r", dataOutBuff, readData);
	//HAL_UART_Transmit(&huart2, ((uint8_t*)buff), len+1, 0xFFFF);
	  if (status == END) {
		  mode = UNKNOWN;
		  status = PREBEGIN;
	  }

	  if (status == PREBEGIN) {
		  HAL_UART_Receive_IT(&huart2, &uartOpReceiveBuff, 1);
		  if (startup) {
			  status = BEGINNING;
			  startup = 0;

		  }
	  }

	  if (!messagePrinted) {
		  switch (status) {
			  case BEGINNING:
				  len = sprintf(buff, "Please Select Op %d-Read %d-Write\r\n", SELECTREAD, SELECTWRITE);
				  HAL_UART_Transmit(&huart2, ((uint8_t*)buff), len+1, 0xFF);
				  messagePrinted = 1;
				  break;

			  case GETADDRESS:
				  len = sprintf(buff, "Please Enter Address               \r\n");
				  HAL_UART_Transmit_IT(&huart2, ((uint8_t*)buff), len+1);
				  messagePrinted = 1;
				  break;
			  case GETINPUTDATA:
				  len = sprintf(buff, "Please Enter value to write        \r\n");
				  HAL_UART_Transmit_IT(&huart2, ((uint8_t*)buff), len+1);
				  messagePrinted = 1;
				  break;
			  case PERFORM:
				  if(mode == WRITE) {
					  len = sprintf(buff, "Performing Write address:%u value:%u                      \r\n", selectedAddr, selectedInput);
				  }

				  else if (mode == READ) {
					  len = sprintf(buff, "Performing Read address:%u                     \r\n", selectedAddr, selectedInput);
				  }

				  if (mode == READ) {
					  int fetched = readOutput(dataOutBuff, selectedAddr);
					  LCD_Clear();
					  sendEEPROMValToLCD(dataOutBuff, fetched, 1, 1);

				  }

				  else if (mode == WRITE) {
					writeData(selectedAddr, selectedInput);
				  }
				  HAL_UART_Transmit_IT(&huart2, ((uint8_t*)buff), len+1);
				  messagePrinted = 1;
				  status = END;
				  mode = UNKNOWN;
				  break;
			  default:
				  messagePrinted = 1;


		  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  htim1.Init.Prescaler = 15;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, SRLATCHCLOCK_Pin|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_Pin|OUTB1_Pin|OUTB2_Pin|OUTB3_Pin 
                          |OUTB4_Pin|OUTB5_Pin|OUTB6_Pin|OUTB7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SRLATCHCLOCK_Pin PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = SRLATCHCLOCK_Pin|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_Pin OUTB1_Pin OUTB2_Pin OUTB3_Pin 
                           OUTB4_Pin OUTB5_Pin OUTB6_Pin OUTB7_Pin */
  GPIO_InitStruct.Pin = OUT_Pin|OUTB1_Pin|OUTB2_Pin|OUTB3_Pin 
                          |OUTB4_Pin|OUTB5_Pin|OUTB6_Pin|OUTB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
