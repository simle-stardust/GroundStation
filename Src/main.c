/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "string.h"

/* USER CODE BEGIN Includes */
#include "aes.h"

#define FREQUENCY  868100000lu // in Mhz! (868.1)

#define DEV_ADDR                              (uint32_t)0x260112BA
// network session key
static const uint8_t NwkSKey[16] =
{ 0x47, 0xB8, 0xCE, 0xE5, 0xA2, 0x82, 0xBC, 0xB4, 0x73, 0xA0, 0xC1, 0xEF, 0x5C,
		0xF3, 0x01, 0x6A };

// application session key
static const uint8_t ArtSKey[16] =
{ 0x4F, 0xE9, 0xA8, 0x61, 0x02, 0x33, 0xA6, 0xA4, 0x7C, 0x87, 0x0D, 0xC4, 0x9C,
		0x10, 0x37, 0xF1 };

static const uint8_t dupa[46] =
{ '\r', '\n', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-',
		'-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-',
		'-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-',
		'\r', '\n' };

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t receivedUART;
volatile uint8_t requestedState = '0';
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM11_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void writeLoraRegister(uint8_t address, uint8_t data);
uint8_t readLoraRegister(uint8_t address);
uint8_t clearBit(uint8_t value, uint8_t bit);
uint8_t setBit(uint8_t value, uint8_t bit);
void Lora_Init();
uint8_t LoraReceive();
uint8_t LoraWANParseDN(uint8_t* data, uint8_t len);
static void LoraWANTransmitByte(uint8_t* data, uint8_t size);
void LoraTransmitByte(uint8_t *data, uint8_t size);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	requestedState = (uint8_t) receivedUART;
	HAL_UART_Receive_IT(&huart2, &receivedUART, 1); // Ponowne w��czenie nas�uchiwania
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM11_Init();

	/* USER CODE BEGIN 2 */
	Lora_Init();
	uint8_t ReceivedNbOfBytes = 0;
	uint8_t FifoCurrRxAddr = 0;
	uint8_t UARTBufLen = 0;
	uint8_t ReceivedData[64];
	uint8_t ReceivedPayload[64];
	char UARTBuf[512];
	uint8_t DNframe[2] = {0x00, 0x01};
	HAL_UART_Receive_IT(&huart2, &receivedUART, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		memset(UARTBuf, 0x00, sizeof(UARTBuf));
		UARTBufLen = 0;

		ReceivedNbOfBytes = LoraReceive();
		if (ReceivedNbOfBytes)
		{
			FifoCurrRxAddr = readLoraRegister(0x10);
			writeLoraRegister(0x0D, FifoCurrRxAddr);
			memset(ReceivedData, 0, sizeof(ReceivedData));

			UARTBufLen = sprintf(UARTBuf, "==================Received packet==================\r\nRaw  data = ");
			HAL_UART_Transmit(&huart2, (uint8_t *) UARTBuf, UARTBufLen, 100);

			UARTBufLen = 0;
			for (int i = 0; i < ReceivedNbOfBytes; i++)
			{
				if (i < sizeof(ReceivedData))
				{
					ReceivedData[i] = readLoraRegister(0x00);
				}
				UARTBufLen += sprintf(&UARTBuf[UARTBufLen], "%02x ", ReceivedData[i]);
			}
			UARTBufLen += sprintf(&UARTBuf[UARTBufLen], "\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *) UARTBuf, UARTBufLen, 100);




			if (LoraWANParseDN(ReceivedData, ReceivedNbOfBytes) == 0x01)
			{
				// packet verified and ok
				UARTBufLen = sprintf(UARTBuf, "Packet verified successfully\r\nDecrypted payload = ");
				HAL_UART_Transmit(&huart2, (uint8_t *) UARTBuf, UARTBufLen, 100);

				memcpy(ReceivedPayload, &ReceivedData[9], ReceivedNbOfBytes - 13);
				UARTBufLen = 0;
				for (int i = 0; i < ReceivedNbOfBytes - 13; i++)
				{
					UARTBufLen += sprintf(&UARTBuf[UARTBufLen], "%02x ", ReceivedPayload[i]);
				}
				UARTBufLen += sprintf(&UARTBuf[UARTBufLen], "\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *) UARTBuf, UARTBufLen, 100);



				switch (requestedState)
				{
				case '1':
					DNframe[0] = 0x00;
					DNframe[1] = 0x02;
					break;
				case '2':
					DNframe[0] = 0x55;
					DNframe[1] = 0x55;
					break;
				default:
					DNframe[0] = 0x00;
					DNframe[1] = 0x01;
					break;
				}
				LoraWANTransmitByte(DNframe, 2);

			}

			HAL_Delay(1);

			/*
			 PktSnr = readLoraRegister(0x19);
			 PktSnr = 4 * PktSnr;
			 RSSI = readLoraRegister(0x1A);
			 if (PktSnr >= 0)
			 RSSI = RSSI - 139;
			 else
			 RSSI = RSSI - 139 + PktSnr * 0.25;
			 memset(PacketData, 0, sizeof(PacketData));
			 sprintf(PacketData, "\r\nPacket SNR = %d\n\rPacket RSSI = %d\r\n",
			 PktSnr, RSSI);
			 HAL_Delay(10);
			 HAL_UART_Transmit_IT(&huart2, (uint8_t *) PacketData,
			 sizeof(PacketData));
			 */
		}
		else
		{
			memset(UARTBuf, 0, sizeof(UARTBuf));
			UARTBufLen = sprintf(UARTBuf, "PACKET RECEPTION TIMEOUT\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *) UARTBuf, UARTBufLen, 100);
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			HAL_Delay(20);
			//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		}

		HAL_Delay(1);
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 9999;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 799;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED5_Pin | LED4_Pin | Lora_RX_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin | Lora_RST_Pin | Lora_TX_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED3_Pin | Lora_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED5_Pin LED4_Pin Lora_RX_Pin */
	GPIO_InitStruct.Pin = LED5_Pin | LED4_Pin | Lora_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin Lora_RST_Pin Lora_TX_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | Lora_RST_Pin | Lora_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED3_Pin Lora_CS_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | Lora_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void writeLoraRegister(uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(Lora_CS_GPIO_Port, Lora_CS_Pin, GPIO_PIN_RESET);
	address = setBit(address, 7);
	HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	HAL_GPIO_WritePin(Lora_CS_GPIO_Port, Lora_CS_Pin, GPIO_PIN_SET);
}

uint8_t readLoraRegister(uint8_t address)
{
	HAL_GPIO_WritePin(Lora_CS_GPIO_Port, Lora_CS_Pin, GPIO_PIN_RESET);
	uint8_t value = 0x00;
	address = clearBit(address, 7);
	HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
	HAL_SPI_Receive(&hspi1, &value, 1, 1000);
	HAL_GPIO_WritePin(Lora_CS_GPIO_Port, Lora_CS_Pin, GPIO_PIN_SET);
	;
	return value;
}

uint8_t setBit(uint8_t value, uint8_t bit)
{
	value |= (1 << bit);
	return value;
}

uint8_t clearBit(uint8_t value, uint8_t bit)
{
	value &= ~(1 << bit);
	return value;
}

void Lora_Init()
{
	uint64_t frf = ((uint64_t) FREQUENCY << 19) / 32000000;

	HAL_GPIO_WritePin(Lora_RST_GPIO_Port, Lora_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(Lora_RST_GPIO_Port, Lora_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);

	//Opmode - sleep -> lora -> standby
	writeLoraRegister(0x01, 0x00);
	writeLoraRegister(0x01, 0x80);
	writeLoraRegister(0x01, 0x81);

	// Max Payload length = 255
	writeLoraRegister(0x23, 255);

	// LNA gain = maximum, LNA boot = yes
	writeLoraRegister(0x0C, 0x23);

	// Bandwidth = 125kHz, CR 4/5, Explicit Header Mode, CRC enable, Low Data Rate Optimization on
	writeLoraRegister(0x1D, 0x0B);

	// Spreading Factor 12, LNA gain set by the internal AGC loop, RX timeout MSB = 0b11
	writeLoraRegister(0x1E, 0xC7);   //bylo C5
	// RX timeout LSB = 0xFF
	writeLoraRegister(0x1F, 0xFF);   //bylo A0

	// Sync Word = 0x34 (LoRaWAN sync word)
	writeLoraRegister(0x39, 0x34);

	//868.1 MHz - LoRaWAN channel 1
	writeLoraRegister(0x06, (uint8_t) (frf >> 16));
	writeLoraRegister(0x07, (uint8_t) (frf >> 8));
	writeLoraRegister(0x08, (uint8_t) (frf >> 0));

	// max output power, no PA_BOOST
	writeLoraRegister(0x09, 0x8F);
}

void LoraTransmitByte(uint8_t *data, uint8_t size)
{
	HAL_GPIO_WritePin(Lora_TX_GPIO_Port, Lora_TX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Lora_RX_GPIO_Port, Lora_RX_Pin, GPIO_PIN_RESET);
	writeLoraRegister(0x22, size);
	writeLoraRegister(0x11, 0xFF);
	writeLoraRegister(0x12, 0b11111111);
	writeLoraRegister(0x11, 0x00);
	uint8_t TxDoneInterrupt = readLoraRegister(0x12);
	writeLoraRegister(0x0D, 0x80);
	for (int i = 0; i < size; i++)
	{
		writeLoraRegister(0x00, *data);
		data++;
	}
	writeLoraRegister(0x01, 0x83);
	TxDoneInterrupt = readLoraRegister(0x12);
	TxDoneInterrupt &= (1 << 3);
	while ((TxDoneInterrupt == 0))
	{
		TxDoneInterrupt = readLoraRegister(0x12);
		TxDoneInterrupt &= (1 << 3);
	}
	readLoraRegister(0x01);
}

uint8_t LoraReceive()
{
	HAL_GPIO_WritePin(Lora_RX_GPIO_Port, Lora_RX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Lora_TX_GPIO_Port, Lora_TX_Pin, GPIO_PIN_RESET);
	writeLoraRegister(0x11, 0xFF);
	writeLoraRegister(0x12, 0b11111111);
	writeLoraRegister(0x11, 0x00);
	writeLoraRegister(0x0D, 0x00);
	writeLoraRegister(0x01, 0x86);
	int RxDoneInterrupt = readLoraRegister(0x12);
	int RxTimeoutInterrupt = RxDoneInterrupt;
	RxDoneInterrupt &= (1 << 6);
	RxTimeoutInterrupt &= (1 << 7);
	while (RxDoneInterrupt == 0 && RxTimeoutInterrupt == 0)
	{
		RxDoneInterrupt = readLoraRegister(0x12);
		RxTimeoutInterrupt = RxDoneInterrupt;
		RxDoneInterrupt &= (1 << 6);
		RxTimeoutInterrupt &= (1 << 7);
		HAL_Delay(1);
	}
	uint8_t NbofBytes = readLoraRegister(0x13);
	if (RxTimeoutInterrupt > 0)
		NbofBytes = 0;
	return NbofBytes;
}

uint8_t LoraWANParseDN(uint8_t* data, uint8_t len)
{
	uint8_t returncode = 0x00;
	uint16_t cnt = ((uint16_t) (data[7] << 8) + (data[6] & 0xFF));
	uint32_t addr = ((uint32_t) ((data[4] << 24) + (data[3] << 16)
			+ (data[2] << 8) + (data[1] & 0xFF)));
	if ((aes_verifyMic(NwkSKey, DEV_ADDR, cnt, 0, data, len - 4))
			&& (addr == DEV_ADDR ))
	{
		returncode = 0x01;
		aes_cipher(ArtSKey, DEV_ADDR, cnt, 0, &data[9], len - 13);
	}
	return returncode;
}

static void LoraWANTransmitByte(uint8_t* data, uint8_t size)
{
	uint8_t UplinkFrame[20];
	for (int i = 0; i < size; i++)
	{
		UplinkFrame[9 + i] = data[i];
	}
	UplinkFrame[0] = 0x60; //data unconfirmed up
	UplinkFrame[1] = (uint8_t) (DEV_ADDR & 0xFF);
	UplinkFrame[2] = (uint8_t) (DEV_ADDR >> 8);
	UplinkFrame[3] = (uint8_t) (DEV_ADDR >> 16);
	UplinkFrame[4] = (uint8_t) (DEV_ADDR >> 24);
	UplinkFrame[5] = 0x00; //FCtrl
	UplinkFrame[6] = 0x00; //Fcnt msb
	UplinkFrame[7] = 0x00; //Fcnt lsb
	UplinkFrame[8] = 0x01; //port
	aes_cipher(ArtSKey, DEV_ADDR, 0, /*up*/1, &UplinkFrame[9], size);
	aes_appendMic(NwkSKey, DEV_ADDR, 0, /*up*/1, UplinkFrame, size + 9);
	LoraTransmitByte(UplinkFrame, size + 13);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM11)
{ // Je�eli przerwanie pochodzi od timera 10
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
while (1)
{
}
/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
/* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
