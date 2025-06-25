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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306/ssd1306.h"

#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// The chip doesn't implement native floating-point operations, hence no floats are used in this project's data types

typedef enum : uint8_t {
    INST, CALC, PEAK, GRAPH, MEM, STAT1, STAT2, COMM, DUMP, INFO,

    MENU_ITEMS,
} MenuPage;

typedef struct {
	uint16_t voltage;
	int16_t current;
	int32_t power;
} Measurement;

typedef struct {
	Measurement peak;
	uint32_t duration;
	int32_t energy;
	int32_t average_power;
} SessionData;

typedef struct {
	char line1[20];
	char line2[20];
	char line3[20];
} ScreenText;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CRLF "\r\n"

#define MAX_VOLTAGE 20000 // mV
#define MAX_CURRENT 5000 // mA
#define MAX_POWER ((MAX_VOLTAGE*MAX_CURRENT)/1000) // mW

#define TIMER_PERIOD 125000 // us

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t rotary_state;
static uint8_t rotary_counter;
static uint8_t button_state;

static uint8_t button_down;

// This needs to be volatile because it points to a non-volatile memory location
volatile struct {

	struct {
		MenuPage cur;
		MenuPage prev;
	} menu;

	SessionData last_session;

	bool rt_comm;

}* persistent_data = (void*)DATA_EEPROM_BASE;

uint32_t timestamp = 0;

Measurement instant = {0};

SessionData ram_session = {0};

struct {
	uint8_t buffer[SCREEN_WIDTH];
	uint8_t front;
} plot = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Fonctions projet CREATE

// Draws INST menu
void CREATE_InstMenu(ScreenText* text_buffers) {

	sprintf(text_buffers->line1, "INST VALS");
	sprintf(text_buffers->line2, "%05u mV  %5d mA", instant.voltage, instant.current);
	sprintf(text_buffers->line3, "P= %12ld mW", instant.power);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Draws CALC menu
void CREATE_CalcMenu(ScreenText* text_buffers) {

	// Reset session on button press
	if (button_state) {
		timestamp = 0;
		ram_session.energy = 0;
		ram_session.average_power = 0;
	}

	sprintf(text_buffers->line1, "CALC VALS");
	sprintf(text_buffers->line2, "E= %11ld uWh", ram_session.energy);
	sprintf(text_buffers->line3, "P_avg= %8ld mW", ram_session.average_power);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Draws PEAK menu
void CREATE_PeakMenu(ScreenText* text_buffers) {

	if (button_state) ram_session.peak = (Measurement){0}; // Reset peak values

	sprintf(text_buffers->line1, "PEAK VALS");
	sprintf(text_buffers->line2, "%05u mV  %5d mA", ram_session.peak.voltage, ram_session.peak.current);
	sprintf(text_buffers->line3, "P_pk= %9ld mW", ram_session.peak.power);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Draws GRAPH menu
void CREATE_GraphMenu(ScreenText* text_buffers) {

	ssd1306_Fill(Black); // Clear screen

	for (uint8_t i = 0; i < SCREEN_WIDTH; ++i) {
		uint8_t p = (i + plot.front+1)%SCREEN_WIDTH; // Walk the plot buffer starting from plot.front+1 and wrapping around up to plot.front

		// Draw pixel at appropriate height and position
		ssd1306_DrawPixel(i, SCREEN_HEIGHT-1 - plot.buffer[p], White);
	}

	// Add instant power readout
	sprintf(text_buffers->line3, "%07ld mW", instant.power);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Draws MEM menu
void CREATE_MemMenu(ScreenText* text_buffers) {

	sprintf(text_buffers->line1, "MEMORY");
	sprintf(text_buffers->line2, "Push to save!");
	sprintf(text_buffers->line3, "                   "); // To draw over SAVED message
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);

	ssd1306_UpdateScreen();


	if (!button_down) return; // Early return if button not pressed

	persistent_data->last_session = ram_session;
	FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

	// Indicate that the write is finished
	sprintf(text_buffers->line3, "       SAVED       ");
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Draws STAT1 menu
void CREATE_Stat1Menu(ScreenText* text_buffers) {

	sprintf(text_buffers->line1, "STATS SAVED");
	sprintf(text_buffers->line2, "%05u mV  %5d mA", persistent_data->last_session.peak.voltage, persistent_data->last_session.peak.current);
	sprintf(text_buffers->line3, "P_pk= %9ld mW", persistent_data->last_session.peak.power);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);

}

// Draws STAT2 menu
void CREATE_Stat2Menu(ScreenText* text_buffers) {

	sprintf(text_buffers->line1, "Duration= %6lu s", persistent_data->last_session.duration);
	sprintf(text_buffers->line2, "E= %11ld uWh", persistent_data->last_session.energy);
	sprintf(text_buffers->line3, "P_avg= %8ld mW", persistent_data->last_session.average_power);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);

}

// Draws COMM menu
void CREATE_CommMenu(ScreenText* text_buffers) {

	sprintf(text_buffers->line1, "RT COMMUNICATION");
	sprintf(text_buffers->line2, "Serial: %s", persistent_data->rt_comm ? "ON " : "OFF");
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);

	// Runs every timer cycle, continuous feeding intended for remote machine consumption
	if (persistent_data->rt_comm) {

		char msg[130];
		int size = sprintf(msg,
		                   "INST: %05u mV, %5d mA, %9ld mW" CRLF
		                   "CALC: %10ld uWh, %8ld mW" CRLF
		                   "PEAK: %05u mV, %5d mA, %9ld mW" CRLF CRLF,
		                   instant.voltage, instant.current, instant.power,
		                   ram_session.energy, ram_session.average_power,
		                   ram_session.peak.voltage, ram_session.peak.current, ram_session.peak.power);

		HAL_UART_Transmit(&huart1, (uint8_t*)msg, size, 10); // Transmit with 10ms timeout < TIMER_PERIOD budget
	}

	// Toggle real-time serial communication
	if (button_down) persistent_data->rt_comm = !persistent_data->rt_comm;
}

// Draws DUMP menu
void CREATE_DumpMenu(ScreenText* text_buffers) {

	sprintf(text_buffers->line1, "DUMP SESSION");
	sprintf(text_buffers->line2, "Push to UART-dump!");
	sprintf(text_buffers->line3, "                   ");
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);

	ssd1306_UpdateScreen();


	if (!button_down) return;
	// One-time serial communication

	char msg[130];
	int size = sprintf(msg,
	                   "Session: %6lu s" CRLF
	                   "CALC: %10ld uWh, %8ld mW" CRLF
	                   "PEAK: %05u mV, %5d mA, %9ld mW" CRLF CRLF,
	                   persistent_data->last_session.duration,
	                   persistent_data->last_session.energy, persistent_data->last_session.average_power,
	                   persistent_data->last_session.peak.voltage, persistent_data->last_session.peak.current, persistent_data->last_session.peak.power);

	HAL_UART_Transmit(&huart1, (uint8_t*)msg, size, 10);

	// Reset saved session
	persistent_data->last_session = (SessionData){0};

	sprintf(text_buffers->line3, "      DUMPED       ");
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Draws INFO menu
void CREATE_InfoMenu(ScreenText* text_buffers) {

	// Additional information, mainly useful for debugging but I thought it best to keep it

	sprintf(text_buffers->line1, "INFORMATION");
	sprintf(text_buffers->line2, "TS= %14lu", timestamp);
	sprintf(text_buffers->line3, "Duration= %6lu s", ram_session.duration);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(text_buffers->line1, Font_7x10, White);
	ssd1306_SetCursor(0, 11);
	ssd1306_WriteString(text_buffers->line2, Font_7x10, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(text_buffers->line3, Font_7x10, White);
}

// Dispatches the screen menu draws and also fetches and calculates the necessary information.
// This is where most of the work happens.
void CREATE_ScreenFrame(void) {

	// Unlock EEPROM region for subsequent writes
	HAL_FLASHEx_DATAEEPROM_Unlock();
	FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE); // Wait for any unfinished operation (shouldn't be much)

	// Take measurements, used to display INST page
	instant.voltage = ((int16_t)(Get_ADC_Value(ADC_CHANNEL_4) - 110) * 8); // mV
	instant.current = ((int16_t)(Get_ADC_Value(ADC_CHANNEL_3) - 2152) * 41)/10; // mA
	instant.power = (int32_t)(instant.voltage * instant.current)/1000; // mW

	// Calculate energy, for CALC page
	ram_session.energy += ((int64_t)instant.power * TIMER_PERIOD)/3600000; // uWh
	ram_session.average_power = (((int64_t)ram_session.energy * 3600000)/(timestamp * TIMER_PERIOD)); // mW
	ram_session.duration = (timestamp * TIMER_PERIOD) / 1000000; // s

	// Determine peaks, for PEAK page
	if (instant.voltage > ram_session.peak.voltage) ram_session.peak.voltage = instant.voltage;
	if (abs(instant.current) >= abs(ram_session.peak.current)) ram_session.peak.current = instant.current;
	if (labs(instant.power) >= labs(ram_session.peak.power)) ram_session.peak.power = instant.power;

	// Update plot front, for GRAPH page
	plot.front = (plot.front + 1) % SCREEN_WIDTH;
	plot.buffer[plot.front] = (((instant.power + MAX_POWER) * (SCREEN_HEIGHT-1))/(2*MAX_POWER)) % SCREEN_HEIGHT;

	// Setup/clear screen text buffers
	ScreenText text_buffers = {0};

	// Only fully clear the screen if changing menu page
	if (persistent_data->menu.cur != persistent_data->menu.prev) ssd1306_Fill(Black);
	persistent_data->menu.prev = persistent_data->menu.cur;

	// Dispatch menu displays
	switch (persistent_data->menu.cur) {

		case INST:  CREATE_InstMenu(&text_buffers);  break;
		case CALC:  CREATE_CalcMenu(&text_buffers);  break;
		case PEAK:  CREATE_PeakMenu(&text_buffers);  break;
		case GRAPH: CREATE_GraphMenu(&text_buffers); break;
		case MEM:   CREATE_MemMenu(&text_buffers);   break;
		case STAT1: CREATE_Stat1Menu(&text_buffers); break;
		case STAT2: CREATE_Stat2Menu(&text_buffers); break;
		case COMM:  CREATE_CommMenu(&text_buffers);  break;
		case DUMP:  CREATE_DumpMenu(&text_buffers);  break;
		case INFO:  CREATE_InfoMenu(&text_buffers);  break;
		default:                                     break;
	}

	// Actually apply screen changes
	ssd1306_UpdateScreen();

	FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE); // Wait for unfinished ops done in the function
	HAL_FLASHEx_DATAEEPROM_Lock(); // Lock the EEPROM again, prevent writes
}


/**
  * @brief  Interrupt handler for TIM6 timer
  * @note	This function is called when the timer is reloaded
  *         It reads ADC values from volateg and current inputs and updates screen information
  */
void Timer_Interrupt_Handler(void) {

	++timestamp; // Ever-increasing time counter

	CREATE_ScreenFrame();

	button_down = false; // Reset edge variable so that it can be true only during a frame
}

/**
  * @brief  Interrupt handler for User Button GPIO
  * @note	This function is called when a rising edge is detected on User Button input pin
  */
void User_Button_Interrupt_Handler(void) {

	button_state = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

	// Use a second variable to detect button edge
	button_down = button_state; // Needs to be reset regularly elsewhere
}

/**
  * @brief  Interrupt handler for Rotary Encoder Channel A
  * @note	This function is called when a rising edge is detected on channel A of the rotary encoder
  */
void Rotary_Encoder_Interrupt_Handler(void) {

	static int8_t rotary_buffer = 0;
	/* Check for rotary encoder turned clockwise or counter-clockwise */
	HAL_Delay(5);
	uint8_t rotary_new = HAL_GPIO_ReadPin(ROT_CHA_GPIO_Port, ROT_CHA_Pin) << 1;
	rotary_new += HAL_GPIO_ReadPin(ROT_CHB_GPIO_Port, ROT_CHB_Pin);
	if (rotary_new != rotary_state) {
		if (((rotary_state == 0b00) && (rotary_new == 0b10)) || ((rotary_state == 0b10) && (rotary_new == 0b11)) ||
				((rotary_state == 0b11) && (rotary_new == 0b01)) || ((rotary_state == 0b01) && (rotary_new == 0b00))) {
			rotary_buffer ++;
		}
		if (((rotary_state == 0b00) && (rotary_new == 0b01)) || ((rotary_state == 0b01) && (rotary_new == 0b11)) ||
				((rotary_state == 0b11) && (rotary_new == 0b10)) || ((rotary_state == 0b10) && (rotary_new == 0b00))) {
			rotary_buffer --;
		}

		rotary_state = rotary_new;

		// Menu selection is stored in non-volatile memory hence needs to unlock EEPROM
		HAL_FLASHEx_DATAEEPROM_Unlock();
		FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
		if (rotary_buffer > 3) {
			rotary_counter ++;
			rotary_buffer = 0;

			// Increase selection on clockwise rotation
			persistent_data->menu.cur = (persistent_data->menu.cur + 1) % MENU_ITEMS;
		}

		if (rotary_buffer < -3) {
			rotary_counter --;
			rotary_buffer = 0;

			// Decrease selection on clockwise rotation
			persistent_data->menu.cur = (persistent_data->menu.cur + MENU_ITEMS-1) % MENU_ITEMS;
		}

		FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
		// EEPROM Lock is not reentrant hence we can't re-lock it here, because it potentially will prevent accesses in ScreenFrame routine
	}
}

/**
  * @brief  Start a conversion and return the value converted.
  * @note   This function waits for the end of conversion in a blocking way
  * @param  hadc ADC handle
  * @param  adc_channel Channel macro such as ADC_CHANNEL_0, ADC_CHANNEL_1, etc.
  * @retval Channel converted value
  */
uint32_t Get_ADC_Value(uint32_t adc_channel)
{
	/* Disable all previous channel configuration */
	hadc.Instance->CHSELR = 0;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = adc_channel;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK)
	{
		Error_Handler();
	}
	return HAL_ADC_GetValue(&hadc);
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
  MX_I2C1_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  rotary_state = HAL_GPIO_ReadPin(ROT_CHA_GPIO_Port, ROT_CHA_Pin) << 1;
  rotary_state += HAL_GPIO_ReadPin(ROT_CHB_GPIO_Port, ROT_CHB_Pin);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00B07CB4;
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
  htim6.Init.Prescaler = 39999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_BUTTON_Pin ROT_CHB_Pin ROT_CHA_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin|ROT_CHB_Pin|ROT_CHA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
