/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
#include "i2c-lcd.h"
#include "stdio.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef myTime;
RTC_DateTypeDef myDate;
RTC_AlarmTypeDef myAlarm;

RTC_TimeTypeDef sunriseTime;
RTC_TimeTypeDef sunsetTime;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

int ITreason = 0;				// Interrupt reason. Variable not used in this version.
int *pITreason = &ITreason;		// Enables us to access variable ITreason in other source files.
uint8_t old_AB = 0;

int menue_state = 1;			// 4 possible states. Menu contains 4 lines on LCD display.
int state = 0;					// Sunrise, sunset or neither
int button_pressed = 0;

int sunset_timer = 0;
int sunrise_timer = 0;
int display_timer = 0;			// Minute display 

int red = 0;
int green = 0;
int blue = 0;
int FLbrightness = 0;

uint8_t  u8SampleButton = 0;
uint8_t  u8SampleLastButton = 0;

int8_t encoder_val = 0;
uint8_t  u8SampleEncA = 0;		// Variable not used.
uint8_t  u8SampleLastEncA = 0;	// Variable not used.
uint8_t  u8SampleEncB = 0;		// Variable not used.
uint8_t  u8SampleLastEncB = 0;	// Variable not used.

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

static void MX_TIM4_Init(int duty);
static void MX_TIM3_Init(int duty);
static void MX_TIM8_Init(int duty);
static void MX_RTC_Init(void);
void RTC_get_Time_and_Date(void);

static void MX_TIM2_Init(int brightness);
void set_FL(int brightness);				//set brightness of the FL light
void set_RGB(int red, int green, int blue); //set RGB value for led strip

//void menu_print_cursor (int linenumber);
void menu_print_text (void);
void menu_print_time (uint8_t HoursSunrise, uint8_t MinutesSunrise,uint8_t HoursSunset, uint8_t MinutesSunset);

void sunrise(void);			// Simulates sunrise
void sunset(void);			// Simulates sunset
// These functions are necessary because we avoid any sudden light changes
void LED_Dimm_Up(void);		// Turns on LED strip and FL to enable sunset simulation
void LED_Dimm_Down(void);	// Turns off LED strip and FL after sunrise simulation


/* Private user code ---------------------------------------------------------*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM4_Init(0);//all PWM (for LED and FL) initialized with 0 (all lights off)
	MX_TIM3_Init(0);
	MX_TIM8_Init(0);
	MX_TIM2_Init(0);
	MX_RTC_Init();

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	lcd_init();
	menu_print_text();
	menue_state = 1;

	//Set time, data and alarm
	//1) Set time
	myTime.Hours = 7;
	myTime.Minutes = 59;
	myTime.Seconds = 30;
	HAL_RTC_SetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
	//2) Set date
	myDate.Date = 6;
	myDate.Month = RTC_MONTH_DECEMBER;
	myDate.WeekDay = RTC_WEEKDAY_THURSDAY;
	myDate.Year = 18;
	HAL_RTC_SetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
	//3)Set alarm

	/* Infinite loop */
	//default times
	sunriseTime.Hours = 8;
	sunriseTime.Minutes = 0;

	sunsetTime.Hours = 18;
	sunsetTime.Minutes = 0;

	/* Infinite loop */
	while (1)
	{

		RTC_get_Time_and_Date();
		//zeit anzeigen
		if(display_timer!=myTime.Minutes){
			menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			display_timer=myTime.Minutes;
		}




		if(sunriseTime.Hours == myTime.Hours){			// Check if the current time equals the set sunrise time.
			if(sunriseTime.Minutes == myTime.Minutes){
				state=1;//sunrise state
			}
		}
		if(sunriseTime.Hours+1 == myTime.Hours){		// Check if the current time equals the quick dim-down time.
			if(sunriseTime.Minutes == myTime.Minutes){
				state=3;
			}
		}

		if(sunsetTime.Hours == myTime.Hours){			// Check if the current time equals the set sunset time.
			if(sunsetTime.Minutes == myTime.Minutes){
				state=2;
			}
		}
		if(sunsetTime.Hours-1 == myTime.Hours){			// Check if the current time equals the quick turn-on time.
			if(sunsetTime.Minutes == myTime.Minutes){
				state=4;
			}
		}

		switch (state) {	 		//taeglicher zyklus mit stateevent
		case 1:
			sunrise();
			break;
		case 2:
			sunset();
			break;
		case 3:
			LED_Dimm_Down();
			break;
		case 4:
			LED_Dimm_Up();
			break;
		default:

			break;
		}


		//taster auslesen (positive flankentriggerung)
		button_pressed=0;
		u8SampleButton = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		if((u8SampleButton != 0) && (u8SampleLastButton == 0)){
		  button_pressed=1;
		}
		u8SampleLastButton = u8SampleButton;


		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)==1)//wenn taste gedrueckt ledring an
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		}
		else//sonst aus
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		}


		encoder_val=read_encoder();

		//menue state machine
		switch (menue_state){ 			//Change menu state if button is pushed
		case 1://running...
			if(button_pressed == GPIO_PIN_SET){
				menue_state = 2;
				menu_print_cursor(2);
			}
			break;
		case 2://TIME
			if(button_pressed  == GPIO_PIN_SET){
				menue_state = 3;
				menu_print_cursor(3);
			}
			if(encoder_val == 1)
			{
				myTime.Minutes = myTime.Minutes+1;
				if(myTime.Minutes >= 60)
				{
					myTime.Hours = myTime.Hours+1;
					myTime.Minutes = 0;
					if(myTime.Hours >= 24)
					{
						myTime.Hours = 0;
					}
				}
				HAL_RTC_SetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
				menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			}
			if(encoder_val == -1)
			{
				myTime.Minutes = myTime.Minutes-1;
					if(myTime.Minutes >= 60)
					{
						myTime.Hours = myTime.Hours-1;
						myTime.Minutes = 59;
						if(myTime.Hours >= 24)
						{
							myTime.Hours = 23;
						}
					}
					HAL_RTC_SetTime(&hrtc, &myTime, RTC_FORMAT_BIN);//myTime in RTC schreiben
					menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			}
			break;
		case 3://SUNRISE
			if(button_pressed  == GPIO_PIN_SET){
				menue_state = 4;
				menu_print_cursor(4);
			}
			if(encoder_val == 1)
			{
				sunriseTime.Minutes = sunriseTime.Minutes+1;
				if(sunriseTime.Minutes >= 60)
				{
					sunriseTime.Hours = sunriseTime.Hours+1;
					sunriseTime.Minutes = 0;
					if(sunriseTime.Hours >= 24)
					{
						sunriseTime.Hours = 0;
					}
				}
				menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			}
			if(encoder_val == -1)
			{
				sunriseTime.Minutes = sunriseTime.Minutes-1;
					if(sunriseTime.Minutes >= 60)
					{
						sunriseTime.Hours = sunriseTime.Hours-1;
						sunriseTime.Minutes = 59;
						if(sunriseTime.Hours >= 24)
						{
							sunriseTime.Hours = 23;
						}
					}
					menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			}
			break;
		case 4://SUNSET
			if(button_pressed  == GPIO_PIN_SET){
				menue_state = 1;
				menu_print_cursor(1);
			}
			if(encoder_val == 1)
			{
				sunsetTime.Minutes = sunsetTime.Minutes+1;
				if(sunsetTime.Minutes >= 60)
				{
					sunsetTime.Hours = sunsetTime.Hours+1;
					sunsetTime.Minutes = 0;
					if(sunsetTime.Hours >= 24)
					{
						sunsetTime.Hours = 0;
					}
				}
				menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			}
			if(encoder_val == -1)
			{
				sunsetTime.Minutes = sunsetTime.Minutes-1;
					if(sunsetTime.Minutes >= 60)
					{
						sunsetTime.Hours = sunsetTime.Hours-1;
						sunsetTime.Minutes = 59;
						if(sunsetTime.Hours >= 24)
						{
							sunsetTime.Hours = 23;
						}
					}
					menu_print_time(sunriseTime.Hours,sunriseTime.Minutes,sunsetTime.Hours,sunsetTime.Minutes);
			}
			break;
		}
		HAL_Delay(10);//entprellen des tasters
	}
}

/**
 * @brief Function that adjusts colour of LED strip
 * @param Duty cycles of red, green and blue LEDs, respectively
 * @retval None
 */
void set_RGB(int red, int green, int blue)
{
	MX_TIM4_Init(red);
	MX_TIM3_Init(green);
	MX_TIM8_Init(blue);
}

/**
 * @brief Function that controls LED and fluorescent lamp to simulate sunrise
 * @param None
 * @retval None
 */
void sunrise(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_SET);//relais fuer FL ein
	RTC_get_Time_and_Date();
	if((red<=1000) && (green<=1000) && (blue<=1000) && (FLbrightness<=1000))//nur dimmen wenn max helligkeit noch nicht erreicht
	{
		if(sunrise_timer!=myTime.Seconds)//jede sekunde
		{
			red=red+25;
			green=green+8;
			blue=blue+1;
			FLbrightness=FLbrightness+25;
			set_FL(FLbrightness);
			set_RGB(red,green,blue);
			sunrise_timer=myTime.Seconds;
		}
	}
}

/**
 * @brief Function that controls LED and fluorescent lamp to simulate sunset
 * @param None
 * @retval None
 */
void sunset(void)
{
	RTC_get_Time_and_Date();
	if((red>=25) && (green>=8) && (blue>=1) && (FLbrightness>=25))//nur dimmen wenn die Led noch leuchten
	{
		if(sunrise_timer!=myTime.Seconds)//jede sekunde
		{
			red=red-25;
			green=green-8;
			blue=blue-1;
			FLbrightness=FLbrightness-25;
			set_FL(FLbrightness);
			set_RGB(red,green,blue);
			sunrise_timer=myTime.Seconds;
		}
	}
	else //unterlauf der variablen fuer die helligkeit abfangen
	{
		if(sunrise_timer!=myTime.Seconds)//jede sekunde
		{
			red=0;
			green=0;
			blue=0;
			FLbrightness=0;
			set_FL(FLbrightness);
			set_RGB(red,green,blue);
			sunrise_timer=myTime.Seconds;
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_RESET);
	}
}

/**
 * @brief Function that gradually turns up the LED strip
 * @param None
 * @retval None
 */
void LED_Dimm_Up(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_SET);
	RTC_get_Time_and_Date();
	if((red<=1000) && (green<=1000) && (blue<=1000))//nur dimmen wenn max helligkeit noch nicht erreicht
	{
		if(sunrise_timer!=myTime.Seconds)//jede sekunde
		{
			red=red+50;
			green=green+16;
			blue=blue+2;
			FLbrightness=FLbrightness+50;
			set_FL(FLbrightness);
			set_RGB(red,green,blue);
			sunrise_timer=myTime.Seconds;
		}
	}
}

/**
 * @brief Function that gradually dims the LED strip
 * @param None
 * @retval None
 */
void LED_Dimm_Down(void)
{
	RTC_get_Time_and_Date();
	if((red>=50) && (green>=16) && (blue>=2) && (FLbrightness>=25)) //nur dimmen wenn die Led noch leuchten
	{
		if(sunrise_timer!=myTime.Seconds)//jede sekunde
		{
			red=red-50;
			green=green-16;
			blue=blue-2;
			FLbrightness=FLbrightness-50;
			set_FL(FLbrightness);
			set_RGB(red,green,blue);
			sunrise_timer=myTime.Seconds;
		}
	}
	else	//unterlauf der variablen fuer die helligkeit abfangen
	{
		if(sunrise_timer!=myTime.Seconds)//jede sekunde
		{
			red=0;
			green=0;
			blue=0;
			FLbrightness=0;
			set_FL(FLbrightness);
			set_RGB(red,green,blue);
			sunrise_timer=myTime.Seconds;
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_RESET);
	}
}


/**
 * @brief Real Time Clock update time and date
 * @retval None
 */
void RTC_get_Time_and_Date(void)
{
	//1)Get time
	HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
	//2)Get data
	HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
}

/**
 * @brief Read new encoder state and decide turning direction
 * @param None
 * @retval 8bit unsigned integer. 0 for no rotation, -1 for CCW rotation, 1 for CW rotation.
 */
int8_t read_encoder(void)
{
	static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // LUT for rotation direction
	uint8_t encAB = 0x00;
	old_AB = old_AB << 2;                   	//remember previous state on the positions bit no. 2 and 3 (bit 0 is first element)
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET){ 	// If channel A is high level, bit no. 1 is set
		encAB = encAB | (0x01 << 1);
	}
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET){	// If channel B is high level, bit no. 0 is set
		encAB = encAB | (0x01 << 0);
	}
	old_AB = (old_AB | ( encAB & 0x03 ));  		//add current state without deleting previous state
	return ( enc_states[( old_AB & 0x0f )]);	//Pick out the corresponding element in the LUT
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
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
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
}

/**
 * @brief Funktion soll den Pfeil auf dem display ausgeben an der Position di vom Rotary
 * gewuenscht wird. Dazu braucht sie die Postion (linenumber). Sie giebt je nach
 * zeilenumer 1, 2 oder  3 den Pfeil im 2 und 3 Feld  auf der 1. 3. oder 4. Zeile aus.
 * @param Gewuenschte Position
 * @retval None
 */
/*
void menu_print_cursor (int linenumber)
{
	switch(linenumber){
	case 1:
		cursor_jumpto_r_c (2, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (3, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (4, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (1, 2);
		lcd_send_string("->");
		break;
	case 2:
		cursor_jumpto_r_c (1, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (3, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (4, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (2, 2);
		lcd_send_string("->");
		break;
	case 3:
		cursor_jumpto_r_c (1, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (2, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (4, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (3, 2);
		lcd_send_string("->");
		break;
	case 4:
		cursor_jumpto_r_c (1, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (2, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (3, 2);
		delete_some_chars(2);
		cursor_jumpto_r_c (4, 2);
		lcd_send_string("->");
		break;
	}
}
*/

/**
 * @brief Gibt den Text aus der aendert sich nicht daher braucht sie keine Uebergabewerte
 * Time ist die Aktuelle Uhrzeit
 * Start_AM soll der Text sein fuer den Start des Sonnenaufgang
 * Start_PM soll der Text sein fuer den Start des Sonnenuntergang
 * @param None
 * @retval None
 */

void menu_print_text (void)
{
	cursor_jumpto_r_c(1, 5);
	lcd_send_string("running...");

	cursor_jumpto_r_c(2, 5);
	lcd_send_string("TIME");

	cursor_jumpto_r_c(3, 5);
	lcd_send_string("SUNRISE");

	cursor_jumpto_r_c(4, 5);
	lcd_send_string("SUNSET");
}

/**
 * @brief Mit dieser Funktion soll die Aktuelle Uhrzeit, die Startzeit fuer den Aufgang und Untergang
 * ausgegeben werden.
 * @param
 *  -Aktuelle stunde
 *  -Aktuelle Minute
 *  -Eingestellte Startzeit Morgen (time_am)
 *  -Eingestellte Startzeit Abend (time_pm)
 *  @retval None
 */

void menu_print_time (uint8_t HoursSunrise, uint8_t MinutesSunrise,uint8_t HoursSunset, uint8_t MinutesSunset)
{
	char sunrise[5];
	sprintf(sunrise, "%02d:%02d",HoursSunrise,MinutesSunrise);
	cursor_jumpto_r_c(3, 15);
	//delete_some_chars(5);
	lcd_send_string(&sunrise);

	char sunset[5];
	sprintf(sunset, "%02d:%02d",HoursSunset,MinutesSunset);
	cursor_jumpto_r_c(4, 15);
	//delete_some_chars(5);
	lcd_send_string(&sunset);


	RTC_get_Time_and_Date();

	char realtime[5];
	sprintf(realtime, "%02d:%02d",myTime.Hours,myTime.Minutes);
	cursor_jumpto_r_c(2, 15);
	//delete_some_chars(5);
	lcd_send_string(&realtime);
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	RTC_AlarmTypeDef sAlarm = {0};

	/**Initialize RTC Only
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

	/**Initialize RTC and set the Time and Date
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
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x18;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/**Enable the Alarm A
	 */
	sAlarm.AlarmTime.Hours = 0x0;
	sAlarm.AlarmTime.Minutes = 0x0;
	sAlarm.AlarmTime.Seconds = 0x0;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}


}

/**
 * @brief TIM2 Initialization Function
 * @param Duty cycle in percentage (value between 0 and 100)
 * @retval None
 */
static void MX_TIM2_Init(int brightness)
{

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = brightness;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param Duty cycle in percentage (value between 0 and 100)
 * @retval None
 */
static void MX_TIM3_Init(int duty)
{

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param Duty cycle in percentage (value between 0 and 100)
 * @retval None
 */
static void MX_TIM4_Init(int duty)
{

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM8 Initialization Function
 * @param Duty cycle in percentage (value between 0 and 100)
 * @retval None
 */
static void MX_TIM8_Init(int duty)
{

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 84;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1000;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
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
		Error_Handler();
	}
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
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC1 : Push button in*/
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA6 */
	GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/**
 * @brief Adjust brightness of fluorescent lamp
 * @param Duty cycle in percentage (value between 0 and 100)
 * @retval None
 */
void set_FL(int brightness)
{
	MX_TIM2_Init(brightness);
}


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
