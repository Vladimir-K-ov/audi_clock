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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include "ssd1306.h"
#include "fonts.h"
#include "Pic_bmp.h"
#include "button.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C1_EEPROM_ADDRESS      0x50   /* A0 = A1 = A2 = 0 */

#define EEPROM_ADDRESS_COR_X 0x00
#define EEPROM_ADDRESS_COR_Y 0x01

#define EEPROM_ADDRESS_invert_on_h 0x02
#define EEPROM_ADDRESS_invert_on_m 0x03
#define EEPROM_ADDRESS_invert_off_h 0x04
#define EEPROM_ADDRESS_invert_off_m 0x05

#define EEPROM_ADDRESS_Temp_cor 0x06

#define EEPROM_ADDRESS_Vbat_cor 0x07

#define EEPROM_ADDRESS_light_min 0x08

#define EEPROM_ADDRESS_light_max 0x09

#define EEPROM_ADDRESS_Rest_warning_time 0x0A

#define EEPROM_ADDRESS_RTC_cor 0x0B

#define EEPROM_ADDRESS_lcd_drive_screen 0x0C

#define EEPROM_ADDRESS_Ilm_start 0x0D

#define EEPROM_ADDRESS_ADC_channel_Temp_select 0x0E

#define EEPROM_ADDRESS_Engine_Hours_Set 0x0F

#define EEPROM_ADDRESS_RTC_Presc 0x10

#define EEPROM_ADDRESS_time_format_print 0x11

#define adc_lenght 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void lcd_main_screen_print(int8_t lcd_offset_start_x, int8_t lcd_offset_start_y, uint8_t lcd_time_format, int8_t Temp , int8_t Vbat_cor);
void lcd_drive_screen_print(int8_t lcd_offset_start_x, int8_t lcd_offset_start_y);
void lcd_info_screen(uint8_t Rest_warning_time);
void lcd_on_off(uint8_t LCD_Light_Current);
void Set_RTC_h_up();
void Set_RTC_m_up();
uint8_t menu_setup();

uint16_t Service_Rest_Warning(uint8_t time_trigger, int8_t lcd_offset_start_x, int8_t lcd_offset_start_y);
void Service_lcd_invert(uint8_t lcd_time_invert_on_h, uint8_t lcd_time_invert_on_m, uint8_t lcd_time_invert_off_h, uint8_t lcd_time_invert_off_m);
uint8_t Service_lcd_light(uint8_t Ilm_start, uint8_t light_min, uint8_t light_max);
void Service_Control_Bat_Voltage(int8_t Vbat_cor);
void Service_Check_Engine();
int8_t Service_Temp (uint8_t Temp_ADC_port, int8_t Temp_cor);
void Service_Engine_Hours();

uint16_t Get_ADC(uint8_t channel);

int16_t value_test(int16_t value_current, int16_t value_min, int16_t value_max, int16_t value_default);

int16_t EEPROM_Load(uint8_t EEPROM_ADDRESS, int16_t Data_min, int16_t Data_max, int16_t Data_default, uint8_t type_return);

int16_t Input_Data(int16_t Data_current, int16_t Data_min, int16_t Data_max, uint8_t EEPROM_save, uint16_t EEPROM_offset_save, char *Text_data);

int32_t interpol(int16_t xN, int16_t x1, int16_t x2, int32_t fx1, int32_t fx2);

void lcd_line_set (uint8_t poz_x, uint8_t poz_y, uint8_t line);

uint16_t expRunningAverage(float *filVal, uint16_t newVal);

uint8_t Time_on();

uint16_t Delay_int_button(uint16_t delay_ms);

void SSD1306_set_mid_pos(FontDef_t *font, int8_t offset_x, uint8_t poz_y, char *Text_data);

void EEPROM_Error_print (uint8_t event_error, uint8_t event_adress, uint8_t eeprom_adress);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const char *firmware_number = "01.45";

const uint16_t delay_message_info = 2100;

const uint16_t delay_blank_screen = 400;

const uint16_t screen_delay_after_set = 170;

const uint8_t lcd_font_7_line_0 = 12;
const uint8_t lcd_font_7_line_1 = lcd_font_7_line_0 + 15;
const uint8_t lcd_font_7_line_2 = lcd_font_7_line_1 + 15;
const uint8_t lcd_font_7_line_3 = 60;

const int8_t lcd_offset_max = 5;
const int8_t lcd_offset_min = -5;
const int8_t lcd_offset_default = 0;

// Множитель для ацп.
const uint8_t ADC_mul = 16;

const uint8_t lcd_light_min = 1;
const uint8_t lcd_light_max = 255;
const uint8_t Ilm_ADC_div_min = 0;
const uint8_t Ilm_ADC_div_max = 132;

const uint16_t delay_before_light_up = 700;
const uint16_t delay_start_logo = 400;
const uint8_t light_lcd_start = lcd_light_max;

const uint8_t Time_h_max = 23;
const uint8_t Time_m_max = 59;

const int8_t Temp_cor_max = 30;
const int8_t Temp_cor_min = -30;
const int8_t Temp_cor_default = 0;

const int8_t Vbat_cor_max = 30;
const int8_t Vbat_cor_min = -30;
const int8_t Vbat_cor_default = 0;

const uint8_t EEPROM_Read_Write_Delay = 30;

const float Battery_diode_cor = 0.8;
const float Battery_step_cor = 0.2;
const float Battery_div = 7.06;
const float Battery_voltage_min = 12.2;
const float Battery_voltage_max = 15.3;
const uint16_t Battery_error_max = 500;

const float Step_Vref_Volt_Div = 3.3 / 4096;

const float Filter_Ratio_Work = 0.1;

const uint8_t ADC_channel_Temp = 0;
const uint8_t ADC_channel_Vbat = 1;
const uint8_t ADC_channel_Ilm = 2;
const uint8_t ADC_channel_Temp_ext = 3;

const uint8_t ADC_channel_Temp_select_min = 0;
const uint8_t ADC_channel_Temp_select_max = 1;
const uint8_t ADC_channel_Temp_default = 0;

const uint8_t Rest_warning_time_min = 0;
const uint8_t Rest_warning_time_max = 255;

const uint8_t lcd_drive_screen_update_min = 0;
const uint8_t lcd_drive_screen_update_max = 255;
const uint16_t lcd_drive_screen_update_mult = 60000;

const int8_t RTC_cor_min = -127;
const int8_t RTC_cor_max = 0;

const int8_t RTC_Prescaler_min = -127;
const int8_t RTC_Prescaler_max = 0;

const uint8_t Poz_line_time_y = 10;

const uint8_t time_format_print_min = 0;
const uint8_t time_format_print_max = 2;

// Номер регистра бэкап для моточасов
const uint32_t Sec_BKP_reg_Low = 1;
const uint32_t Sec_BKP_reg_Hi = 2;

const uint8_t Engine_Hourse_Mul = 10;

char lcd_buff[25] = "";

uint8_t ADC_Ready = 0;

uint32_t RTC_Seconds = 0;

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

  /**  Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Light(lcd_light_min);
  SSD1306_Clear();

  // переменные минимальной и максимальной яркости
  uint8_t Ilm_start;
  uint8_t light_min;
  uint8_t light_max;

  // корректировка по осям
  int8_t lcd_offset_start_x;
  int8_t lcd_offset_start_y;

  // переменные инверсии подсветки
  uint8_t lcd_time_invert_on_h;
  uint8_t lcd_time_invert_on_m;
  uint8_t lcd_time_invert_off_h;
  uint8_t lcd_time_invert_off_m;

  // переменые коректировки температуры и батареи
  int8_t Temp_cor;
  int8_t Vbat_cor;

  // переменная о сообщении отдохнуть
  uint8_t Rest_warning_time;

  // коректировка часов
  int8_t RTC_cor;

  // время отображения времени в пути.
  uint8_t lcd_drive_screen_update;

  // коректировка RTC
  int8_t RTC_Presc;

  // формат вывода времени
  uint8_t time_format_print;

  uint8_t ADC_channel_Temp_select;

  // Загрузка переменных
  lcd_offset_start_x = EEPROM_Load(EEPROM_ADDRESS_COR_X, lcd_offset_min, lcd_offset_max, lcd_offset_default, 1);
  lcd_offset_start_y = EEPROM_Load(EEPROM_ADDRESS_COR_Y, lcd_offset_min, lcd_offset_max, lcd_offset_default, 1);

  lcd_time_invert_on_h = EEPROM_Load(EEPROM_ADDRESS_invert_on_h, 0, Time_h_max, 0, 0);
  lcd_time_invert_on_m = EEPROM_Load(EEPROM_ADDRESS_invert_on_m, 0, Time_m_max, 0, 0);
  lcd_time_invert_off_h = EEPROM_Load(EEPROM_ADDRESS_invert_off_h, 0, Time_h_max, 1, 0);
  lcd_time_invert_off_m = EEPROM_Load(EEPROM_ADDRESS_invert_off_m, 0, Time_m_max, 1, 0);

  Temp_cor = EEPROM_Load(EEPROM_ADDRESS_Temp_cor, Temp_cor_min, Temp_cor_max, Temp_cor_default, 1);
  Vbat_cor = EEPROM_Load(EEPROM_ADDRESS_Vbat_cor, Vbat_cor_min, Vbat_cor_max, Vbat_cor_default, 1);

  light_min = EEPROM_Load(EEPROM_ADDRESS_light_min, lcd_light_min, lcd_light_max, lcd_light_min, 0);
  light_max = EEPROM_Load(EEPROM_ADDRESS_light_max, lcd_light_min, lcd_light_max, lcd_light_max, 0);

  Ilm_start = EEPROM_Load(EEPROM_ADDRESS_Ilm_start, Ilm_ADC_div_min, Ilm_ADC_div_max, Ilm_ADC_div_min, 0);

  ADC_channel_Temp_select = EEPROM_Load(EEPROM_ADDRESS_ADC_channel_Temp_select, ADC_channel_Temp_select_min, ADC_channel_Temp_select_max, ADC_channel_Temp_default, 0);

  Rest_warning_time = EEPROM_Load(EEPROM_ADDRESS_Rest_warning_time, Rest_warning_time_min, Rest_warning_time_max, Rest_warning_time_min, 0);

  RTC_cor = EEPROM_Load(EEPROM_ADDRESS_RTC_cor, RTC_cor_min, RTC_cor_max, 0, 1);

  lcd_drive_screen_update = EEPROM_Load(EEPROM_ADDRESS_lcd_drive_screen, lcd_drive_screen_update_min, lcd_drive_screen_update_max, lcd_drive_screen_update_min, 0);

  time_format_print = EEPROM_Load(EEPROM_ADDRESS_time_format_print, time_format_print_min, time_format_print_max, time_format_print_min, 0);

  RTC_Presc = EEPROM_Load(EEPROM_ADDRESS_RTC_Presc, RTC_Prescaler_min, RTC_Prescaler_max, 0, 1);

  // Установка RTC и корректировки
  if (RTC_Presc != 0)
  {
	  hrtc.Init.AsynchPrediv = (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_RTC) - 1) + RTC_Presc;
  }

  // Внимание ! при генерации через куб удалить MX_RTC_Init в секции Initialize all configured peripherals
  MX_RTC_Init();
  HAL_RTCEx_SetSmoothCalib(&hrtc,0,0,-1 * RTC_cor);

  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_SetSecond_IT(&hrtc);

  // Запуск дисплея и вывод лого
  SSD1306_Clear();
  SSD1306_Light(0);
  Service_lcd_invert(lcd_time_invert_on_h,lcd_time_invert_on_m,lcd_time_invert_off_h,lcd_time_invert_off_m);
  SSD1306_DrawBitmap(0,0,Audi_bmp, 128, 64, 1);

  SSD1306_UpdateScreen();

  HAL_Delay(delay_before_light_up);

  // Плавное нарастание яркости
  for(uint8_t Light_up = 0; Light_up < light_lcd_start; Light_up++)
  {
	SSD1306_Light(Light_up);
	HAL_Delay(5);
  }

  HAL_Delay(delay_start_logo);
  SSD1306_Clear();

  HAL_TIM_Base_Start_IT(&htim2);

  // Проверить моточасы
  Service_Engine_Hours();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	BUTTON_Action_Freeze(30);

	// Обновление основного экрана
	const int16_t lcd_main_screen_update = 400;

	uint32_t lcd_update_millis_current = 0;
	uint32_t lcd_update_execute = 0;
	static uint32_t lcd_update_millis_prev = 0;

	lcd_update_millis_current = HAL_GetTick();
	lcd_update_execute = lcd_update_millis_current - lcd_update_millis_prev;

	if (lcd_update_execute >= lcd_main_screen_update)
	{
		lcd_main_screen_print(lcd_offset_start_x, lcd_offset_start_y, time_format_print, Service_Temp(ADC_channel_Temp_select, Temp_cor), Vbat_cor);
		lcd_update_millis_prev = HAL_GetTick();
	}

	// Вывод на оcновной экран времени за рулём
	uint32_t lcd_drive_millis_current = 0;
	uint32_t lcd_drive_execute = 0;
	static uint32_t lcd_drive_millis_prev = 0;

	lcd_drive_millis_current = HAL_GetTick();
	lcd_drive_execute = lcd_drive_millis_current - lcd_drive_millis_prev;

	if (lcd_drive_screen_update > 0 && lcd_drive_execute >= (lcd_drive_screen_update * lcd_drive_screen_update_mult))
	{
		lcd_drive_screen_print(lcd_offset_start_x, lcd_offset_start_y);
		lcd_drive_millis_prev = HAL_GetTick();
	}

	// Обработка кнопок
	if (BUTTON_Get_Actions(BUTTON_HS) == BUTTON_CLICK_SHORT)
	{
		Set_RTC_h_up();
		lcd_main_screen_print(lcd_offset_start_x, lcd_offset_start_y, time_format_print, Service_Temp(ADC_channel_Temp_select, Temp_cor), Vbat_cor);
		HAL_Delay(screen_delay_after_set);
	}

	if (BUTTON_Get_Actions(BUTTON_HS) == BUTTON_CLICK_LONG)
	{
		lcd_on_off(Service_lcd_light(Ilm_start, light_min, light_max));
		lcd_main_screen_print(lcd_offset_start_x, lcd_offset_start_y, time_format_print, Service_Temp(ADC_channel_Temp_select, Temp_cor), Vbat_cor);
		HAL_Delay(screen_delay_after_set);
	}

	if (BUTTON_Get_Actions(BUTTON_MS) == BUTTON_CLICK_SHORT)
	{
		Set_RTC_m_up();
		lcd_main_screen_print(lcd_offset_start_x, lcd_offset_start_y, time_format_print, Service_Temp(ADC_channel_Temp_select, Temp_cor), Vbat_cor);
		HAL_Delay(screen_delay_after_set);
	}

	if (BUTTON_Get_Actions(BUTTON_MS) == BUTTON_CLICK_LONG)
	{
		BUTTON_Action_Freeze (10);

		const uint16_t lcd_delay_print_setup = 1500;
		sprintf(lcd_buff,"MENU SETUP");
		SSD1306_Clear();
		SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
		SSD1306_UpdateScreen();
		HAL_Delay (lcd_delay_print_setup);

		static uint8_t menu_number = 0;
		do
		{
			menu_number = menu_setup();
			switch(menu_number)
			{
				case 0:
					lcd_offset_start_x = (uint8_t) Input_Data(lcd_offset_start_x, lcd_offset_min, lcd_offset_max, true, EEPROM_ADDRESS_COR_X, "Offset X = ");
				break;

				case 1:
					lcd_offset_start_y = (uint8_t) Input_Data(lcd_offset_start_y, lcd_offset_min, lcd_offset_max, true, EEPROM_ADDRESS_COR_Y, "Offset Y = ");
				break;

				case 2:
					lcd_offset_start_x = 0;
					lcd_offset_start_y = 0;

					uint8_t lcd_offset_temp;

					lcd_offset_temp = (uint8_t)lcd_offset_start_x;

					HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_ADDRESS_COR_X , 1, &lcd_offset_temp, 1, 5);
					HAL_Delay(EEPROM_Read_Write_Delay);

					lcd_offset_temp = (uint8_t)lcd_offset_start_y;;

					HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_ADDRESS_COR_Y , 1, &lcd_offset_temp, 1, 5);
					HAL_Delay(EEPROM_Read_Write_Delay);
				break;

				case 3:
					sprintf(lcd_buff,"Offset X=%02i,Y=%02i",lcd_offset_start_x,lcd_offset_start_y);
					SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
					SSD1306_UpdateScreen();
					HAL_Delay (delay_message_info);
				break;

				case 4:
					lcd_time_invert_on_h = (uint8_t) Input_Data(lcd_time_invert_on_h, 0,Time_h_max, true, EEPROM_ADDRESS_invert_on_h, "Hour on = ");
					lcd_time_invert_on_m = (uint8_t) Input_Data(lcd_time_invert_on_m, 0,Time_m_max, true, EEPROM_ADDRESS_invert_on_m, "Minutes on = ");

					sprintf(lcd_buff,"Lcd invert ON");
					SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
					sprintf(lcd_buff,"%02u:%02u",lcd_time_invert_on_h, lcd_time_invert_on_m);
					SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_2, lcd_buff);

					SSD1306_UpdateScreen();
					HAL_Delay (delay_message_info);
				break;

				case 5:
					lcd_time_invert_off_h = (uint8_t) Input_Data(lcd_time_invert_off_h, 0,Time_h_max, true, EEPROM_ADDRESS_invert_off_h, "Hour off = ");
					lcd_time_invert_off_m = (uint8_t) Input_Data(lcd_time_invert_off_m, 0,Time_m_max, true, EEPROM_ADDRESS_invert_off_m,"Minutes off = ");

					sprintf(lcd_buff,"Lcd invert OFF");
					SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
					sprintf(lcd_buff,"%02u:%02u",lcd_time_invert_off_h, lcd_time_invert_off_m);
					SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_2, lcd_buff);

					SSD1306_UpdateScreen();
					HAL_Delay (delay_message_info);
				break;

				case 6:
					Temp_cor = (int8_t)Input_Data(Temp_cor, Temp_cor_min, Temp_cor_max, true, EEPROM_ADDRESS_Temp_cor,"Offset C = ");
				break;

				case 7:
					Vbat_cor = (int8_t)Input_Data(Vbat_cor, Vbat_cor_min, Vbat_cor_max, true, EEPROM_ADDRESS_Vbat_cor,"Cor. step = ");
				break;

				case 8:
					light_min = (uint8_t) Input_Data(light_min, lcd_light_min, lcd_light_max, true, EEPROM_ADDRESS_light_min, "Light MIN = ");
				break;

				case 9:
					light_max = (uint8_t) Input_Data(light_max, lcd_light_min, lcd_light_max, true, EEPROM_ADDRESS_light_max,"Light MAX = ");
				break;

				case 10:
					Ilm_start = (uint8_t) Input_Data(Ilm_start, Ilm_ADC_div_min, Ilm_ADC_div_max, true, EEPROM_ADDRESS_Ilm_start,"*16 ADC = ");
				break;

				case 11:
					Rest_warning_time = (uint8_t) Input_Data(Rest_warning_time, Rest_warning_time_min, Rest_warning_time_max, true, EEPROM_ADDRESS_Rest_warning_time,"*100 Sec = ");
				break;

				case 12:
					RTC_cor = (int8_t)Input_Data(RTC_cor, RTC_cor_min, RTC_cor_max, true, EEPROM_ADDRESS_RTC_cor,"RTCEx = ");
					HAL_RTCEx_SetSmoothCalib(&hrtc,0,0,-1 * RTC_cor);
				break;

				case 13:
					lcd_drive_screen_update = (int8_t)Input_Data(lcd_drive_screen_update, lcd_drive_screen_update_min, lcd_drive_screen_update_max, true, EEPROM_ADDRESS_lcd_drive_screen,"Info min = ");
				break;

				case 14:
					ADC_channel_Temp_select = (uint8_t) Input_Data(ADC_channel_Temp_select, ADC_channel_Temp_select_min, ADC_channel_Temp_select_max, true, EEPROM_ADDRESS_ADC_channel_Temp_select,"Temp. ext. ? = ");
				break;

				case 15:
					char buff_tmp[25];
					sprintf(buff_tmp,"Time * %uh = ",Engine_Hourse_Mul);
					Input_Data(EEPROM_Load(EEPROM_ADDRESS_Engine_Hours_Set, 0, 255,0,0), 0,255, true, EEPROM_ADDRESS_Engine_Hours_Set,buff_tmp);
					HAL_RTCEx_BKUPWrite(&hrtc, Sec_BKP_reg_Hi, 0);
					HAL_RTCEx_BKUPWrite(&hrtc, Sec_BKP_reg_Low, 0);
				break;

				case 16:
					lcd_info_screen(Rest_warning_time);
				break;

				case 17:
					RTC_TimeTypeDef sTime;
					HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

					sTime.Hours = Input_Data(sTime.Hours, 0, Time_h_max, false, 0, "RTC Hours = ");
					sTime.Minutes = Input_Data(sTime.Minutes, 0, Time_m_max, false, 0, "RTC Min. = ");
					sTime.Seconds = 0;

					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					HAL_Delay (100);
				break;

				case 18:
					HAL_RTC_DeInit(&hrtc);
					HAL_RTC_Init(&hrtc);
				break;

				case 19:
					hrtc.Init.AsynchPrediv = (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_RTC) - 1) + Input_Data(RTC_Presc, RTC_Prescaler_min, RTC_Prescaler_max, true, EEPROM_ADDRESS_RTC_Presc, "RTC Prsc = ");
					HAL_RTC_Init(&hrtc);
				break;

				case 20:
					time_format_print = Input_Data(time_format_print, time_format_print_min, time_format_print_max, true, EEPROM_ADDRESS_time_format_print,"Style time = ");
				break;

				case 21:
					SSD1306_Clear();
					SSD1306_GotoXY (10, lcd_font_7_line_0);
					SSD1306_Puts ("The author", &Font_7x10, 1);
					SSD1306_GotoXY (10, lcd_font_7_line_1);
					SSD1306_Puts ("Kazakov", &Font_7x10, 1);
					SSD1306_GotoXY (10, lcd_font_7_line_2);
					SSD1306_Puts ("Vladimir", &Font_7x10, 1);
					SSD1306_UpdateScreen();
					HAL_Delay (delay_message_info);
					SSD1306_Clear();
					SSD1306_GotoXY (10, lcd_font_7_line_0);
					SSD1306_Puts ("e-mail", &Font_7x10, 1);
					SSD1306_GotoXY (10, lcd_font_7_line_1);
					SSD1306_Puts ("kaz100690@", &Font_7x10, 1);
					SSD1306_GotoXY (10, lcd_font_7_line_2);
					SSD1306_Puts ("gmail.com", &Font_7x10, 1);
					SSD1306_UpdateScreen();
					HAL_Delay (delay_message_info);
				break;

			}

		Service_lcd_invert(lcd_time_invert_on_h,lcd_time_invert_on_m,lcd_time_invert_off_h,lcd_time_invert_off_m);
		Service_lcd_light(Ilm_start, light_min, light_max);
		SSD1306_Clear();
		}while(menu_number < 22);
	}

	//*********************************Сервисы*********************************
	const int16_t Service_update = 10;

	uint32_t Service_update_millis_current = 0;
	uint32_t Service_update_execute = 0;
	static uint32_t Service_update_millis_prev = 0;

	Service_update_millis_current = HAL_GetTick();
	Service_update_execute = Service_update_millis_current - Service_update_millis_prev;

	// Опрос сервисов
	if (Service_update_execute >= Service_update)
	{
		// Сервис контроля АКБ и вывода предупреждения
		Service_Control_Bat_Voltage(Vbat_cor);
		// Вывод предупреждения о долгой поездке.
		Service_Rest_Warning(Rest_warning_time, lcd_offset_start_x, lcd_offset_start_y);
		// Сервис управления инверсией подстветки
		Service_lcd_invert(lcd_time_invert_on_h,lcd_time_invert_on_m,lcd_time_invert_off_h,lcd_time_invert_off_m);
		// Сервис управления подсветкой
		Service_lcd_light(Ilm_start, light_min, light_max);
		// Сервис проверить двигатель
		Service_Check_Engine();

		Service_update_millis_prev = HAL_GetTick();
	}
	//******************************************************************
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  SSD1306_Init();
  /* USER CODE END I2C1_Init 2 */

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
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  // Добавить строку в файл stm32f1xx_hal_rcc.h
  // #define RCC_FLAG_RTCEN                   ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_RTCEN_Pos))  /*!< RTC Enable */
  // При генрации через куб функцию HAL_RTC_Init необходимо завернуть в условие
  /*
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET || __HAL_RCC_GET_FLAG(RCC_FLAG_RTCEN) == RESET)
  {
	  SSD1306_Clear();

	  sprintf(lcd_buff,"* RTC_Init");

	  SSD1306_GotoXY (10,10);
	  SSD1306_Puts (lcd_buff, &Font_7x10, 1);
	  SSD1306_UpdateScreen();

	  // Delay
	  for (uint32_t timer = 600000; timer > 0; timer--)
	  {
	  }
	}
   */

  /* USER CODE END RTC_Init 1 */

  /* Initialize RTC Only  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_CALIBCLOCK;

  // Условие включения инициализации RTC
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET || __HAL_RCC_GET_FLAG(RCC_FLAG_RTCEN) == RESET)
  {
	  SSD1306_Clear();

	  sprintf(lcd_buff,"* RTC_Init");

	  SSD1306_GotoXY (10,10);
	  SSD1306_Puts (lcd_buff, &Font_7x10, 1);
	  SSD1306_UpdateScreen();

	  // Delay
	  for (uint32_t timer = 600000; timer > 0; timer--)
	  {
	  }

	  while (HAL_RTC_Init(&hrtc) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /* USER CODE BEGIN Check_RTC_BKUP */

	  /* USER CODE END Check_RTC_BKUP */

	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0;
	  sTime.Minutes = 0;
	  sTime.Seconds = 0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		Error_Handler();
	  }
	  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	  DateToUpdate.Month = RTC_MONTH_JANUARY;
	  DateToUpdate.Date = 1;
	  DateToUpdate.Year = 0;

	  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		Error_Handler();
	  }
  }else
  {
	  /* Initialize RTC MSP */
	  HAL_RTC_MspInit(&hrtc);
	  /* Clear flag Second */
	  __HAL_RTC_SECOND_CLEAR_FLAG(&hrtc, RTC_FLAG_SEC);

	  hrtc.State = HAL_RTC_STATE_READY;
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 0;
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
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
	// Ограничить инкрементирование минут до 99 часов и 59 минут.
	if (RTC_Seconds < 359940)
	{
		RTC_Seconds++;
	}

	// Запись секунд в бекап регистр
	uint16_t Sec_BKP_Hi;
	uint16_t Sec_BKP_Low;

	Sec_BKP_Hi = (uint16_t) HAL_RTCEx_BKUPRead(hrtc, Sec_BKP_reg_Hi);
	Sec_BKP_Low = (uint16_t) HAL_RTCEx_BKUPRead(hrtc, Sec_BKP_reg_Low);

	if (Sec_BKP_Hi < 65535)
	{
		Sec_BKP_Hi++;
	}else
	{
		Sec_BKP_Low++;
		HAL_RTCEx_BKUPWrite(hrtc, Sec_BKP_reg_Low, Sec_BKP_Low);
		Sec_BKP_Hi = 0;
	}

	HAL_RTCEx_BKUPWrite(hrtc, Sec_BKP_reg_Hi, Sec_BKP_Hi);
}

void Service_Engine_Hours()
{
	const char *Text_service = "Service !";

	const uint16_t message_service_delay = 10000;

	uint8_t Hours_set = 0;

	HAL_I2C_Mem_Read(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_ADDRESS_Engine_Hours_Set, 1, &Hours_set, 1, 5);
	HAL_Delay(EEPROM_Read_Write_Delay);

	// Выполнять, если заданные часы > 0
	if (Hours_set > 0)
	{
		uint16_t Sec_BKP_Hi;
		uint16_t Sec_BKP_Low;
		Sec_BKP_Hi = (uint16_t) HAL_RTCEx_BKUPRead(&hrtc, Sec_BKP_reg_Hi);
		Sec_BKP_Low = (uint16_t) HAL_RTCEx_BKUPRead(&hrtc, Sec_BKP_reg_Low);

		uint32_t Hours_work = 0;

		Hours_work = Sec_BKP_Low;
		Hours_work <<= 16;
		Hours_work |= Sec_BKP_Hi;

		Hours_work /= 3600;

		// Если моточасы превышены, то отобразить сервис на экран
		if (Hours_work >= (Hours_set * Engine_Hourse_Mul))
		{
			SSD1306_Clear();
			sprintf(lcd_buff,"%s",Text_service);
			SSD1306_set_mid_pos(&Font_7x10, 0, SSD1306_HEIGHT / 2 - 3, lcd_buff);
			SSD1306_UpdateScreen();
			Delay_int_button(message_service_delay);
			SSD1306_Clear();
			HAL_Delay(delay_blank_screen);
		}
	}
}

int8_t Service_Temp(uint8_t Temp_ADC_port, int8_t Temp_cor)
{
	int16_t Temp_return = 0;

	const int8_t limit_temp_min = -35;
	const int8_t limit_temp_max = 85;

	#define Temp_sensor_table 25

	// Датчик NTC 10K (b3950-3977). Значения при делителе 10 кОм.
	const int16_t Temp_sensor_b3950[2][Temp_sensor_table] =
											{{limit_temp_max,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0,-5,-10,-15,-20,-25,-30,limit_temp_min},
			         {396,457,528,610,706,816,942,1085,1245,1423,1618,1827,2048,2275,2503,2726,2939,3136,3313,3469,3602,3714,3804,3877,3934}};

	switch(Temp_ADC_port)
	{
		case 0:
			// Внутренний датчик температуры
			Temp_return = (1.43 - (float)(Get_ADC(ADC_channel_Temp) * Step_Vref_Volt_Div))/0.0043;
		break;

		case 1:
			// Внешний датчик температуры

			// Термистор NTC 10K
			uint16_t ADC_temp_ext;
			ADC_temp_ext = Get_ADC(ADC_channel_Temp_ext);

			// Поиск значений по таблице близких к показниям ацп внешнего датчика
			uint8_t Temp_sensor_counter = 1;
			while (ADC_temp_ext > Temp_sensor_b3950[1][Temp_sensor_counter] && Temp_sensor_counter < Temp_sensor_table - 1)
			{Temp_sensor_counter++;}

			// интерполяция между соседними точками
			Temp_return = interpol(ADC_temp_ext, Temp_sensor_b3950[1][Temp_sensor_counter - 1], Temp_sensor_b3950[1][Temp_sensor_counter], Temp_sensor_b3950[0][Temp_sensor_counter - 1], Temp_sensor_b3950[0][Temp_sensor_counter]);

			// LM335
			//Temp_return = ((float)Get_ADC(ADC_channel_Temp_ext) * Step_Vref_Volt_Div) * 100 - 173.15;
		break;
	}

	Temp_return += Temp_cor;

	if (Temp_return < limit_temp_min)
	{
		Temp_return = limit_temp_min;
	}

	if (Temp_return > limit_temp_max)
	{
		Temp_return = limit_temp_max;
	}

	return (int8_t) Temp_return;
}

void Service_Check_Engine()
{
	// Переменная для отключения предупреждения.
	static uint8_t Button_check_warning = false;

	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
	{
		if (Button_check_warning == false)
		{
			SSD1306_Clear();
			SSD1306_DrawBitmap(24,5,Check_engine, 80, 54, 1);
			SSD1306_UpdateScreen();
			while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
			{
				asm("NOP");

				if (BUTTON_Get_Prev_Actions(BUTTON_HS) != BUTTON_NOT_ACTION || BUTTON_Get_Prev_Actions(BUTTON_MS) != BUTTON_NOT_ACTION)
				{
					Button_check_warning = true;

					BUTTON_IF_Actions_stop();

					break;
				}
			}
			SSD1306_Clear();
		}
	}else
	{
		Button_check_warning = false;
	}
}

void Service_Control_Bat_Voltage(int8_t Vbat_cor)
{
	const uint16_t Screen_time_delay_blink = 1000;

	static uint16_t Battery_error_counter = 0;

	// Переменная для отключения предупреждения.ё
	static uint8_t Button_check_warning = false;

	float Battery_voltage;

	Battery_voltage = ((Get_ADC(ADC_channel_Vbat) * Step_Vref_Volt_Div * Battery_div) + Battery_diode_cor) + (Vbat_cor * Battery_step_cor);

	if ((Battery_voltage < Battery_voltage_min || Battery_voltage > Battery_voltage_max))
	{
		if (Battery_error_counter >= Battery_error_max && Button_check_warning == false)
		{
			SSD1306_Clear();
			// Отрисовать аккумулятор
			SSD1306_DrawBitmap(SSD1306_WIDTH / 2 - 11, SSD1306_HEIGHT / 2 - 18,Bat_bmp, 22, 18, 1);

			while (Battery_voltage < Battery_voltage_min || Battery_voltage > Battery_voltage_max)
			{
				Battery_voltage = ((Get_ADC(ADC_channel_Vbat) * Step_Vref_Volt_Div * Battery_div) + Battery_diode_cor) + (Vbat_cor * Battery_step_cor);

				const int16_t lcd_screen_update = Screen_time_delay_blink;
				static uint8_t InvertDisplay = false;

				uint32_t lcd_update_millis_current = 0;
				uint32_t lcd_update_execute = 0;
				static uint32_t lcd_update_millis_prev = 0;

				lcd_update_millis_current = HAL_GetTick();
				lcd_update_execute = lcd_update_millis_current - lcd_update_millis_prev;

				// Обновление экрана напряжения
				if (lcd_update_execute >= lcd_screen_update)
				{
					// Вывод напряжения на дисплей.

					sprintf(lcd_buff,"%4.1f%s",Battery_voltage, "Vt");
					SSD1306_set_mid_pos(&Font_11x18, 0, SSD1306_HEIGHT / 2 + 3, lcd_buff);

					SSD1306_UpdateScreen();

					if (InvertDisplay)
					{
						SSD1306_InvertDisplay (0);
						InvertDisplay = false;
					}else
					{
						SSD1306_InvertDisplay (1);
						InvertDisplay = true;
					}

					lcd_update_millis_prev = HAL_GetTick();
				}


				if (BUTTON_Get_Prev_Actions(BUTTON_HS) != BUTTON_NOT_ACTION || BUTTON_Get_Prev_Actions(BUTTON_MS) != BUTTON_NOT_ACTION)
				{
					Button_check_warning = true;

					BUTTON_IF_Actions_stop();

					break;
				}
			}
			Battery_error_counter = 0;
			SSD1306_Clear();
		}else
		{
			if (Battery_error_counter < Battery_error_max)
			{
				Battery_error_counter++;
			}
		}
	}else
	{
		Battery_error_counter = 0;
		Button_check_warning = false;
	}

}

uint16_t Get_ADC(uint8_t channel)
{
	static float filVal[adc_lenght] = {0};
	uint16_t newVal[adc_lenght] = {0};
	uint16_t adc[adc_lenght] = {0};

	ADC_Ready = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&newVal, adc_lenght); // Запуск АЦП.

	// Ожидание завершения АЦП
    while(!ADC_Ready)
    {
    }

	for (uint8_t Table_counter = 0; Table_counter < adc_lenght; Table_counter++)
	{
		adc[Table_counter] = expRunningAverage(&filVal[Table_counter], newVal[Table_counter]);
	}

	return adc[channel];
}

// Бегущее среднее
uint16_t expRunningAverage(float *filVal, uint16_t newVal)
{
	*filVal += (newVal - *filVal) * Filter_Ratio_Work;
	return (uint16_t)*filVal;
}

uint8_t Service_lcd_light(uint8_t Ilm_start, uint8_t light_min, uint8_t light_max)
{
	uint8_t light_lcd;

	// АЦП прочитанно, АЦП наименьшее, АЦП наибольшее, Яркость минимальная, Яркость максимальная
	light_lcd = (uint8_t)interpol(Get_ADC(ADC_channel_Ilm), (Ilm_start * ADC_mul), (Ilm_ADC_div_max * ADC_mul), light_min, light_max);
	SSD1306_Light(light_lcd);

	return light_lcd;
}

// интерполяция
int32_t interpol(int16_t xN, int16_t x1, int16_t x2, int32_t fx1, int32_t fx2)
{
	int32_t result;
	int32_t temp;

	if (xN <= x1)
	{result = fx1;}
	else
	{
		if (xN >= x2)
		{result = fx2;}
		else
		{
			temp = fx2 - fx1;
			temp *= (int32_t)xN - x1;

			result = temp / (x2 - x1) + fx1;
		}
	}

	return result;
}

int16_t Input_Data(int16_t Data_current, int16_t Data_min, int16_t Data_max, uint8_t EEPROM_save, uint16_t EEPROM_offset_save, char *Text_data)
{
	SSD1306_Clear();

	do
	{
		SSD1306_GotoXY (10, lcd_font_7_line_1);
		sprintf(lcd_buff,"%s%-5i", Text_data, Data_current);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		HAL_Delay(10);

		// Операции с кнопками
		while (BUTTON_Get_Actions(BUTTON_HS) == BUTTON_NOT_ACTION && BUTTON_Get_Actions(BUTTON_MS) == BUTTON_NOT_ACTION)
		{
		}

		if (BUTTON_Get_Actions(BUTTON_MS) >= BUTTON_CLICK_SHORT)
		{
			if (Data_current < Data_max)
			{
				Data_current++;
			}else
			{
				Data_current = Data_min;
			}
		}

		if (BUTTON_Get_Actions(BUTTON_HS) == BUTTON_CLICK_SHORT)
		{
			if (Data_current > Data_min)
			{
				Data_current--;
			}else
			{
				Data_current = Data_max;
			}
		}
	}
	while(BUTTON_Get_Actions(BUTTON_HS) != BUTTON_CLICK_LONG);

	SSD1306_Clear();

	// Проверить наличие EEPROM
	if (HAL_I2C_IsDeviceReady(&hi2c1,(uint16_t) I2C1_EEPROM_ADDRESS<<1, 1, 50) == HAL_OK)
	{
		// Сохранить значение в EEPROM, если есть разрешение
		if (EEPROM_save == true)
		{
			uint8_t temp_data = (uint8_t) Data_current;
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_offset_save, 1, &temp_data, 1, 5);
			HAL_Delay(EEPROM_Read_Write_Delay);

			sprintf(lcd_buff,"EEPROM Save !");
			SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
		}else
		{
			sprintf(lcd_buff,"Successful !");
			SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
		}
	}else
	{
		sprintf(lcd_buff,"NO EEPROM!");
		SSD1306_set_mid_pos(&Font_7x10, 0, lcd_font_7_line_1, lcd_buff);
	}

	SSD1306_UpdateScreen();
	while (BUTTON_Get_Actions(BUTTON_MS) != BUTTON_NOT_ACTION)
	{
	}
	HAL_Delay(delay_message_info);
	SSD1306_Clear();

	return  Data_current;
}

int16_t EEPROM_Load(uint8_t EEPROM_ADDRESS, int16_t Data_min, int16_t Data_max, int16_t Data_default, uint8_t type_return)
{
	int16_t Data_return = 0;

	if (HAL_I2C_IsDeviceReady(&hi2c1,(uint16_t) I2C1_EEPROM_ADDRESS<<1, 1, 50) != HAL_OK)
	{
		EEPROM_Error_print(0, 0, I2C1_EEPROM_ADDRESS);
		return Data_default;
	}

	uint8_t EEPROM_Data = 0;
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_ADDRESS, 1, &EEPROM_Data, 1, 5);
	HAL_Delay(EEPROM_Read_Write_Delay);

	// Явно преобразовать значение.
	if (type_return > 0)
	{
		Data_return = (int8_t)EEPROM_Data;
	}else
	{
		Data_return = EEPROM_Data;
	}

	// Проверка загруженного значения
	if (Data_return < Data_min || Data_return > Data_max)
	{
		// Вывести ошибку
		EEPROM_Error_print(1,EEPROM_ADDRESS, I2C1_EEPROM_ADDRESS);
		Data_return = Data_default;

		EEPROM_Data = (uint8_t)Data_default;
		// Записать в еепром безопасное значение.
		HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_ADDRESS, 1, &EEPROM_Data, 1, 5);
		HAL_Delay(EEPROM_Read_Write_Delay);
	}

	return Data_return;
}

void EEPROM_Error_print (uint8_t event_error, uint8_t event_adress, uint8_t eeprom_adress)
{
	switch(event_error)
	{
		case 0:
			sprintf(lcd_buff,"*NO EEPROM 0x%02x", eeprom_adress);
		break;

		case 1:
			sprintf(lcd_buff,"*BAD DATA 0x%02x", event_adress);
		break;
	}

	SSD1306_Clear();

	SSD1306_GotoXY (10,10);
	SSD1306_Puts (lcd_buff, &Font_7x10, 1);
	SSD1306_UpdateScreen();

	HAL_Delay (300);
}

void Service_lcd_invert(uint8_t lcd_time_invert_on_h, uint8_t lcd_time_invert_on_m, uint8_t lcd_time_invert_off_h, uint8_t lcd_time_invert_off_m)
{
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	uint32_t time_get;
	uint32_t time_on;
	uint32_t time_off;

	time_get = sTime.Hours * 3600;
	time_get += sTime.Minutes * 60;

	time_on = lcd_time_invert_on_h * 3600;
	time_on += lcd_time_invert_on_m * 60;

	time_off = lcd_time_invert_off_h * 3600;
	time_off += lcd_time_invert_off_m * 60;

	// Определить направление времени
	if (time_on < time_off)
	{
		// Включить инверсию по времени
		if (time_get >= time_on && time_get < time_off)
		{
			SSD1306_InvertDisplay (1);
		}else
		{
			SSD1306_InvertDisplay (0);
		}
	}else
	{
		// Включить инверсию по времени
		if (time_get >= time_off && time_get < time_on)
		{
			SSD1306_InvertDisplay (0);
		}else
		{
			SSD1306_InvertDisplay (1);
		}
	}
}

int16_t value_test(int16_t value_current, int16_t value_min, int16_t value_max, int16_t value_default)
{
	if (value_current > value_max || value_current < value_min)
	{
			return value_default;
	}

	return value_current;
}

uint8_t menu_setup()
{
	struct table{char *menu_item;};

	const struct table menu_lst[] = {{"Lcd correction X"},{"Lcd correction Y"},{"Lcd cor.reset"},{"Lcd offset"},{"Time invert ON"},{"Time invert OFF"}, {"Temp correction"}, {"Battery correction"}, {"Lcd light min"}, {"Lcd light max"}, {"Illumination ADC start"}, {"Rest warning"},{"RTC correction"},{"Time drive"},{"Temp. sensor channel"},{"Service set"},{"Info"},{"Set RTC"},{"RTC Init"},{"RTC Prescaler"},{"Time style"},{"Author"},{"Exit"}};
	const uint8_t menu_lcd_lines = 3;
	const uint8_t menu_lenght_pointer = 4;

	const uint8_t menu_poz_x_start = 10;
	const uint8_t menu_poz_y_start = 0;

	const uint8_t menu_poz_x_offset_start_line = 10;

	const uint8_t symbols_lcd_line_max_max = 14;
	const uint8_t symbols_lcd_line_max_last = 9;

	const uint16_t ticker_delay_screen_change = 500;
	const uint16_t ticker_delay_begin = 700;

	// Общие количество линий
	const uint8_t menu_line_all = (sizeof(menu_lst) / menu_lenght_pointer) - 1;
	// Общие количество страниц
	const uint8_t menu_page_all = menu_line_all / menu_lcd_lines;

	uint8_t menu_current_line = 0;
	uint8_t menu_current_page = 0;
	uint8_t menu_page_line = 0;

	SSD1306_Clear();

    while (BUTTON_Get_Actions(BUTTON_MS) != BUTTON_NOT_ACTION)
    {
    }

	//
	uint8_t menu_lines_on_page;

	do
	{
		if (menu_current_page < menu_page_all)
		{
			menu_lines_on_page = menu_lcd_lines;
		}else
		{
			menu_lines_on_page = menu_line_all - (menu_page_line - 1);
		}

		// Установить позицию линии
		for (uint8_t line_print_counter = 0; line_print_counter < menu_lines_on_page; line_print_counter++)
		{
		  switch(line_print_counter)
		  {
			  case 0:
				  SSD1306_GotoXY(menu_poz_x_start + menu_poz_x_offset_start_line, lcd_font_7_line_0 + menu_poz_y_start);
			  break;

			  case 1:
				  SSD1306_GotoXY(menu_poz_x_start + menu_poz_x_offset_start_line, lcd_font_7_line_1 + menu_poz_y_start);
			  break;

			  case 2:
				  SSD1306_GotoXY(menu_poz_x_start + menu_poz_x_offset_start_line, lcd_font_7_line_2 + menu_poz_y_start);
			  break;

			  case 3:
				  SSD1306_GotoXY(menu_poz_x_start + menu_poz_x_offset_start_line, lcd_font_7_line_3 + menu_poz_y_start);
			  break;
		  }

		  // Вывод пукнта меню и поставить многоточие если строка длинная
		  uint8_t symbols_in_text = 0;
		  char *menu_lst_text_pointer;

		  menu_lst_text_pointer = menu_lst[menu_page_line + line_print_counter].menu_item;
		  symbols_in_text = strlen(menu_lst_text_pointer);

		  if ((line_print_counter < (menu_lcd_lines - 1) && symbols_in_text <= symbols_lcd_line_max_max) || (line_print_counter >= (menu_lcd_lines - 1) && symbols_in_text <= symbols_lcd_line_max_last))
		  {
			  SSD1306_Puts(menu_lst_text_pointer, &Font_7x10, 1);
		  }else
		  {
			uint8_t symbols_lcd_line_max = 0;
			if (line_print_counter >= (menu_lcd_lines - 1))
			{
				symbols_lcd_line_max = symbols_lcd_line_max_last;
			}else
			{
				symbols_lcd_line_max = symbols_lcd_line_max_max;
			}
			sprintf(lcd_buff,"%.*s..",symbols_lcd_line_max - 2, menu_lst_text_pointer);
			SSD1306_Puts(lcd_buff, &Font_7x10, 1);
		  }
		}

		// Вывести текущий номер страницы
		sprintf(lcd_buff,"P%u:%u",menu_current_page,menu_page_all);
		lcd_line_set ((symbols_lcd_line_max_last * 10) + 1, menu_poz_y_start, menu_lcd_lines - 1);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);

		// Стереть точку
		for (uint8_t menu_point_clear_counter = 0; menu_point_clear_counter < menu_lcd_lines; menu_point_clear_counter++)
		{
			switch(menu_point_clear_counter)
			{
			  case 0:
					SSD1306_GotoXY(menu_poz_x_start, lcd_font_7_line_0 + menu_poz_y_start);
			  break;

			  case 1:
				  	SSD1306_GotoXY(menu_poz_x_start, lcd_font_7_line_1 + menu_poz_y_start);
			  break;

			  case 2:
				 	SSD1306_GotoXY(menu_poz_x_start, lcd_font_7_line_2 + menu_poz_y_start);
			  break;

			  case 3:
				    SSD1306_GotoXY(menu_poz_x_start, lcd_font_7_line_3 + menu_poz_y_start);
			  break;
			}
			SSD1306_Puts(" ", &Font_7x10, 1);
		}

		// Узнать позицию линии
		uint8_t menu_line_poz;
		switch(menu_current_line)
		{
		  case 0:
			  menu_line_poz = lcd_font_7_line_0 + menu_poz_y_start;
		  break;

		  case 1:
			  menu_line_poz = lcd_font_7_line_1 + menu_poz_y_start;
		  break;

		  case 2:
			  menu_line_poz = lcd_font_7_line_2 + menu_poz_y_start;
		  break;

		  case 3:
			  menu_line_poz = lcd_font_7_line_3 + menu_poz_y_start;
		  break;

		  default:
			  menu_line_poz = 0;
		  break;
		}

		// Установить точку
		SSD1306_GotoXY(menu_poz_x_start, menu_line_poz);
		SSD1306_Puts("*", &Font_7x10, 1);
		SSD1306_UpdateScreen();

		while (BUTTON_Get_Prev_Actions(BUTTON_HS) != BUTTON_NOT_ACTION)
		{}

		// Бегущая строка
		uint8_t set_point_symbols_in_text = 0;
		char *set_point_menu_lst_text_pointer_current;

		set_point_menu_lst_text_pointer_current = menu_lst[menu_page_line + menu_current_line].menu_item;

		uint8_t set_point_symbols_lcd_line_max = 0;
		if (menu_current_line >= (menu_lcd_lines - 1))
		{
			set_point_symbols_lcd_line_max = symbols_lcd_line_max_last;
		}else
		{
			set_point_symbols_lcd_line_max = symbols_lcd_line_max_max;
		}

		set_point_symbols_in_text = strlen(set_point_menu_lst_text_pointer_current);

		// Скролинг не работает
		//SSD1306_ScrollRight(menu_line_poz,50);

		// Ожидать действие пользователя
		do
		{
			// Если символов в тексте больше чем доступно в строке LCD, тогда запустить бегущую строку
			if (set_point_symbols_in_text > set_point_symbols_lcd_line_max)
			{
				Delay_int_button(ticker_delay_begin);

				// Бегущая строка
				for (uint8_t line_pointer_counter = 0; line_pointer_counter <= (set_point_symbols_in_text - set_point_symbols_lcd_line_max); line_pointer_counter++)
				{
					sprintf(lcd_buff,"%.*s",set_point_symbols_lcd_line_max, set_point_menu_lst_text_pointer_current + line_pointer_counter);
					SSD1306_GotoXY(menu_poz_x_start + menu_poz_x_offset_start_line, menu_line_poz);
					SSD1306_Puts(lcd_buff, &Font_7x10, 1);
					SSD1306_UpdateScreen();
					Delay_int_button(ticker_delay_screen_change);
					if (BUTTON_Get_Prev_Actions(BUTTON_HS) != BUTTON_NOT_ACTION || BUTTON_Get_Prev_Actions(BUTTON_MS) != BUTTON_NOT_ACTION)
					{
						break;
					}
				}
			}
		}while (BUTTON_Get_Prev_Actions(BUTTON_HS) == BUTTON_NOT_ACTION && BUTTON_Get_Prev_Actions(BUTTON_MS) == BUTTON_NOT_ACTION);


		if (BUTTON_Get_Prev_Actions(BUTTON_HS) >= BUTTON_CLICK_SHORT)
		{
			if (menu_current_line < menu_lines_on_page - 1)
			{
				menu_current_line++;
			}else
			{
				menu_current_line = 0;

				if (menu_current_page < menu_page_all)
				{
					menu_current_page++;
					menu_page_line += menu_lcd_lines;
				}else
				{
					menu_current_page = 0;
					menu_page_line = 0;
				}
				SSD1306_Clear();
			}
		}
	}while (BUTTON_Get_Prev_Actions(BUTTON_MS) == BUTTON_NOT_ACTION);

	BUTTON_IF_Actions_stop();

	return menu_page_line + menu_current_line;
}

uint16_t Service_Rest_Warning(uint8_t time_trigger, int8_t lcd_offset_start_x, int8_t lcd_offset_start_y)
{
	uint32_t Rest_Warning_screen_update = time_trigger * 100;		// Через какое время выводит сообщение  time_trigger * 100 = Секунды.
	const uint16_t Rest_Warning_before_delay = 3000;				// Время экрана бездействия для привлечения внимания (Мс.)
	const uint16_t Rest_Warning_screen_delay = 20000;				// Время отображения сообщения (Мс.)

	uint32_t lcd_update_seconds_current = 0;
	uint32_t lcd_update_execute = 0;
	static uint32_t lcd_update_seconds_prev = 0;

	lcd_update_seconds_current = RTC_Seconds;
	lcd_update_execute = lcd_update_seconds_current - lcd_update_seconds_prev;

	// Обновление экрана на главной экране
	if (lcd_update_execute >= Rest_Warning_screen_update && time_trigger > 0)
	{
		SSD1306_Clear();
		HAL_Delay(Rest_Warning_before_delay);

		sprintf(lcd_buff,"DRIVING");
		SSD1306_set_mid_pos(&Font_11x18, 0, lcd_offset_start_y + 15, lcd_buff);
		sprintf(lcd_buff,"LONG TIME!");
		SSD1306_set_mid_pos(&Font_11x18, 0, lcd_offset_start_y + 35, lcd_buff);
		SSD1306_UpdateScreen();
		Delay_int_button(Rest_Warning_screen_delay);
		lcd_update_seconds_prev = RTC_Seconds;
		SSD1306_Clear();
		HAL_Delay(delay_blank_screen);
	}
	return (Rest_Warning_screen_update - lcd_update_execute) / 60;
}

void lcd_on_off(uint8_t LCD_Light_Current)
{
	const uint16_t lcd_change_mode_delay = 1000;

	const uint8_t Light_change_delay = 35;

	for(uint8_t Light_down = LCD_Light_Current; Light_down > 0; Light_down--)
	{
		SSD1306_Light(Light_down);
		HAL_Delay(Light_change_delay);
	}

	SSD1306_OFF();
    while (BUTTON_Get_Actions(BUTTON_HS) != BUTTON_NOT_ACTION)
    {
    }
    while (BUTTON_Get_Actions(BUTTON_HS) == BUTTON_NOT_ACTION && BUTTON_Get_Actions(BUTTON_MS) == BUTTON_NOT_ACTION)
    {
    }

    SSD1306_ON();

	for(uint8_t Light_up = 0; Light_up <= LCD_Light_Current; Light_up++)
	{
		SSD1306_Light(Light_up);
		HAL_Delay(Light_change_delay);
	}

	HAL_Delay(lcd_change_mode_delay);
}

void Set_RTC_h_up()
{
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	if (sTime.Hours < Time_h_max)
	{
		sTime.Hours++;
	}else
	{
		sTime.Hours = 0;
	}
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}

void Set_RTC_m_up()
{
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	if (sTime.Hours < Time_m_max)
	{
		sTime.Minutes++;
	}else
	{
		sTime.Minutes = 0;
	}
	sTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}

void lcd_main_screen_print(int8_t lcd_offset_start_x, int8_t lcd_offset_start_y, uint8_t lcd_time_format, int8_t Temp , int8_t Vbat_cor)
{
	float adc_volt[1] = {0};
	adc_volt[0] = Get_ADC(ADC_channel_Vbat) * Step_Vref_Volt_Div;

	const uint8_t Poz_line_info = 36;

	// Считать время
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	static int8_t lcd_offset_prev_x;
	static int8_t lcd_offset_prev_y;

	if (lcd_offset_start_x != lcd_offset_prev_x || lcd_offset_start_y != lcd_offset_prev_y)
	{
		SSD1306_Clear();

		lcd_offset_prev_x = lcd_offset_start_x;
		lcd_offset_prev_y = lcd_offset_start_y;
	}

	// Тип вывода на дисплей времени
	switch (lcd_time_format)
	{
		case 0:
			sprintf(lcd_buff,"%02u:%02u:%02u",sTime.Hours,sTime.Minutes,sTime.Seconds);
			SSD1306_set_mid_pos(&Font_11x18, lcd_offset_start_x, Poz_line_time_y + lcd_offset_start_y, lcd_buff);
		break;

		case 1:
			sprintf(lcd_buff,"%02u:%02u",sTime.Hours,sTime.Minutes);
			SSD1306_set_mid_pos(&Font_11x18, lcd_offset_start_x, Poz_line_time_y + lcd_offset_start_y, lcd_buff);
		break;

		case 2:
			sprintf(lcd_buff,"%02u:%02u",sTime.Hours,sTime.Minutes);
			SSD1306_set_mid_pos(&Font_16x26, lcd_offset_start_x, Poz_line_time_y + lcd_offset_start_y, lcd_buff);
		break;
	}

	const uint8_t Temp_pic_size_x = 18;
	const uint8_t Temp_pic_size_y = 18;

	// Стереть прошлый символ значка температуры
	SSD1306_DrawFilledRectangle(lcd_offset_start_x + 7, lcd_offset_start_y + Poz_line_info, lcd_offset_start_x + 7 + Temp_pic_size_x, lcd_offset_start_y + Poz_line_info + Temp_pic_size_y, 0);
	// Вывод на дисплей соотвествующего символа
	if (Temp >= 0)
	{
		SSD1306_DrawBitmap(lcd_offset_start_x + 7, lcd_offset_start_y + Poz_line_info,Temp_heat_bmp, Temp_pic_size_x, Temp_pic_size_y, 1);
		sprintf(lcd_buff,"%-2u",Temp);
	}else
	{
		SSD1306_DrawBitmap(lcd_offset_start_x + 7, lcd_offset_start_y + Poz_line_info,Temp_cold_bmp, Temp_pic_size_x, Temp_pic_size_y, 1);
		sprintf(lcd_buff,"%-2u",Temp * -1);
	}

	SSD1306_GotoXY (lcd_offset_start_x + 27, lcd_offset_start_y + Poz_line_info);
	SSD1306_Puts (lcd_buff, &Font_11x18, 1);

	// Расчёт напряжения батареи
	sprintf(lcd_buff,"%-4.1f",((adc_volt[0] * Battery_div) + Battery_diode_cor) + (Vbat_cor * Battery_step_cor));
	// Отрисовать аккумулятор
	SSD1306_DrawBitmap(lcd_offset_start_x + 55, lcd_offset_start_y + Poz_line_info,Bat_bmp, 22, 18, 1);
	SSD1306_GotoXY (lcd_offset_start_x + 78, lcd_offset_start_y + Poz_line_info);
	SSD1306_Puts (lcd_buff, &Font_11x18, 1);

	// Обновить экран.
	SSD1306_UpdateScreen();
}

void lcd_drive_screen_print(int8_t lcd_offset_start_x, int8_t lcd_offset_start_y)
{
	uint16_t Delay_screen_drive = 3000;

	uint32_t Seconds = RTC_Seconds;

	uint8_t drive_time_h;
	uint8_t drive_time_m;

	drive_time_h = Seconds / 3600;
	drive_time_m = (Seconds - (drive_time_h * 3600)) / 60 ;

	sprintf(lcd_buff,".%02u:%02u ",drive_time_h,drive_time_m);
	SSD1306_set_mid_pos(&Font_16x26, lcd_offset_start_x, lcd_offset_start_y + Poz_line_time_y, lcd_buff);

	// Обновить экран.
	SSD1306_UpdateScreen();

	if (Delay_int_button(Delay_screen_drive) > 0)
	{
		HAL_Delay(delay_message_info);
	}

	// Очистить строку времени
	sprintf(lcd_buff,"        ");
	SSD1306_GotoXY (0, lcd_offset_start_y + Poz_line_time_y);
	SSD1306_Puts (lcd_buff, &Font_16x26, 1);
}

void lcd_info_screen(uint8_t Rest_warning_time)
{
	const uint8_t lcd_offset_x = 15;

	const uint16_t lcd_time_update = 200;
	const uint16_t lcd_firmware_delay_print = delay_message_info;

	// Инфо прошивка, HAL, DEV ID
	SSD1306_Clear();

	sprintf(lcd_buff,"FW: %s", firmware_number);
	SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_0);
	SSD1306_Puts (lcd_buff, &Font_7x10, 1);

	sprintf(lcd_buff,"HAL:%lu", HAL_GetHalVersion());
	SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_1);
	SSD1306_Puts (lcd_buff, &Font_7x10, 1);

	sprintf(lcd_buff,"DEV ID:%lu", HAL_GetDEVID());
	SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_2);
	SSD1306_Puts (lcd_buff, &Font_7x10, 1);

	SSD1306_UpdateScreen();
	HAL_Delay(lcd_firmware_delay_print);
	SSD1306_Clear();

	// Инфо АЦП
	for (uint8_t lcd_print_info_counter = 0; lcd_print_info_counter < 40; lcd_print_info_counter++)
	{
		sprintf(lcd_buff,"TMP = %04u_%04u",Get_ADC(ADC_channel_Temp),Get_ADC(ADC_channel_Temp_ext));
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_0);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);
		sprintf(lcd_buff,"IGN = %04u",Get_ADC(ADC_channel_Vbat));
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_1);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);
		sprintf(lcd_buff,"ILM = %04u",Get_ADC(ADC_channel_Ilm));
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_2);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);
		SSD1306_UpdateScreen();
		HAL_Delay(lcd_time_update);
	}

	// Инфо таймеры
	SSD1306_Clear();

	RTC_TimeTypeDef sTime;
	for (uint8_t lcd_print_info_counter = 0; lcd_print_info_counter < 40; lcd_print_info_counter++)
	{
		SSD1306_GotoXY (0, lcd_font_7_line_0);
		SSD1306_Puts ("                  ", &Font_7x10, 1);
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		sprintf(lcd_buff,"RTC = %02u:%02u:%02u",sTime.Hours,sTime.Minutes,sTime.Seconds);
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_0);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);

		SSD1306_GotoXY (0, lcd_font_7_line_1);
		SSD1306_Puts ("                  ", &Font_7x10, 1);
		sprintf(lcd_buff,"RUN = %lu Sec.", RTC_Seconds);
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_1);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);

		SSD1306_GotoXY (0, lcd_font_7_line_2);
		SSD1306_Puts ("                  ", &Font_7x10, 1);
		sprintf(lcd_buff,"Warn = %u Min.",Service_Rest_Warning(Rest_warning_time, 0, 0));
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_2);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);

		SSD1306_UpdateScreen();
		HAL_Delay(lcd_time_update);
	}

	// Сервис инфо
	uint8_t Hours_set_tmp = 0;

	uint16_t Sec_BKP_Hi;
	uint16_t Sec_BKP_Low;

	HAL_I2C_Mem_Read(&hi2c1, (uint16_t) I2C1_EEPROM_ADDRESS<<1, EEPROM_ADDRESS_Engine_Hours_Set, 1, &Hours_set_tmp, 1, 5);
	HAL_Delay(EEPROM_Read_Write_Delay);

	SSD1306_Clear();
	SSD1306_GotoXY (22, lcd_font_7_line_0);
	SSD1306_Puts ("SERVICE INFO", &Font_7x10, 1);
	for (uint8_t lcd_print_info_counter = 0; lcd_print_info_counter < 40; lcd_print_info_counter++)
	{
		SSD1306_GotoXY (0, lcd_font_7_line_1);
		SSD1306_Puts ("                  ", &Font_7x10, 1);
		sprintf(lcd_buff,"SET = %u H.", Hours_set_tmp * Engine_Hourse_Mul);
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_1);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);

		Sec_BKP_Hi = (uint16_t) HAL_RTCEx_BKUPRead(&hrtc, Sec_BKP_reg_Hi);
		Sec_BKP_Low = (uint16_t) HAL_RTCEx_BKUPRead(&hrtc, Sec_BKP_reg_Low);

		uint32_t Sec_work = 0;

		Sec_work = Sec_BKP_Low;
		Sec_work <<= 16;
		Sec_work |= Sec_BKP_Hi;

		SSD1306_GotoXY (0, lcd_font_7_line_2);
		SSD1306_Puts ("                  ", &Font_7x10, 1);
		sprintf(lcd_buff,"RUN = %lu:%02u",Sec_work / 3600, (uint16_t)(Sec_work % 3600) / 60);
		SSD1306_GotoXY (lcd_offset_x, lcd_font_7_line_2);
		SSD1306_Puts (lcd_buff, &Font_7x10, 1);

		SSD1306_UpdateScreen();
		HAL_Delay(lcd_time_update);
	}

	SSD1306_Clear();
}

void lcd_line_set (uint8_t poz_x, uint8_t poz_y, uint8_t line)
{
	switch (line)
	{
		case 0:
			SSD1306_GotoXY (poz_x, poz_y + lcd_font_7_line_0);
		break;

		case 1:
			SSD1306_GotoXY (poz_x, poz_y + lcd_font_7_line_1);
		break;

		case 2:
			SSD1306_GotoXY (poz_x, poz_y + lcd_font_7_line_2);
		break;

		case 3:
			SSD1306_GotoXY (poz_x, poz_y + lcd_font_7_line_3);
		break;
	}
}

uint16_t Delay_int_button(uint16_t delay_ms)
{
	while (delay_ms > 0)
	{
		if (BUTTON_Get_Prev_Actions(BUTTON_HS) != BUTTON_NOT_ACTION || BUTTON_Get_Prev_Actions(BUTTON_MS) != BUTTON_NOT_ACTION)
		{
			break;
		}
		HAL_Delay(1);
		delay_ms--;
	}
	return delay_ms;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Stop_DMA(&hadc1);
	ADC_Ready = 1;
}

void SSD1306_set_mid_pos(FontDef_t *font, int8_t offset_x, uint8_t poz_y, char *Text_data)
{
	SSD1306_GotoXY ((SSD1306_WIDTH / 2 - (strlen(lcd_buff) * font->FontWidth / 2)) + offset_x, poz_y);
	SSD1306_Puts (lcd_buff, font, 1);
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
  //__disable_irq();

  SSD1306_GotoXY (10,10);
  SSD1306_Puts ("               ", &Font_7x10, 1);
  SSD1306_UpdateScreen();

  sprintf(lcd_buff,"Error_Handler");
  SSD1306_GotoXY (10,10);
  SSD1306_Puts (lcd_buff, &Font_7x10, 1);
  SSD1306_UpdateScreen();

  // Delay
  for (uint32_t timer = 600000; timer > 0; timer--)
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
