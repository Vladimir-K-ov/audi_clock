/**
  ******************************************************************************
  * @file           : button.c
  * @brief          : Buttons driver
  * @author         : TQFP (for https://microtechnics.ru/user-blogs/)
  ******************************************************************************
  */



/* Includes ------------------------------------------------------------------*/

#include "button.h"
#include "stdint.h"
/* Declarations and definitions ----------------------------------------------*/

// Configuration
#define Sampler 3

const uint16_t Set_State_Sample_CLICK_SHORT = 1;
const uint16_t Set_State_Sample_CLICK_LONG = 270;

static McuPin buttons[BUTTONS_NUM] = {{GPIOB, GPIO_PIN_9},
									  {GPIOB, GPIO_PIN_8}};

static ButtonState 	Button_State_Processed[BUTTONS_NUM] = {BUTTON_NOT_PRESSED};
static ButtonAction Button_Current_Action[BUTTONS_NUM] = {BUTTON_NOT_ACTION};
static ButtonAction Button_SS_Action[BUTTONS_NUM] = {BUTTON_NOT_ACTION};

// Определяем действие кнопки.
static uint16_t State_counter[BUTTONS_NUM] = {0};

// Скорость сброса состояния кнопки
static uint8_t Action_Freeze = 10;

/* Functions -----------------------------------------------------------------*/

void BUTTON_Service()
{
//--------------------------------------
	static uint8_t Sampler_counter = 0;
	static ButtonState Hardware_ButtonState[BUTTONS_NUM][Sampler] = {BUTTON_NOT_PRESSED};

	// Опрос физических кнопок.
	for (uint8_t port_counter = 0; port_counter < BUTTONS_NUM; port_counter++)
	{
		asm("NOP");
		asm("NOP");
		asm("NOP");

		// Опрос на нажатие кнопок
		if (!HAL_GPIO_ReadPin(buttons[port_counter].port, buttons[port_counter].pin))
		{
			Hardware_ButtonState[port_counter][Sampler_counter] = BUTTON_PRESSED;
		}else
		{
			Hardware_ButtonState[port_counter][Sampler_counter] = BUTTON_NOT_PRESSED;
		}

		// Анализ таблицы состояний.
		// Если хоть одно из состояние != BUTTON_PRESSED, то состояние будет сброшенно на BUTTON_NOT_PRESSED
		Button_State_Processed[port_counter] = BUTTON_PRESSED;
		for (uint8_t y = 0; y < (Sampler - 1); y++)
		{
			if (Hardware_ButtonState[port_counter][y] != BUTTON_PRESSED)
			{
				Button_State_Processed[port_counter] = BUTTON_NOT_PRESSED;
				break;
			}
		}

		if (Button_State_Processed[port_counter] == BUTTON_PRESSED)
		{
			if (State_counter[port_counter] >= Set_State_Sample_CLICK_LONG)
			{
				Button_Current_Action[port_counter] = BUTTON_CLICK_LONG;
				Button_SS_Action[port_counter] = Button_Current_Action[port_counter];
			}

			// Счётчик определения действия кнопки.
			if (State_counter[port_counter] < UINT16_MAX)
			{State_counter[port_counter]++;}

		}else
		{
			//*******************
			if (State_counter[port_counter] >= Set_State_Sample_CLICK_SHORT)
			{
				Button_Current_Action[port_counter] = BUTTON_CLICK_SHORT;
				Button_SS_Action[port_counter] = Button_Current_Action[port_counter];
			}


			// Заморозить состояние действия, после отпускания кнопки.
			static uint8_t Current_Action_Freeze_Counter[BUTTONS_NUM] = {0};
			if (Current_Action_Freeze_Counter[port_counter] >= Action_Freeze)
			{
				Button_Current_Action[port_counter] = BUTTON_NOT_ACTION;
				State_counter[port_counter] = 0;
				Current_Action_Freeze_Counter[port_counter] = 0;
			}else
			{
				Current_Action_Freeze_Counter[port_counter]++;
			}
		}
	}

	// Счётчик таблицы.
	if (Sampler_counter < Sampler - 1)
	{
		Sampler_counter++;
	}else
	{
		Sampler_counter = 0;
	}
//--------------------------------------

}

// Получить значение кнопки дождавшись её отпускания, "Если зажатие превысит значение Set_State_Sample_CLICK_LONG, то отпускание не требуется"
extern ButtonAction BUTTON_Get_Actions(uint8_t port)
{
	return Button_Current_Action[port];
}

// Получить значение кнопки не дождавшись её отпускания
extern ButtonAction BUTTON_Get_Prev_Actions(uint8_t port)
{
	if (State_counter[port] >= Set_State_Sample_CLICK_SHORT)
	{
		return BUTTON_CLICK_SHORT;
	}else
	{
		return BUTTON_NOT_ACTION;
	}
}

// Если нажата хоть одна кнопка, то дождаться её отпускания
extern void BUTTON_IF_Actions_stop()
{
	const uint16_t delay_blank_screen = 400;

	// Так как конструкция выполнения обработки одинаковая, внесём изменения
	SSD1306_Clear();

	// В цикле опрашивать каждую кнопку
	for (uint8_t port_counter = 0; port_counter < BUTTONS_NUM; port_counter++)
	{
		// Если хоть одно значение не равно BUTTON_NOT_ACTION, то обнулить счётчик
		if (Button_Current_Action[port_counter] != BUTTON_NOT_ACTION)
		{
			port_counter = 0;
		}
	}

	// Так как конструкция выполнения обработки одинаковая, внесём изменения
	HAL_Delay(delay_blank_screen);
}

extern void BUTTON_Action_Freeze(uint8_t value)
{
	Action_Freeze = value;
}
/******************************************************************************/
