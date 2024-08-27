/**
  ******************************************************************************
  * @file           : button.h
  * @brief          : Buttons driver
  * @author         : TQFP (for https://microtechnics.ru/user-blogs/)
  ******************************************************************************
  */

#ifndef BUTTON_H
#define BUTTON_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "ssd1306.h"
/* Declarations and definitions ----------------------------------------------*/

// Configuration

#define GPIO_BUTTON_NOT_PRESSED                              (GPIO_PIN_SET)

typedef enum {BUTTON_HS,
    BUTTON_MS,
	BUTTONS_NUM,
} ButtonID;

// End of configuration

#define DEBOUNCE_TIME_MS                                     1
#define GPIO_BUTTON_PRESSED                                  (!GPIO_BUTTON_NOT_PRESSED)

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} McuPin;

typedef enum
{
  BUTTON_NOT_PRESSED                   = 0,
  BUTTON_PRESSED                       = 1,
} ButtonState;

typedef enum
{
  BUTTON_NOT_ACTION		                   	= 0,
  BUTTON_CLICK_SHORT						= 1,
  BUTTON_CLICK_LONG							= 2,
} ButtonAction;

/* Functions -----------------------------------------------------------------*/

extern void BUTTON_Service();
extern ButtonAction BUTTON_Get_Actions(uint8_t port);
extern ButtonAction BUTTON_Get_Prev_Actions(uint8_t port);
extern void BUTTON_Action_Freeze(uint8_t value);
extern void BUTTON_IF_Actions_stop();

#endif // #ifndef BUTTON_H
