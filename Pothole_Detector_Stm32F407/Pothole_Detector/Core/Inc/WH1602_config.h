#ifndef INC_WH1602_CONFIG_H_
#define INC_WH1602_CONFIG_H_

#define _LCD_USE_FREERTOS 0
#define _LCD_USE_MENU_LIB 0

#define _LCD_COLS         16
#define _LCD_ROWS         2
#include "main.h"
#define _LCD_RS_PORT      RS_GPIO_Port
#define _LCD_RS_PIN       GPIO_PIN_7

#define _LCD_E_PORT       EN_GPIO_Port
#define _LCD_E_PIN        GPIO_PIN_11

#define _LCD_RW_PORT      RW_GPIO_Port
#define _LCD_RW_PIN       GPIO_PIN_10

#define _LCD_D4_PORT      DB4_GPIO_Port
#define _LCD_D4_PIN				GPIO_PIN_12

#define _LCD_D5_PORT      DB5_GPIO_Port
#define _LCD_D5_PIN       GPIO_PIN_13

#define _LCD_D6_PORT      DB6_GPIO_Port
#define _LCD_D6_PIN       GPIO_PIN_14

#define _LCD_D7_PORT      DB7_GPIO_Port
#define _LCD_D7_PIN       GPIO_PIN_15

#endif /* INC_WH1602_CONFIG_H_ */
