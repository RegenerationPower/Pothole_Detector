#ifndef WH1602_H_
#define WH1602_H_

#include "main.h"

void LCD_Init(void);
void LCD_Clear(void);
void LCD_PutString(uint8_t x, uint8_t y, char *str);

#endif /* WH1602_H_ */
