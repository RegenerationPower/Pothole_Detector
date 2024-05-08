#ifndef INC_WH1602_H_
#define INC_WH1602_H_
#include <stdint.h>
void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(void);
void LCD_Puts(uint8_t x, uint8_t y, char* str);
void LCD_BlinkOn(void);
void LCD_BlinkOff(void);
void LCD_CursorOn(void);
void LCD_CursorOff(void);
void LCD_ScrollLeft(void);
void LCD_ScrollRight(void);
void LCD_CreateChar(uint8_t location, uint8_t* data);
void LCD_PutCustom(uint8_t x, uint8_t y, uint8_t location);
void LCD_Put(uint8_t Data);
void LCD_Read_str(char* str, uint8_t x, uint8_t y,uint8_t length);

#endif /* INC_WH1602_H_ */
