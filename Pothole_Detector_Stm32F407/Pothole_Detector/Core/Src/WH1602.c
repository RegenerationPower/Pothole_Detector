#include "WH1602.h"

typedef struct
{
	uint8_t displayControl;
	uint8_t displayFunction;
	uint8_t displayMode;
	uint8_t X;
	uint8_t Y;

} LCD_Options;

static LCD_Options lcdOptions;

#define COLUMNS 16
#define ROWS 2

#define CLEAR_DISPLAY        0x01
#define ENTRY_SET        0x04
#define DISPLAY_CONTROL      0x08
#define FUNCTION_SET         0x20
#define SET_DDRAM_ADDR        0x80
#define ENTRY_LEFT           0x02
#define DISPLAY_ON           0x04
#define TWO_LINE               0x08

// Затримка в мікросекундах
void Delay_us(uint16_t us)
{
	uint32_t division = (SysTick->LOAD + 1) / 1000;
	uint32_t startUs = HAL_GetTick() * 1000 + (1000 - SysTick->VAL / division);
	while ((HAL_GetTick() * 1000 + (1000 - SysTick->VAL / division) - startUs
			< us))
		;
}

// Надсилання команди у 4-бітному режимі
static void sendCommand4bit(uint8_t cmd)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, (GPIO_PinState) (cmd & 0x08));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, (GPIO_PinState) (cmd & 0x04));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, (GPIO_PinState) (cmd & 0x02));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, (GPIO_PinState) (cmd & 0x01));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	Delay_us(50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	Delay_us(50);
}

static void sendCommand(uint8_t cmd)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	sendCommand4bit(cmd >> 4);
	sendCommand4bit(cmd & 0x0F);
}

static void setCursor(uint8_t col, uint8_t row)
{
	uint8_t row_offsets[] =
	{ 0x00, 0x40 };
	if (row >= ROWS)
	{
		row = 0;
	}
	lcdOptions.X = col;
	lcdOptions.Y = row;
	sendCommand(SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void displayOn(void)
{
	lcdOptions.displayControl |= DISPLAY_ON;
	sendCommand(DISPLAY_CONTROL | lcdOptions.displayControl);
}

// Виведення рядка на дисплей
void LCD_PutString(uint8_t x, uint8_t y, char *str)
{
	setCursor(x, y);
	while (*str)
	{
		// Перевірка чи не виходить курсор за межі рядка
		if (lcdOptions.X >= COLUMNS)
		{
			lcdOptions.X = 0;
			lcdOptions.Y++;
			setCursor(lcdOptions.X, lcdOptions.Y);
		}
		if (*str == '\n')
		{
			lcdOptions.Y++;
			setCursor(lcdOptions.X, lcdOptions.Y);
		}
		else if (*str == '\r')
		{
			setCursor(0, lcdOptions.Y);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
			sendCommand4bit(*str >> 4);
			sendCommand4bit(*str & 0x0F);
			lcdOptions.X++;
		}
		str++;
	}
}

void LCD_Clear(void)
{
	sendCommand(CLEAR_DISPLAY);
	HAL_Delay(2);
}

void LCD_Init(void)
{
	// Затримка для стабілізації після включення живлення
	while (HAL_GetTick() < 200)
	{
		HAL_Delay(1);
	}

	lcdOptions.X = 0;
	lcdOptions.Y = 0;
	lcdOptions.displayFunction = TWO_LINE;

	// Спроби встановити 4-бітний режим
	sendCommand4bit(0x03);
	HAL_Delay(20);

	sendCommand4bit(0x03);
	HAL_Delay(20);

	sendCommand4bit(0x03);
	HAL_Delay(20);

	sendCommand4bit(0x02);
	HAL_Delay(20);

	// Встановлення функціонального режиму дисплея
	sendCommand(FUNCTION_SET | lcdOptions.displayFunction);

	lcdOptions.displayControl = DISPLAY_ON;
	displayOn();
	LCD_Clear();

	// Встановлення режиму вводу
	lcdOptions.displayMode = ENTRY_LEFT;
	sendCommand(ENTRY_SET | lcdOptions.displayMode);
	HAL_Delay(20);
}
