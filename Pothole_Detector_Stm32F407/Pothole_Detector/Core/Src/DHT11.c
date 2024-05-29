#include "DHT11.h"

#define DHT_Timeout 10000
#define readPin()		(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET)

DHT_Data DHT_getData()
{
	uint16_t timeout = 0;
	DHT_Data data =
	{ 0.0f, 0.0f };
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	uint8_t rawData[5] =
	{ 0, 0, 0, 0, 0 };

	// Налаштування GPIO на вихід
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// Ініціалізація датчика
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

	// Налаштування GPIO на вхід
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// Очікування початку відповіді від датчика
	while (readPin())
	{
		timeout++;
		if (timeout > DHT_Timeout)
		{
			return data;
		}
	}
	timeout = 0;

	while (!readPin())
	{
		timeout++;
		if (timeout > DHT_Timeout)
		{
			return data;
		}
	}
	timeout = 0;

	while (readPin())
	{
		timeout++;
		if (timeout > DHT_Timeout)
		{
			return data;

		}
	}

	// Зчитування даних з датчика
	for (uint8_t i = 0; i < 5; i++)
	{
		for (uint8_t j = 7; j != 255; j--)
		{
			uint16_t highLevel = 0, lowLevel = 0;

			while (!readPin())
			{
				lowLevel++;
			}

			while (readPin())
			{
				highLevel++;
			}

			if (highLevel > lowLevel)
			{
				rawData[i] |= (1 << j);
			}
		}
	}

	// Перевірка контрольної суми і запис даних
	if ((uint8_t) (rawData[0] + rawData[1] + rawData[2] + rawData[3])
			== rawData[4])
	{

		data.humidity = (float) rawData[0];
		data.temperature = (float) rawData[2];
	}

	return data;
}
