#include "main.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "DHT11.h"
#include "WH1602.h"

// Адреса та регістри для MPU6050
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Пороги для увімкнення світлодіодів при нахилі системи
#define LEFT_THRESHOLD -10
#define RIGHT_THRESHOLD 10
#define TOP_THRESHOLD 10
#define BOTTOM_THRESHOLD -10

#define COMP_FILTER_ALPHA 0.98

// Структура для зберігання даних датчика
typedef struct
{
	float X;
	float Y;
	float Z;
} sensorData;

sensorData MPU6050_Data;

int16_t accel_X_Raw = 0, accel_Y_Raw = 0, accel_Z_Raw = 0;
int16_t gyro_X_Raw = 0, gyro_Y_Raw = 0, gyro_Z_Raw = 0;

float accel_X = 0.0, accel_Y = 0.0, accel_Z = 0.0;
float gyro_X = 0.0, gyro_Y = 0.0, gyro_Z = 0.0;

float prev_X = 0.0, prev_Y = 0.0, prev_Z = 0.0;

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);

// Функція ініціалізації MPU6050
void MPU6050_Init(void)
{
	uint8_t addressCheck;
	uint8_t data;

	// Перевірка ID пристрою
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &addressCheck, 1,
			1000);

	if (addressCheck == 0x68)
	{
		// Пробудження датчика
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1,
				1000);

		// Встановлення частоти даних 1KHz
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1,
				1000);

		// AFS_SEL = 0 => +-2g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1,
				1000);

		// FS_SEL = 0 => +-250s
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1,
				1000);
	}
}

void MPU6050_ReadAccel(void)
{
	uint8_t receivedData[6];

	// Зчитування 6 байт даних, починаючи з регістра ACCEL_XOUT_H
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
	ACCEL_XOUT_H_REG, 1, receivedData, 6, 1000);
	if (status != HAL_OK)
	{
		char errorMessage[] = "Error reading accelerometer data with I2C\r\n";
		CDC_Transmit_FS((uint8_t*) errorMessage, strlen(errorMessage));
		return;
	}

	// Об'єднання старшого і молодшого байта координати в 16-бітне значення для отримання сирих даних
	accel_X_Raw = (int16_t) (receivedData[0] << 8 | receivedData[1]);
	accel_Y_Raw = (int16_t) (receivedData[2] << 8 | receivedData[3]);
	accel_Z_Raw = (int16_t) (receivedData[4] << 8 | receivedData[5]);

	// Перетворення сирих значень акселерометра в "g"
	accel_X = accel_X_Raw / 16384.0;
	accel_Y = accel_Y_Raw / 16384.0;
	accel_Z = accel_Z_Raw / 16384.0;
}

void MPU6050_ReadGyro(void)
{
	uint8_t receivedData[6];

	// Зчитування 6 байт даних, починаючи з регістра GYRO_XOUT_H
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
	GYRO_XOUT_H_REG, 1, receivedData, 6, 1000);
	if (status != HAL_OK)
	{
		char errorMessage[] = "Error reading gyroscope data with I2C\r\n";
		CDC_Transmit_FS((uint8_t*) errorMessage, strlen(errorMessage));
		return;
	}

	// Об'єднання старшого і молодшого байта координати в 16-бітне значення для отримання сирих даних
	gyro_X_Raw = (int16_t) (receivedData[0] << 8 | receivedData[1]);
	gyro_Y_Raw = (int16_t) (receivedData[2] << 8 | receivedData[3]);
	gyro_Z_Raw = (int16_t) (receivedData[4] << 8 | receivedData[5]);

	// Перетворення сирих значень гіроскопа в "градус на секунду"
	gyro_X = gyro_X_Raw / 131.0;
	gyro_Y = gyro_Y_Raw / 131.0;
	gyro_Z = gyro_Z_Raw / 131.0;

	// Обчислення кута нахилу на основі даних акселерометра та гіроскопа
	// Якщо не використовувати (180.0 * M_PI) тоді результат буде в радіанах
	MPU6050_Data.X = atan2(accel_Y, accel_Z) * 180.0 / M_PI;
	MPU6050_Data.Y = atan2(-accel_X,
			sqrt(accel_Y * accel_Y + accel_Z * accel_Z)) / 180.0 * M_PI;
	float Z = (gyro_Z) / 131.0;

	// Перевіряється, чи є значення кутової швидкості обертання навколо Z достатнім для корекції X та Y
	MPU6050_Data.Z = Z;
	if (abs(Z) > 0.01)
	{
		float new_Z = sin(Z * M_PI / 180);
		MPU6050_Data.X -= MPU6050_Data.Y * new_Z;
		MPU6050_Data.Y += MPU6050_Data.X * new_Z;
	}
}

void ComplementaryFilter(float accelX, float gyroX, float accelY, float gyroY,
		float accelZ, float gyroZ)
{
	char USB_DataBufer[50];
	// Застосування комплементарного фільтра для згладжування вимірювань акселерометра та гіроскопа
	MPU6050_Data.X = COMP_FILTER_ALPHA * (gyroX + accelX)
			+ (1 - COMP_FILTER_ALPHA) * prev_X;
	MPU6050_Data.Y = COMP_FILTER_ALPHA * (gyroY + accelY)
			+ (1 - COMP_FILTER_ALPHA) * prev_Y;
	MPU6050_Data.Z = COMP_FILTER_ALPHA * (gyroZ + accelZ)
			+ (1 - COMP_FILTER_ALPHA) * prev_Z;

	sprintf(USB_DataBufer, "%1.f; %1.f; %1.f\n", MPU6050_Data.X, MPU6050_Data.Y,
			MPU6050_Data.Z);
	CDC_Transmit_FS((uint8_t*) USB_DataBufer, strlen(USB_DataBufer));
}

void ControlLEDs(float x, float y, float z)
{
	if (x < LEFT_THRESHOLD)
	{
		// Увімкнути червоний світлодіод
		HAL_GPIO_WritePin(GPIOD, Red_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Вимкнути червоний світлодіод
		HAL_GPIO_WritePin(GPIOD, Red_LED_Pin, GPIO_PIN_RESET);
	}
	if (x > RIGHT_THRESHOLD)
	{
		// Увімкнути зелений світлодіод
		HAL_GPIO_WritePin(GPIOD, Green_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Вимкнути зелений світлодіод
		HAL_GPIO_WritePin(GPIOD, Green_LED_Pin, GPIO_PIN_RESET);
	}

	if (y > TOP_THRESHOLD)
	{
		// Увімкнути синій світлодіод
		HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Вимкнути синій світлодіод
		HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin, GPIO_PIN_RESET);
	}

	if (y < BOTTOM_THRESHOLD)
	{
		// Увімкнути помаранчевий світлодіод
		HAL_GPIO_WritePin(GPIOD, Orange_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Вимкнути помаранчевий світлодіод
		HAL_GPIO_WritePin(GPIOD, Orange_LED_Pin, GPIO_PIN_RESET);
	}
}

int extractGPGGA(const char *buffer, char *gpgga_Sentence)
{
	// Знаходимо початок речення GPGGA
	const char *start = strstr(buffer, "$GPGGA");
	if (start)
	{
		// Знаходимо кінець речення GPGGA
		const char *end = strchr(start, '\n');
		if (end)
		{
			size_t len = end - start + 1;
			// Копіюємо речення GPGGA у вихідний буфер
			strncpy(gpgga_Sentence, start, len);
			gpgga_Sentence[len] = '\0';
			return 1;
		}
	}
	return 0;
}

void parseGPGGA(const char *sentence, char *latitude, char *longitude)
{
	char tempLatitude[15], tempLongitude[15];
	char latitudeDirection, longtitudeDirection;

	// Зчитуємо широту та довготу з рядка GPGGA
	if (sscanf(sentence, "$GPGGA,%*f,%[^,],%c,%[^,],%c", tempLatitude,
			&latitudeDirection, tempLongitude, &longtitudeDirection) == 4)
	{
		snprintf(latitude, sizeof(tempLatitude), "%.10s %c", tempLatitude,
				latitudeDirection);
		snprintf(longitude, sizeof(tempLongitude), "%.10s %c", tempLongitude,
				longtitudeDirection);
	}
	else
	{
		strcpy(latitude, "N/A");
		strcpy(longitude, "N/A");
	}
}

int main(void)
{
	char humidity[20];
	char temperature[20];
	uint8_t gpsBuffer[200];
	char gpgga_Sentence[100];
	char latitude[15];
	char longitude[15];

	// Ініціалізація системи та периферії
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	MX_USB_DEVICE_Init();
	MPU6050_Init();
	LCD_Init();

	while (1)
	{
		// Зчитування даних з акселерометра та гіроскопа
		MPU6050_ReadAccel();
		MPU6050_ReadGyro();

		// Обчислення кутів нахилу на основі даних акселерометра
		MPU6050_Data.X = atan2(accel_Y, accel_Z) * 180.0 / M_PI;
		MPU6050_Data.Y = atan2(-accel_X,
				sqrt(accel_Y * accel_Y + accel_Z * accel_Z)) * 180.0 / M_PI;
		MPU6050_Data.Z = 0;

		// Застосування комплементарного фільтра для згладжування даних
		ComplementaryFilter(MPU6050_Data.X, gyro_X, MPU6050_Data.Y, gyro_Y,
				MPU6050_Data.Z, gyro_Z);

		ControlLEDs(MPU6050_Data.X, MPU6050_Data.Y, MPU6050_Data.Z);

		prev_X = MPU6050_Data.X;
		prev_Y = MPU6050_Data.Y;
		prev_Z = MPU6050_Data.Z;

		// Отримання даних з датчика DHT
		DHT_Data dht = DHT_getData();
		float prevTemperature = 0;
		float prevHumidity = 0;

		if (dht.temperature != 0 && dht.humidity != 0
				&& (dht.temperature != prevTemperature
						|| dht.humidity != prevHumidity))
		{
			snprintf(temperature, sizeof(temperature), "Temperature: %1.f",
					dht.temperature);
			snprintf(humidity, sizeof(humidity), "Humidity: %1.f",
					dht.humidity);
			LCD_Clear();
			LCD_PutString(0, 0, temperature); // Відображення температури на першому рядку
			LCD_PutString(0, 1, humidity); // Відображення вологості на другому рядку

			prevTemperature = dht.temperature;
			prevHumidity = dht.humidity;
		}

		// Прийом GPS даних з UART
		HAL_UART_Receive(&huart3, gpsBuffer, sizeof(gpsBuffer), HAL_MAX_DELAY);

		// Забезпечення завершення буфера нульовим байтом
		gpsBuffer[sizeof(gpsBuffer) - 1] = '\0';

		// Витягнення речення GPGGA
		if (extractGPGGA((char*) gpsBuffer, gpgga_Sentence))
		{
			// Розбір витягнутого речення GPGGA
			parseGPGGA(gpgga_Sentence, latitude, longitude);

			// Підготовка повідомлення для відправки через USB
			char gps_Message[75];
			snprintf(gps_Message, sizeof(gps_Message),
					"Latitude: %.15s, Longitude: %.15s\n", latitude, longitude);

			CDC_Transmit_FS((uint8_t*) gps_Message, strlen(gps_Message));
		}
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_I2C1_Init(void)
{
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
}

static void MX_USART3_UART_Init(void)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 57600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
	RS_Pin | RW_Pin | EN_Pin | DB4_Pin | DB5_Pin | DB6_Pin | DB7_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	DHT11_Pin | Green_LED_Pin | Orange_LED_Pin | Red_LED_Pin | Blue_LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : RS_Pin RW_Pin EN_Pin DB4_Pin
	 DB5_Pin DB6_Pin DB7_Pin */
	GPIO_InitStruct.Pin = RS_Pin | RW_Pin | EN_Pin | DB4_Pin | DB5_Pin | DB6_Pin
			| DB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : DHT11_Pin Green_LED_Pin Orange_LED_Pin Red_LED_Pin
	 Blue_LED_Pin */
	GPIO_InitStruct.Pin = DHT11_Pin | Green_LED_Pin | Orange_LED_Pin
			| Red_LED_Pin | Blue_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}
