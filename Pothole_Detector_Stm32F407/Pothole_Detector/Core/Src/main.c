#include "main.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Define LED orientation thresholds
#define LEFT_THRESHOLD -10
#define RIGHT_THRESHOLD 10
#define TOP_THRESHOLD 10
#define BOTTOM_THRESHOLD -10

#define COMP_FILTER_ALPHA 0.98

typedef struct
{
	float X;
	float Y;
	float Z;
} sensorData;

sensorData MPU6050_Data;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

float prevX = 0.0;
float prevY = 0.0;
float prevZ = 0.0;

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void MPU6050_Init(void)
{
	uint8_t check;
	uint8_t data;

	// Check device ID
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 104) // 0x68 will be returned if everything is ok
	{
		// Wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1,
				1000);

		// Set DATA RATE of 1KHz
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

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
	ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	if (status != HAL_OK)
	{
		char errorMessage[] = "Error reading accelerometer data with I2C\r\n";
		CDC_Transmit_FS((uint8_t*) errorMessage, strlen(errorMessage));
		return;
	}

	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	// To convert raw accelerometer values to value in "g"
	// It is necessary to also divide them by the value set in the FS_SEL register
	// We set FS_SEL to 0, so we divide the raw values by 16384.0
	Ax = Accel_X_RAW / 16384.0;
	Ay = Accel_Y_RAW / 16384.0;
	Az = Accel_Z_RAW / 16384.0;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
	GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	if (status != HAL_OK)
	{
		char errorMessage[] = "Error reading gyroscope data with I2C\r\n";
		CDC_Transmit_FS((uint8_t*) errorMessage, strlen(errorMessage));
		return;
	}

	Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	// To convert raw gyroscope values to value in "degree per second"
	// It is necessary to also divide them by the value set in the FS_SEL register
	// We set FS_SEL to 0, so we divide the raw values by 131.0
	Gx = Gyro_X_RAW / 131.0;
	Gy = Gyro_Y_RAW / 131.0;
	Gz = Gyro_Z_RAW / 131.0;

	// If use without (180.0 * M_PI) then the result will be in radians not in degrees
	MPU6050_Data.X = atan2(Ay, Az) * 180.0 / M_PI;
	MPU6050_Data.Y = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) / 180.0 * M_PI;

	float Z = (Gz) / 131.0;
	MPU6050_Data.Z = Z;
	if (abs(Z) > 0.01)
	{
		float _Z = sin(Z * M_PI / 180);
		MPU6050_Data.X -= MPU6050_Data.Y * _Z;
		MPU6050_Data.Y += MPU6050_Data.X * _Z;
	}
}
void ComplementaryFilter(float accelX, float gyroX, float accelY, float gyroY,
		float accelZ, float gyroZ)
{
	char USB_DataBufer[50];
	// Apply a complementary filter to smooth out accelerometer and gyroscope measurements
	MPU6050_Data.X = COMP_FILTER_ALPHA * (gyroX + accelX)
			+ (1 - COMP_FILTER_ALPHA) * prevX;
	MPU6050_Data.Y = COMP_FILTER_ALPHA * (gyroY + accelY)
			+ (1 - COMP_FILTER_ALPHA) * prevY;
	MPU6050_Data.Z = COMP_FILTER_ALPHA * (gyroZ + accelZ)
			+ (1 - COMP_FILTER_ALPHA) * prevZ;

	sprintf(USB_DataBufer, "%1.f; %1.f; %1.f\n", MPU6050_Data.X, MPU6050_Data.Y,
			MPU6050_Data.Z);
	CDC_Transmit_FS((uint8_t*) USB_DataBufer, strlen(USB_DataBufer));
}

void ControlLEDs(float x, float y, float z)
{
	if (x < LEFT_THRESHOLD)
	{
		// Turn on red LED
		HAL_GPIO_WritePin(GPIOD, Red_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Turn off red LED
		HAL_GPIO_WritePin(GPIOD, Red_LED_Pin, GPIO_PIN_RESET);
	}
	if (x > RIGHT_THRESHOLD)
	{
		// Turn on green LED
		HAL_GPIO_WritePin(GPIOD, Green_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Turn off green LED
		HAL_GPIO_WritePin(GPIOD, Green_LED_Pin, GPIO_PIN_RESET);
	}

	if (y > TOP_THRESHOLD)
	{
		// Turn on blue LED
		HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Turn off blue LED
		HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin, GPIO_PIN_RESET);
	}

	if (y < BOTTOM_THRESHOLD)
	{
		// Turn on orange LED
		HAL_GPIO_WritePin(GPIOD, Orange_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Turn off orange LED
		HAL_GPIO_WritePin(GPIOD, Orange_LED_Pin, GPIO_PIN_RESET);
	}
}

int main(void)
{
	char humidity[20];
	char temperature[20];

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	MPU6050_Init();
	LCD_Init();

	while (1)
	{
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		MPU6050_Data.X = atan2(Ay, Az) * 180.0 / M_PI;
		MPU6050_Data.Y = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / M_PI;
		MPU6050_Data.Z = 0;

		ComplementaryFilter(MPU6050_Data.X, Gx, MPU6050_Data.Y, Gy,
				MPU6050_Data.Z, Gz);

		ControlLEDs(MPU6050_Data.X, MPU6050_Data.Y, MPU6050_Data.Z);

		prevX = MPU6050_Data.X;
		prevY = MPU6050_Data.Y;
		prevZ = MPU6050_Data.Z;

		DHT_data dht = DHT_getData();
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
			LCD_PutString(0, 0, temperature); // Print temperature on the first line
			LCD_PutString(0, 1, humidity); // Print humidity on the second line

			prevTemperature = dht.temperature;
			prevHumidity = dht.humidity;
		}

		// TEMP
		HAL_Delay(100);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
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

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
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
