#include "main.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Define LED pins
#define LED3_ORANGE GPIO_PIN_13 // Orange LED
#define LED4_GREEN GPIO_PIN_12  // Green LED
#define LED5_RED GPIO_PIN_14     // Red LED
#define LED6_BLUE GPIO_PIN_15    // Blue LED

// Define orientation thresholds
#define LEFT_THRESHOLD -10
#define RIGHT_THRESHOLD 10
#define TOP_THRESHOLD 10
#define BOTTOM_THRESHOLD -10

#define COMP_FILTER_ALPHA 0.98 // Коэффициент комплиментарного фильтра (от 0 до 1)

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

typedef struct {
  float X;	// X or Roll
  float Y;	// Y or Pitch
  float Z;	// Z or Yaw
} sensorData;

sensorData MPU6050_Data;

float prevX = 0.0;
float prevY = 0.0;
float prevZ = 0.0;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

int _write(int file, char* ptr, int len)
{
  int i;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
  for (i = 0; i < len; i++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void MPU6050_Init(void)
{
  uint8_t check;
  uint8_t data;

  // check device ID WHO_AM_I

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

  if (check == 104) // 0x68 will be returned by the sensor if everything goes well
  {
    // power management register 0X6B we should write all 0's to wake the sensor up
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

    // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

    // Set accelerometer configuration in ACCEL_CONFIG Register
    // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
    data = 0x00;
    // Data = 0x10; // 2 => +-8g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
    data = 0x00;
    // Data = 0x8; // 1 => +-500s
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
  }
}

void MPU6050_Read_Accel(void)
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from ACCEL_XOUT_H register

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
  if (status != HAL_OK) {
    printf("Error reading accelerometer data with I2C\r\n");
    return;
  }

  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

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
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
  if (status != HAL_OK) {
    printf("Error reading gyroscope data with I2C\r\n");
    return;
  }
  Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  // To convert raw gyroscope values to value in "degree per second"
  // It is necessary to also divide them by the value set in the FS_SEL register
  // We set FS_SEL to 0, so we divide the raw values by 131.0

  Gx = Gyro_X_RAW / 131.0;
  Gy = Gyro_Y_RAW / 131.0;
  Gz = Gyro_Z_RAW / 131.0;

  MPU6050_Data.X = atan2(Ay, Az) * 180.0 / M_PI;

  // If use without (180.0 * M_PI) then the result will be in radians not in degrees

  MPU6050_Data.Y = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) / 180.0 * M_PI;

  float Z = (Gz) / 131.0;
  MPU6050_Data.Z = Z;
  if (abs(Z) > 0.01) {
    float _Z = sin(Z * M_PI / 180);
    MPU6050_Data.X -= MPU6050_Data.Y * _Z;
    MPU6050_Data.Y += MPU6050_Data.X * _Z;
  }
}

void ComplementaryFilter(float accelX, float gyroX, float accelY, float gyroY, float accelZ, float gyroZ)
{
  // Apply a complementary filter to smooth out accelerometer and gyroscope measurements
  MPU6050_Data.X = COMP_FILTER_ALPHA * (gyroX + accelX) + (1 - COMP_FILTER_ALPHA) * prevX;
  MPU6050_Data.Y = COMP_FILTER_ALPHA * (gyroY + accelY) + (1 - COMP_FILTER_ALPHA) * prevY;
  MPU6050_Data.Z = COMP_FILTER_ALPHA * (gyroZ + accelZ) + (1 - COMP_FILTER_ALPHA) * prevZ;

  //printf("X: %f, Y: %f, Z: %f\r\n", MPU6050_Data.X, MPU6050_Data.Y, MPU6050_Data.Z);
  printf("%1.f; %1.f; %1.f\n", MPU6050_Data.X, MPU6050_Data.Y, MPU6050_Data.Z);
  //printf("Message ok\n");
  //printf("%1.f\r\n", MPU6050_Data.X);
  //printf("%1.f\r\n", MPU6050_Data.Y);
  //printf("%1.f\r\n", MPU6050_Data.Z);
}

void ControlLEDs(float x, float y, float z)
{
    if (x < LEFT_THRESHOLD)
    {
        // Turn on green LED
        HAL_GPIO_WritePin(GPIOD, LED4_GREEN, GPIO_PIN_SET);
    }
    else
    {
        // Turn off green LED
        HAL_GPIO_WritePin(GPIOD, LED4_GREEN, GPIO_PIN_RESET);
    }

    if (x > RIGHT_THRESHOLD)
    {
        // Turn on red LED
        HAL_GPIO_WritePin(GPIOD, LED5_RED, GPIO_PIN_SET);
    }
    else
    {
        // Turn off red LED
        HAL_GPIO_WritePin(GPIOD, LED5_RED, GPIO_PIN_RESET);
    }

    if (y > TOP_THRESHOLD)
    {
        // Turn on blue LED
        HAL_GPIO_WritePin(GPIOD, LED6_BLUE, GPIO_PIN_SET);
    }
    else
    {
        // Turn off blue LED
        HAL_GPIO_WritePin(GPIOD, LED6_BLUE, GPIO_PIN_RESET);
    }

    if (y < BOTTOM_THRESHOLD)
    {
        // Turn on orange LED
        HAL_GPIO_WritePin(GPIOD, LED3_ORANGE, GPIO_PIN_SET);
    }
    else
    {
        // Turn off orange LED
        HAL_GPIO_WritePin(GPIOD, LED3_ORANGE, GPIO_PIN_RESET);
    }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MPU6050_Init();
  //printf("X, Y, Z\r\n");
  while (1) {
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();
    MPU6050_Data.X = atan2(Ay, Az) * 180.0 / M_PI;
    MPU6050_Data.Y = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / M_PI;
    MPU6050_Data.Z = 0;

    ComplementaryFilter(MPU6050_Data.X, Gx, MPU6050_Data.Y, Gy, MPU6050_Data.Z, Gz);

    ControlLEDs(MPU6050_Data.X, MPU6050_Data.Y, MPU6050_Data.Z);

    prevX = MPU6050_Data.X;
    prevY = MPU6050_Data.Y;
    prevZ = MPU6050_Data.Z;

    // TEMP
    HAL_Delay(100);
    //printf("TEXT");
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
