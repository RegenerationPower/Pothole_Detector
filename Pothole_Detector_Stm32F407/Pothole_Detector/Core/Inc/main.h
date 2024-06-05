#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOE
#define RW_Pin GPIO_PIN_10
#define RW_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_11
#define EN_GPIO_Port GPIOE
#define DB4_Pin GPIO_PIN_12
#define DB4_GPIO_Port GPIOE
#define DB5_Pin GPIO_PIN_13
#define DB5_GPIO_Port GPIOE
#define DB6_Pin GPIO_PIN_14
#define DB6_GPIO_Port GPIOE
#define DB7_Pin GPIO_PIN_15
#define DB7_GPIO_Port GPIOE
#define DHT11_Pin GPIO_PIN_11
#define DHT11_GPIO_Port GPIOD
#define Green_LED_Pin GPIO_PIN_12
#define Green_LED_GPIO_Port GPIOD
#define Orange_LED_Pin GPIO_PIN_13
#define Orange_LED_GPIO_Port GPIOD
#define Red_LED_Pin GPIO_PIN_14
#define Red_LED_GPIO_Port GPIOD
#define Blue_LED_Pin GPIO_PIN_15
#define Blue_LED_GPIO_Port GPIOD

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
