#ifndef DHT11_H_
#define DHT11_H_

#include "main.h"

typedef struct
{
	float humidity;
	float temperature;
} DHT_data;

DHT_data DHT_getData(void);

#endif
