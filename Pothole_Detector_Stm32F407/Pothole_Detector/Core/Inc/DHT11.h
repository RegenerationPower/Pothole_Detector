#ifndef DHT11_H_
#define DHT11_H_

#include "main.h"

typedef struct
{
	float humidity;
	float temperature;
} DHT_Data;

DHT_Data DHT_getData(void);

#endif /* DHT11_H_ */
