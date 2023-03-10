#ifndef __DHT_H
#define __DHT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef uint16_t dht_err_t;
#define DHT_OK 0
#define DHT_ERR_TIMEOUT 0x101
#define DHT_ERR_INVALID_CRC 0x102
#define DHT_ERR_PHASE_B 0x103
#define DHT_ERR_PHASE_C 0x104
#define DHT_ERR_PHASE_D 0x105
#define DHT_ERR_LBIT_TIMEOUT 0x106
#define DHT_ERR_HBIT_TIMEOUT 0x107

#define DHT_DATA_BITS 40
#define DHT_DATA_BYTES (DHT_DATA_BITS/8)

#define DHT11_MAX_TEMPERATURE 60	// degree celsius
#define DHT11_MIN_TEMPERATURE -20	// degree celsius
#define DHT11_MAX_RELATIVE_HUMIDITY 95	// percent %
#define DHT11_MIN_RELATIVE_HUMIDITY 5	// percent %

/**
  * @brief  DHT sensor handle Structure definition
  * @member Error:
  *
  */
typedef struct
{
	dht_err_t Error;
	TIM_HandleTypeDef* Timer;
	uint16_t Pin;
	GPIO_TypeDef* Port;
	uint8_t raw_data[DHT_DATA_BYTES];
	float Temp;
	float Humi;
} DHT_HandleTypeDef;

/**
  * @brief  Initializes the DHT_HandleTypeDef according to the specified parameters.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @param  DHT_Port: Pointer to a GPIO_InitTypeDef structure that is specified for
  * 		DHT sensor port.
  * @param  DHT_Pin: GPIO_PIN_xx that is specified for DHT sensor pin.
  * 		DHT sensor pin.
  * @retval None
  */
void DHT_Init(DHT_HandleTypeDef* DHT, TIM_HandleTypeDef* Timer, GPIO_TypeDef* DHT_Port, uint16_t DHT_Pin);

/**
  * @brief  Fetch temperature and relative humidity data for DHT sensor
  * 		and Check CRC, Convert temperature and RH data to Float.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @retval DHT status
  */
dht_err_t DHT_ReadTempHum(DHT_HandleTypeDef* DHT);

#ifdef __cplusplus
}
#endif

#endif /* __DHT_H */
