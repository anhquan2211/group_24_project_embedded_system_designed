#include "dht.h"
#include "delay_timer.h"

/*
 * 				   Tgo			   Treh									TH0 or TH1
 *  __           ______          _______                              ___________________________			______
 *    \    A    /      \   C    /       \   DHT duration_data_low    /                           \		   /
 *     \_______/   B    \______/    D    \__________________________/   DHT duration_data_high    \_______/
 *		  Tbe			   Trel				T_LOW													Ten
 */

#define Tbe	20
#define Tgo	35
#define Trel 88
#define Treh 92
#define T_LOW 58
#define TH0	27
#define TH1	74
#define Ten	56

#define DHT_TIMER_INTERVAL 2

/***************************************************************************************************************/

/**
  * @brief  This function provides minimum delay (in microseconds)
  *    		Blocking mode based on a timer with function TIM_DelayUs().
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains the configuration information for the specified DHT sensor.
  * @param  Time: specifies the delay time length, in microseconds.
  * @retval None
  */
static void DHT_DelayUs(DHT_HandleTypeDef* DHT, uint16_t Time)
{
	TIM_DelayUs(DHT->Timer, Time);
}

/**
  * @brief  This function provides minimum delay (in miliseconds)
  *    		Blocking mode based on a timer with function TIM_DelayUs().
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @param  Time: specifies the delay time length, in miliseconds.
  * @retval None
  */
static void DHT_DelayMs(DHT_HandleTypeDef* DHT, uint16_t Time)
{
	TIM_DelayMs(DHT->Timer, Time);
}

/**
  * @brief  Initializes the GPIOx pin for sensor DHT according to Open-Drain Output mode.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @retval None
  */
static void DHT_SetPinOut(DHT_HandleTypeDef* DHT)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
	HAL_GPIO_WritePin(DHT->Port, DHT->Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = DHT->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT->Port, &GPIO_InitStruct);
}

/**
  * @brief  Initializes the GPIOx pin for sensor DHT according to NoPull Input mode.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @retval None
  */
static void DHT_SetPinIn(DHT_HandleTypeDef* DHT)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT->Port, &GPIO_InitStruct);
}

/**
  * @brief  Sets or clears the selected data port bit of DHT sensor pin.
  *
  * @note   This function uses HAL_GPIO_WritePin() function.
  *
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @param  PinState: specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
static void DHT_WritePin(DHT_HandleTypeDef* DHT, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(DHT->Port, DHT->Pin, PinState);
}

/**
  * @brief  Reads the specified input port pin of DHT sensor pin.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  *
  * @retval The input port pin value.
  */
static GPIO_PinState DHT_ReadPin(DHT_HandleTypeDef* DHT)
{
	GPIO_PinState PinState;
	PinState =  HAL_GPIO_ReadPin(DHT->Port, DHT->Pin);
	return PinState;
}

/***************************************************************************************************************/

/**
  * @brief  Waits the specified data port bit of DHT sensor pin.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @param  timeout: Timeout duration
  * @param  expected_pin_state: The expected port pin value of sensor pin.
  * @param  duration: Pointer to a uint32_t variable.
  * @retval DHT status:
  * 			DHT_OK: read the expected port pin value on sensor pin.
  * 			DHT_ERR_TIMEOUT: timeout but not read the expected port pin value on sensor pin.
  */
static dht_err_t DHT_AwaitPinState(DHT_HandleTypeDef* DHT, uint32_t timeout,
									GPIO_PinState expected_pin_state, uint32_t* duration)
{
	DHT_SetPinIn(DHT);
    for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL)
    {
        // need to wait at least a single interval to prevent reading a jitter
    	DHT_DelayUs(DHT, DHT_TIMER_INTERVAL);
        if (DHT_ReadPin(DHT) == expected_pin_state)
        {
        	if(duration)
        		*duration = i;
            return DHT_OK;
        }
    }

    return DHT_ERR_TIMEOUT;
}

/**
  * @brief  Fetch temperature and relative humidity data for DHT sensor.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @param  data[DHT_DATA_BYTES]: Pointer to a uint8_t array.
  * @retval DHT status
  */
static dht_err_t DHT_FetchData(DHT_HandleTypeDef* DHT, uint8_t data[DHT_DATA_BYTES]) {
	uint32_t low_duration;
	uint32_t high_duration;

	DHT_SetPinOut(DHT);
	DHT_WritePin(DHT, GPIO_PIN_RESET);
	DHT_DelayMs(DHT, Tbe);
	DHT_WritePin(DHT, GPIO_PIN_SET);

	int ret = DHT_AwaitPinState(DHT, Tgo, GPIO_PIN_RESET, NULL);
	if(ret) {
		return DHT_ERR_PHASE_B;
	}


	ret = DHT_AwaitPinState(DHT, Trel, GPIO_PIN_SET, NULL);
	if(ret) {
		return DHT_ERR_PHASE_C;
	}

	ret = DHT_AwaitPinState(DHT, Treh, GPIO_PIN_RESET, NULL);
	if(ret) {
		return DHT_ERR_PHASE_D;
	}

	for (int i = 0; i < DHT_DATA_BITS; ++i) {
		ret = DHT_AwaitPinState(DHT, T_LOW, GPIO_PIN_SET, &low_duration);
		if (ret) {
			return DHT_ERR_LBIT_TIMEOUT;
		}
		ret = DHT_AwaitPinState(DHT, TH1, GPIO_PIN_RESET, &high_duration);
		if (ret) {
			return DHT_ERR_HBIT_TIMEOUT;
		}

		uint8_t byte = i / 8;
		uint8_t bit = i % 8;
		if (!bit) {
		     data[byte] = 0;
		}
		data[byte] |= (high_duration > low_duration) << (7 - bit);
	}

	return DHT_OK;
}

/***************************************************************************************************************/

/**
  * @brief  Initializes the DHT_HandleTypeDef according to the specified parameters.
  * @note	maximum execution time: Tbe + Tgo + Trel + Treh + 40*(T_LOW + TH1) + Ten
  * 								20ms+ 35us+ 88us + 92us + 40*( 58us + 74us)+ 56us
  * 								~ 26ms
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @param  DHT_Port: Pointer to a GPIO_InitTypeDef structure that is specified for
  * 		DHT sensor port.
  * @param  DHT_Pin: GPIO_PIN_xx that is specified for DHT sensor pin.
  * 		DHT sensor pin.
  * @retval None
  */
void DHT_Init(DHT_HandleTypeDef* DHT, TIM_HandleTypeDef* Timer, GPIO_TypeDef* DHT_Port, uint16_t DHT_Pin)
{
	DHT->Error = DHT_OK;
	DHT->Port = DHT_Port;
	DHT->Pin = DHT_Pin;
	DHT->Timer = Timer;
	DHT_SetPinOut(DHT);

}

/**
  * @brief  Fetch temperature and relative humidity data for DHT sensor
  * 		and Check CRC, Convert temperature and RH data to Float.
  * @param  DHT: Pointer to a DHT_HandleTypeDef structure that contains
  * 		the configuration information for the specified DHT sensor.
  * @retval DHT status
  */
dht_err_t DHT_ReadTempHum(DHT_HandleTypeDef* DHT)
{
	uint8_t data[DHT_DATA_BYTES] = {0};

	DHT_SetPinOut(DHT);
	DHT_WritePin(DHT, GPIO_PIN_SET);

	int ret = DHT_FetchData(DHT, data);
	if(ret) {
		DHT->Error = ret;
		return ret;
	}

	for (int i = 0; i < DHT_DATA_BYTES; i++) {
		DHT->raw_data[i] = data[i];

	}

	if(data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
		DHT->Error = DHT_ERR_INVALID_CRC;
		return DHT_ERR_INVALID_CRC;
	}

	DHT->Temp = data[2] + (data[3]&0x7F)/10.0;
	if(data[3]&0x80) {
		DHT->Temp = - DHT->Temp;
	}
	DHT->Humi = data[0] + data[1]/10.0;

	DHT->Error = DHT_OK;
	return DHT_OK;
}

