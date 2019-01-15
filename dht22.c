#include "dht22.h"
#include "gpio.h"
#include "stdio.h"

#define DEBUG 1
#if DEBUG
#define MSG(args...) fprintf(stderr, args) /* message that is destined to the user */
#else
#define MSG(args...)
#endif

Gpio_t DHT22_SDA;
TIM_HandleTypeDef DHT22_TIM;
dht22_data dht22_data_t;

void DHT22_Init (void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	DHT22_TIM.Instance = DELAY_US_TIM;
	DHT22_TIM.Init.CounterMode = TIM_COUNTERMODE_UP;
	DHT22_TIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	DHT22_TIM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	DHT22_TIM.Init.Prescaler = 71;
	DHT22_TIM.Init.Period = 65535;
	if (HAL_TIM_Base_Init(&DHT22_TIM) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&DHT22_TIM, &sClockSourceConfig) != HAL_OK)
  {
		_Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&DHT22_TIM, &sMasterConfig) != HAL_OK)
  {
		_Error_Handler(__FILE__, __LINE__);
  }
	HAL_TIM_Base_Start(&DHT22_TIM);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

static uint8_t DHT22_GetReadings (dht22_data *out)
{
	uint8_t i;
	uint16_t c;

	// Switch pin to output
	GpioInit( &DHT22_SDA, PA_12, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, SET );
	
	// Generate start impulse
	GpioWrite( &DHT22_SDA, RESET );
	HAL_Delay(2);
	GpioWrite( &DHT22_SDA, SET );

	// Switch pin to input with Pull-Up
	GpioInit( &DHT22_SDA, PA_12, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );

	// Wait for AM2302 to begin communication (20-40us)
	DELAY_US_TIM->CNT = 0;
	while (((c = DELAY_US_TIM->CNT) < 100)
	  && (DHT22_GPIO_PORT->IDR & DHT22_GPIO_PIN))
	;
	
	DELAY_US_TIM->CNT = 0;
	if (c >= 100)
	{
		MSG("DHT22_RCV_NO_RESPONSE \n");
		MSG("DELAY_US_TIM->CNT = %d \n", c);
		return DHT22_RCV_NO_RESPONSE;
	}
	

	// Check ACK strobe from sensor
	while (((c = DELAY_US_TIM->CNT) < 100)
	  && !(DHT22_GPIO_PORT->IDR & DHT22_GPIO_PIN))
	;

	DELAY_US_TIM->CNT = 0;

	if ((c < 65) || (c > 95))
	{
		MSG("DHT22_RCV_BAD_ACK1 \n");
		MSG("DELAY_US_TIM->CNT = %d \n", c);
		return DHT22_RCV_BAD_ACK1;
	}

	while (((c = DELAY_US_TIM->CNT) < 100)
	  && (DHT22_GPIO_PORT->IDR & DHT22_GPIO_PIN))
	;

	DELAY_US_TIM->CNT = 0;

	if ((c < 65) || (c > 95))
	{
		MSG("DHT22_RCV_BAD_ACK2 \n");
		MSG("DELAY_US_TIM->CNT = %d \n", c);
		return DHT22_RCV_BAD_ACK2;
	}
	

	// ACK strobe received --> receive 40 bits
	i = 0;

	while (i < 40)
	{
	  // Measure bit start impulse (T_low = 50us)

	  while (((c = DELAY_US_TIM->CNT) < 100)
	  && !(DHT22_GPIO_PORT->IDR & DHT22_GPIO_PIN))
	;

	  if (c > 75)
	{
	  // invalid bit start impulse length
	  out->bits[i] = 0xff;
	  while (((c = DELAY_US_TIM->CNT) < 100)
		  && (DHT22_GPIO_PORT->IDR & DHT22_GPIO_PIN))
		;
	  DELAY_US_TIM->CNT = 0;
	}
	  else
	{
	  // Measure bit impulse length (T_h0 = 25us, T_h1 = 70us)
	  DELAY_US_TIM->CNT = 0;
	  while (((c = DELAY_US_TIM->CNT) < 100)
		  && (DHT22_GPIO_PORT->IDR & DHT22_GPIO_PIN))
		;
	  DELAY_US_TIM->CNT = 0;
	  out->bits[i] = (c < 100) ? (uint8_t) c : 0xff;
	}

	  i++;
	}

	for (i = 0; i < 40; i++)
	if (out->bits[i] == 0xff)
	  return DHT22_RCV_TIMEOUT;

	return DHT22_RCV_OK;
}

static void DHT22_DecodeReadings (dht22_data *out)
{
	uint8_t i = 0;

	for (; i < 8; i++)
	{
	  out->hMSB <<= 1;
	  if (out->bits[i] > 48)
	out->hMSB |= 1;
	}

	for (; i < 16; i++)
	{
	  out->hLSB <<= 1;
	  if (out->bits[i] > 48)
	out->hLSB |= 1;
	}

	for (; i < 24; i++)
	{
	  out->tMSB <<= 1;
	  if (out->bits[i] > 48)
	out->tMSB |= 1;
	}

	for (; i < 32; i++)
	{
	  out->tLSB <<= 1;
	  if (out->bits[i] > 48)
	out->tLSB |= 1;
	}

	for (; i < 40; i++)
	{
	  out->parity_rcv <<= 1;
	  if (out->bits[i] > 48)
	out->parity_rcv |= 1;
	}

	out->parity = out->hMSB + out->hLSB + out->tMSB + out->tLSB;

}

static uint16_t DHT22_GetHumidity (dht22_data *out)
{
	return (out->hMSB << 8) | out->hLSB;
}

static uint16_t DHT22_GetTemperature (dht22_data *out)
{
	return (out->tMSB << 8) | out->tLSB;
}

bool DHT22_Read (dht22_data *out)
{
	out->rcv_response = DHT22_GetReadings (out);
	if (out->rcv_response != DHT22_RCV_OK)
	{
	  return false;
	}

	DHT22_DecodeReadings (out);

	if (out->parity != out->parity_rcv)
	{
	  out->rcv_response = DHT22_BAD_DATA;
	  return false;
	}

	out->humidity = (float) DHT22_GetHumidity (out) / 10.0f;

	uint16_t temperature = DHT22_GetTemperature (out);
	out->temperature = ((float) (temperature & 0x7fff)) / 10.0f;

	if (temperature & 0x8000)
	{
	  out->temperature = -out->temperature;
	}

	return true;

}
