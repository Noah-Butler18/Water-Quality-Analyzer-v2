/*
 * ds18b20_temp_sensor.c
 *
 *  Created on: Aug 8, 2024
 *      Author: butle
 */

#include "ds18b20_temp_sensor.h"

// NOTE: Driver based off of DS18B20 Chip Datasheet:
// https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_Config

 	 * @brief  		- API configures GPIO pin to send and receive serial data over 1-wire protocol

 	 * @param 		- none

 	 * @retval 		- none

 	 * @Note		- Data line requires external 4.7kOhm resistor
*/
void DS18B20_Config(void)
{
	GPIO_Handle_t DS18B20_pin;

	memset(&DS18B20_pin,0,sizeof(DS18B20_pin));

	//Configure DQ pin
	DS18B20_pin.pGPIOx = DS18B20_GPIO_PORT;

	DS18B20_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;							//Pin will start out as input, then switch between input / output as comms. progress
	DS18B20_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;					//As required by Maxim’s exclusive 1-Wire bus protocol. Data line requires a PU resisotr of 4.7k Ohms
	DS18B20_pin.GPIO_PinConfig.GPIO_PinPuPdControl = DS18B20_GPIO_PIN_NO_PUPD;		//Using external 4.7kOhm resistor
	DS18B20_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH; 					//4nS t_fall when pulling line down
	DS18B20_pin.GPIO_PinConfig.GPIO_PinNumber = DS18B20_GPIO_PIN;

	GPIO_Init(&DS18B20_pin);

	TIM2_5_SetDelayInit(DS18B20_TIM_PERIPHERAL);

}

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_MasterSendInitializeSequence

 	 * @brief  		- API that master device uses to generate an initialization sequence on the 1-wire bus
 	 * @brief  		- All transactions on the 1-Wire bus begin with an initialization sequence
 	 * @brief  		- From: https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf

 	 * @param 		- none

 	 * @retval 		- none

 	 * @Note		- none
*/
void DS18B20_MasterSendInitializeSequence(void)
{
	//1. Master send reset pulse - send logic low on bus
	DS18B20_SET_PIN_OUTPUT();
	DS18B20_PIN_WRITE_0();

	//2. Wait 480us
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_TX_RESET_HOLD_USECS);

	//3. DS18B20 waits 15-60us to send a presence pulse
	// 	 DS18B20's presence pulse is logic 0 that is held for 60-180us
	//	 Master then reads presence pulse and confirms slave device is ready or not
	//   Thus, master can set pin input and wait 60us until ready to read.
	DS18B20_SET_PIN_INPUT();
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_RX_PRESENCE_PULSE_USECS);

	// Time elapsed at this point: ~540+us

	//5. Master confirms presence pulse was sent from DS18B20
	while( ( DS18B20_PIN_READ() ) != GPIO_PIN_SET )
		;

	//6. Fulfill 1-wire requirement of master Rx phase being at least 480us
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_RX_PRESENCE_HOLD_USECS);

	// Time elapsed at this point: ~960+us
}

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_MasterGenerateWriteTimeSlot

 	 * @brief  		- API that master device uses to generate a write time slot on the 1-wire bus
 	 * @brief  		- All write time slots must be a minimum of 60µs in duration with a minimum of a 1µs recovery time
 	 * @brief  		- From: https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf

 	 * @param 		- WriteValue: Either 1 or 0 representing data bit on serial line

 	 * @retval 		- none

 	 * @Note		- none
*/
inline void DS18B20_MasterGenerateWriteTimeSlot(uint8_t WriteValue)
{
	//1. Master pulls 1-wire bus low and releases within 15us (Bus is released below when data pin is set to input
	DS18B20_SET_PIN_OUTPUT();
	DS18B20_PIN_WRITE_0();

	if( WriteValue )
	{
		//2. If generating a write '1' time slot, release bus within 15us but wait at least 1us.
		//   Pull-up resistor will automatically pull bus up to HIGH
		TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_TX_RELEASE_BUS_DELAY_USECS);
		DS18B20_SET_PIN_INPUT();
	}

	//3. Wait until end of write time slot for DS18B20 to sample the data bus (minimum 60us)
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_TX_TIMESLOT_HOLD_USECS);

	//4. Release bus and wait recovery time in-between read or write time slots
	DS18B20_SET_PIN_INPUT();
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_TX_RX_RECOVERY_HOLD_USECS);
}

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_MasterGenerateReadTimeSlot

 	 * @brief  		- API that master device uses to generate a read time slot on the 1-wire bus
 	 * @brief  		- From: https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf

 	 * @param 		- none

 	 * @retval 		- Read bit: Value on bus that was received by 1-wire master

 	 * @Note		- none
*/
inline uint8_t DS18B20_MasterGenerateReadTimeSlot(void)
{
	//1. Master pulls 1-wire bus low for at least 1us then and releases
	DS18B20_SET_PIN_OUTPUT();
	DS18B20_PIN_WRITE_0();
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_RX_INITIATE_USECS);
	DS18B20_SET_PIN_INPUT();

	//2. DS18B20 has transmitted either a 1 or 0. Data is valid for at most 15us, so master should sample data before then
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, MASTER_RX_SAMPLE_USECS);
	uint8_t val = DS18B20_PIN_READ();

	//3. All read time slots are a minimum of 60 us
	uint32_t Recovery_Microseconds = 1;
	TIM2_5_Delay(DS18B20_TIM_PERIPHERAL, (MASTER_RX_TIMESLOT_HOLD_USECS + Recovery_Microseconds));
	return val;
}

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_MasterSendData

 	 * @brief  		- API that master device uses to transmit serial data over 1-wire bus

 	 * @param 		- TxBuffer: pointer to buffer that contains transmit data bytes
 	 * @param 		- len: Length of transmit buffer in bytes

 	 * @retval 		- none

 	 * @Note		- none
*/
void DS18B20_MasterSendData(uint8_t *TxBuffer, uint8_t len)
{
	//1. Begin sending data over bus bit by bit, LSB first
	//NOTE: data is transmitted least significant to most significant over bus

	uint8_t i, temp, temp2;
	while( len )
	{
		//temporary variable to save contents of buffer
		temp = *TxBuffer;

		for( i = 0 ; i < 8U ; i++ )
		{
			temp2 = temp & 0x1U;
			DS18B20_MasterGenerateWriteTimeSlot( temp2 );
			temp >>= 1U;
		}

		len--;
		TxBuffer++;
	}
}

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_MasterReceiveData

 	 * @brief  		- API that master device uses to receive serial data over 1-wire bus

 	 * @param 		- RxBuffer: Pointer to buffer that contains receive data bytes (modified by function)
 	 * @param 		- len: Length of receive buffer in bytes

 	 * @retval 		- none

 	 * @Note		- none
*/
void DS18B20_MasterReceiveData(uint8_t *RxBuffer, uint8_t len)
{
	//1. Begin reading data over bus bit by bit, LSB first
	//NOTE: data is transmitted least significant to most significant over bus
	//Strategy: start at the end of the buffer. load the first bit into the LSB of the buffer, second bit into the next LSB, etc.

	RxBuffer += (len - 1U);
	uint8_t temp, i;

	while( len )
	{
		//temporary variable to save contents of read
		temp = 0;

		//Save bits of DS18B20 registers (going LSB to MSB) in proper order by shifting each bit into correct position
		for( i = 0 ; i < 8U ; i++ )
		{
			temp |= ( ( DS18B20_MasterGenerateReadTimeSlot() ) << i );
		}

		//2. Save contents of temp buffer into user Rx buffer
		*RxBuffer = temp;

		len--;
		RxBuffer--;
	}
}

/*********************** Function Documentation ***************************************
 	 * @fn			- DS18B20_ConvertTemp

 	 * @brief  		- API that converts DS18B20 digital temperature reading to a floating-point human readable temperature value

 	 * @param 		- TempBuffer: Buffer pointer to 2 bytes of digital reading from temperature sensor

 	 * @retval 		- Temperature: (in degrees Celsius)

 	 * @Note		- none
*/
float DS18B20_ConvertTemp(uint8_t *TempBuffer)
{
	//This assumes a 2-element array is passed with MSB byte first then LSB byte last
	uint16_t temperature = (*TempBuffer) << 8U;
	temperature |= *(++TempBuffer);

	return ( (float) temperature / 16.0 );
}

