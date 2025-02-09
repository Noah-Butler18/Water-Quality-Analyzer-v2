/*
 * FinalProjectSTMToArduino.c
 *
 * NOTE: For more information on project hardware, see FinalProjectSTMToArduino.h
 *
 * Project: Embedded Water Quality Analyzer
 * Author: Noah Butler
*/

#include "FinalProjectSTMToArduino.h"

/************************ START: Application global variables ************************/
//Peripheral handles
ADC_Handle_t ADCHandle;
GPIO_Handle_t GPIOHandle;
I2C_Handle_t I2CHandle;

//Flags
__vo uint8_t Flag_All_Values_Converted = 0;
/************************ END: Application global variables ************************/


int main(void)
{
	/************************ Semi-hosting INIT ************************/
#ifdef SEMIHOSTING_ENABLE
	initialise_monitor_handles();
	printf("Application starting...\n");
#endif

	/************************ Peripheral initializations ************************/
	DS18B20_Config();
	GPIO_WQEInitialize();
	ADC_WQEInitialize();
	I2C_WQEInitialize();

	/************************ NVIC/Peripheral interrupt initializations ************************/
	ADC_IRQInterruptConfig(IRQ_NO_ADC, ENABLE);
	ADC_IRQPriorityConfig(IRQ_NO_ADC, NVIC_IRQ_PRIO_1 );

	TIM2_5_IRQInterruptConfig(IRQ_NO_TIM2, ENABLE );
	TIM2_5_IRQPriorityConfig(IRQ_NO_TIM2, NVIC_IRQ_PRIO_2 );

	/************************ TIM interrupt configuration & enable ************************/
	float freq = 0.5;

	// Assert that the TIM interrupt is no more frequent than it takes main loop to execute
	// As of 2-9-25, this threshold is 0.5 seconds
	if( freq > TIM_INTERRUPT_MAX_FREQ)
		while(1);

	TIM2_5_SetIT(TIM2, freq);

	/************************ START: Local variables ************************/
	// Temperature variables
	uint8_t BufferOneWireRawTemperature[2] = {0,0};
	float Temperature = 0.0;
	// ADC buffer
	uint16_t BufferADCValues[NUM_OF_ANALOG_CONVERSIONS] = {0,0};
	//TDS variables
	float VoltsTDS = 0.0;
	uint16_t TDS = 0;
	float TDSTemperatureCompensationCoefficient = 0.0;
	//Turbidity global variables
	float VoltsTurbidity = 0.0;
	float Turbidity = 0;
	//I2C Tx buffer
	uint8_t BufferDataToArduino[I2C_BUFFER_SIZE];

	for(uint8_t temp = 0; temp < I2C_BUFFER_SIZE; temp++)
		BufferDataToArduino[temp] = 0;
	/************************ END: Local variables ************************/

	while(1)
	{

#ifdef SLEEP_MODE_ENABLE
		PWR_SleepUntilInterrupt();
#endif

#ifdef TIMING_TESTING_ENABLE
	TESTING_PIN_WRITE_0();
#endif

		// NOTE: when TIM interrupt is triggered, processor wakes up and starts executing sequentially from this point

		// Get Temperature From DS18B20
		DS18B20_MasterGetTemperature(BufferOneWireRawTemperature);
		Temperature = DS18B20_ConvertTemp(BufferOneWireRawTemperature);

		// ADC interrupt configuration & enable
		ADC_EnableIT(&ADCHandle, BufferADCValues, (ADCHandle.ADC_Config.ADC_Seq_Len) );

		// Block while there are unprocessed ADC interrupts
		while( Flag_All_Values_Converted != 1 )
				;

		// Reset global flag
		Flag_All_Values_Converted = 0;

		// At this point in program flow, all ADC conversions are done.
		// Calculate display values based off of voltages and then store all in buffer to be sent to Arduino via I2C bus
		// NOTE: ADC conversion buffer structure:
			// uint16_t BufferADCValues[0] = | TDS |
			// uint16_t BufferADCValues[1] = | Turbidity |

		// TDS Calculations:
		VoltsTDS = ((float) BufferADCValues[0] * ANALOG_SENSOR_SUPPLY_VOLTAGE) / (float) ADC_DIGITAL_RESOLUTION;
		TDSTemperatureCompensationCoefficient = 1.0 + (0.02 * (Temperature - 25.0));
		TDS = TDS_ConvertVoltageToPPM(VoltsTDS, TDSTemperatureCompensationCoefficient);

		// Turbidity Calculations:
		VoltsTurbidity = ((float) BufferADCValues[1] * ANALOG_SENSOR_SUPPLY_VOLTAGE) / (float) ADC_DIGITAL_RESOLUTION;
		Turbidity = Turbidity_ConvertVoltageToPercentage(VoltsTurbidity);

		// At this point in program flow, all data conversions are done. Need to format data for Arduino and send over I2C
		// NOTE: I2C Tx buffer structure:
			// uint8_t BufferDataToArduino[0-1] | 	Temperature MSB 	| 	2) Temperature LSB 	|
			// uint8_t BufferDataToArduino[2-3] | 	TDS MSB  			| 	TDS LSB 			|
			// uint8_t BufferDataToArduino[4-5] | 	Turbidity (integer) | 	Turbidity (decimal)	|

		BufferDataToArduino[0] = BufferOneWireRawTemperature[0];
		BufferDataToArduino[1] = BufferOneWireRawTemperature[1];
		I2C_ConvertTDSPPMToBytes(TDS, &BufferDataToArduino[2]);
		I2C_ConvertTurbidityPercentageToBytes(Turbidity, &BufferDataToArduino[4]);

		// Send all data to Arduino over I2C bus
		uint32_t len = (sizeof(BufferDataToArduino))/(sizeof(BufferDataToArduino[0]));
		I2C_MasterSendDataToArduino(&BufferDataToArduino[0], len);



#ifdef SEMIHOSTING_ENABLE
		printf("Sent:  | 0x%X | 0x%X | 0x%X | 0x%X | 0x%X | 0x%X |\n", BufferDataToArduino[0], BufferDataToArduino[1], BufferDataToArduino[2], BufferDataToArduino[3], BufferDataToArduino[4], BufferDataToArduino[5] );
		printf("Current water readings: Temp - %.2f°C   TDS - %dppm   Turbidity - %.2f%% \n", Temperature, TDS, Turbidity);
#endif

#ifdef TIMING_TESTING_ENABLE
	TESTING_PIN_WRITE_1();
#endif
	}
}

void TIM2_IRQHandler(void)
{
	// Note: Wakes up processor from sleep mode

	// Clear interrupt flag
	TIM2_5_IRQHandling(TIM2);
}

void ADC_IRQHandler(void)
{
	// Note: Wakes up processor from sleep mode

	// Read values in from TDS sensor first, then turbidity sensor
	ADC_IRQHandling(&ADCHandle);
}

void I2C_MasterSendDataToArduino(uint8_t *Buffer, uint32_t Length)
{
	//1. enable peripheral hardware
	I2C_PeripheralControl(I2CHandle.pI2Cx, ENABLE);

	//2. start comms, send address phase, then send all information. Close comms once finished
	uint8_t SlaveAddr = ARDUINO_I2C_SLAVE_ADDRESS;
	I2C_MasterSendData(&I2CHandle, Buffer, Length, SlaveAddr, 0);

	//3. Disable the I2C peripheral once communication is over
	I2C_PeripheralControl(I2CHandle.pI2Cx,DISABLE);
}


void DS18B20_MasterGetTemperature(uint8_t *BufferReceiveTemperature)
{
	uint8_t TempSensorCommand = 0;
	//1. Master initiates communication sequence (Master Tx) and waits for presence pulse from DS18B20 (Master Rx)
	DS18B20_MasterSendInitializeSequence();

	//2. Master sends skip ROM command since there is only 1 slave on bus (Master Tx)
	TempSensorCommand = MASTER_COMMAND_SKIP_ROM;
	DS18B20_MasterSendData(&TempSensorCommand, 1);

	//3. Master sends convert T command to make DS18B20 start converting (Master Tx)
	TempSensorCommand = MASTER_COMMAND_CONVERT_T;
	DS18B20_MasterSendData(&TempSensorCommand, 1);

	//4. Master continuously sends read time slots to gauge when DS18B20 is finished converting temp (Master Tx)
	//5. Master waits until it receives 1 '1' on the bus notifying it the temperature is ready (Master Rx)
	while( !DS18B20_MasterGenerateReadTimeSlot() )
		;

	//6. Master initiates another communication sequence (Master Tx) and waits for presence pulse from DS18B20 (Master Rx)
	DS18B20_MasterSendInitializeSequence();

	//7. Master sends skip ROM command since there is only 1 slave on bus (Master Tx)
	TempSensorCommand = MASTER_COMMAND_SKIP_ROM;
	DS18B20_MasterSendData(&TempSensorCommand, 1);

	//8. Master sends read scratch pad command (Master Tx)
	TempSensorCommand = MASTER_COMMAND_READ_SCRATCHPAD;
	DS18B20_MasterSendData(&TempSensorCommand, 1);

	//9. Master waits until 2 bytes of data have been sent (Master Rx)
	DS18B20_MasterReceiveData(BufferReceiveTemperature, 2U);

	//10. Master sends a reset pulse to stop reading scratch pad (the temperature is only the first 2 bytes) is (Master Tx)
	DS18B20_MasterSendInitializeSequence();
}


void GPIO_WQEInitialize(void)
{
	//Initialize the ADC input pins - PA1, PA2
	memset(&GPIOHandle,0,sizeof(GPIOHandle));

	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;		//GPIO pin in analog mode for ADC
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;			//GPIO pin in analog mode for TDS ADC (PA1 is free IO)
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		//GPIO output type - don't care
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;		//GPIO output speed - don't care
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;		//No pull up or pull down resistor

	GPIOHandle.pGPIOx = GPIOA;										//Using GPIOA peripheral

	GPIO_Init(&GPIOHandle);

	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;			//GPIO pin in analog mode for Turbidity ADC (PA2 is free IO)

	GPIO_Init(&GPIOHandle);

#ifdef TIMING_TESTING_ENABLE

	//Initialize GPIO pin to measure timings using logic analyzer:
	memset(&GPIOHandle,0,sizeof(GPIOHandle));
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = TIMING_TESTING_GPIO_PIN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIOHandle.pGPIOx = TIMING_TESTING_GPIO_PORT;

	// Start pin out logic HIGH.
	// Any ISR/function/delay timing to measure should have its contents wrapped in the pin low and pin high macros
	// Measuring the time the pin is pulled low gives total duration
	GPIOHandle.pGPIOx->ODR |= ( GPIO_PIN_SET << TIMING_TESTING_GPIO_PIN);

	GPIO_Init(&GPIOHandle);

#endif

	//Initialize I2C pins of the master STM32
	memset(&GPIOHandle,0,sizeof(GPIOHandle)); 		//sets each member element of the structure to zero. Avoids bugs caused by random garbage values in local variables upon first declaration

	GPIOHandle.pGPIOx = GPIOB;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_AF4;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH; //4nS t_fall when pulling line down

	//SCL
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&GPIOHandle);

	//SDA
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&GPIOHandle);
}

void I2C_WQEInitialize(void)
{
	memset(&I2CHandle,0,sizeof(I2CHandle)); 		//sets each member element of the structure to zero. Avoids bugs caused by random garbage values in local variables upon first declaration

	I2CHandle.pI2Cx = I2C1;
	I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2CHandle.I2C_Config.I2C_DeviceAddress = 0x01;
	I2CHandle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;


	I2C_Init(&I2CHandle);
}

void ADC_WQEInitialize(void)
{
	memset(&ADCHandle,0,sizeof(ADCHandle));

	ADCHandle.ADC_Config.ADC_ClkPrescaler = ADC_CLK_DIV_2; 				//ADC clk = 8MHz
	ADCHandle.ADC_Config.ADC_Resolution = ADC_RES_12BITS;					//DR resolution = 12 bits
	ADCHandle.ADC_Config.ADC_DataAlignment = ADC_RIGHT_ALIGNMENT;			//DR alignment = right
	ADCHandle.ADC_Config.ADC_Mode = ADC_SINGLE_CONVERSION_MODE;				//ADC mode
	ADCHandle.ADC_Config.ADC_SamplingTime[ADC_IN1] = ADC_SMP_480_CYCLES;	//Channel 1 sampling time = 480 cycles
	ADCHandle.ADC_Config.ADC_SamplingTime[ADC_IN2] = ADC_SMP_480_CYCLES;	//Channel 2 sampling time = 480 cycles
	ADCHandle.ADC_Config.ADC_AWDHT = 0xFFF;								//High voltage threshold: digital 4095 | analog 3.3V /// digital 2048 | analog 1.65V
	ADCHandle.ADC_Config.ADC_AWDLT = 0x0;									//Low voltage threshold: digital 0 | analog 0V /// digital 2048 | analog 1.65V
	ADCHandle.ADC_Config.ADC_Seq_Len = NUM_OF_ANALOG_CONVERSIONS;			//channel conversion sequence length = 1 channel
	ADCHandle.ADC_Config.ADC_Seq_Order[0] = ADC_IN1;						//ADC channel sequence order = 1) ADC_IN1 - TDS sensor
	ADCHandle.ADC_Config.ADC_Seq_Order[1] = ADC_IN2;						//ADC channel sequence order = 2) ADC_IN2 - Turbidity sensor

	ADCHandle.pADCx = WQE_ADC;												//Using ADC1 peripheral

	ADC_Init(&ADCHandle);
}

uint16_t TDS_ConvertVoltageToPPM(float Voltage, float TemperatureCompensation)
{
	float CompensatedVoltage;
	uint16_t TDSppm;

	CompensatedVoltage = Voltage / TemperatureCompensation; //Temperature Compensation

	//convert voltage value to TDS value using user manual formula
	TDSppm = (	((133.42*CompensatedVoltage*CompensatedVoltage*CompensatedVoltage) - 	\
				(255.86*CompensatedVoltage*CompensatedVoltage) + 						\
				(857.39*CompensatedVoltage)) *											\
				0.5);

	return TDSppm;
}

uint16_t TDS_CalibratePPM(uint16_t UncalibratedTDSPPM)
{
	//Calibration value based off of real life data measuring known TDS value vs sensor TDS value
	//Calibration relationship is a linear function based off of 2 real life data points:
		//(7ppm measured,0ppm known) ; (128ppm measured,707ppm known)
	//This yields the rough relationship: CorrectedValue = 5.84 * MeasuredValue − 41
	if( (5.84 * (int16_t)UncalibratedTDSPPM - 41) < 0 )
	{
		return 0;
	}
	else
		return (5.84 * UncalibratedTDSPPM - 41);
}

float Turbidity_ConvertVoltageToPercentage(float Voltage)
{
	//0V = 3.5% turbidity, 1.53V = 0% turbidity

	//Due to error within the measurements, guard band the readings and say anything above the possible error is 100% clear
	if( Voltage > 1.53 )
	{
		return 0.0;
	}
	else
	{
		float TurbidityPercentage;
		float Step = 3.5 / 1.53; //3.5% is the highest reading capable on sensor, 1.53V is the theoretical highest value the ADC can read from this sensor

		TurbidityPercentage = Voltage * Step;
		TurbidityPercentage = 3.5 - TurbidityPercentage;

		return TurbidityPercentage;
	}
}

void I2C_ConvertTurbidityPercentageToBytes(float TurbidityPercentage, uint8_t *Bufferi2c)
{
	//Takes percentage and stores it into an 8-bit buffer of size 2. The first byte is the whole number part, the second byte is the fractional part
	//i.e.,   2.5%  -->  | 00000010 | 00000101 |

	uint16_t temp;

	temp = TurbidityPercentage*10;

	//Load MSB of buffer
	*(++Bufferi2c) = temp % 10;

	temp /= 10;
	//Load LSB of buffer
	*(--Bufferi2c) = temp % 10;
}

void I2C_ConvertTDSPPMToBytes(uint16_t TDSPPM, uint8_t *Bufferi2c)
{
	//First byte is MSB, second byte is LSB
	*Bufferi2c = (TDSPPM >> 8) & 0xFF;
	*(++Bufferi2c) = TDSPPM & 0xFF;
}

#ifdef SLEEP_MODE_ENABLE
/*********************** Function Documentation ***************************************
 *
 	 * @fn			- PWR_WFIEnterSleepMode
 	 * @brief  		- Puts ARM Cortex M-4 processor into sleep mode, saving power during downtime.
 	 * @brief  		-  The Cortex-M4 processor sleep modes reduce power consumption. The sleep modes your
						device implements are implementation-defined. The modes can be one or both of the following:
						• sleep mode stops the processor clock (Turns off processor clock)
						• deep sleep mode stops the system clock and switches off the PLL and flash memory. (NOT USED IN THIS API)
 	 * @param 		- none
 	 * @retval 		- none
 	 * @Note		- ENTRY INTO SLEEP MODE: Enter SLEEP mode with WFI instruction
 	 * @Note		- EXIT OUT OF SLEEP MODE: Any peripheral interrupt acknowledged by the nested vectored interrupt
              	  		controller (NVIC) can wake up the device from Sleep mode.
*/
void PWR_SleepUntilInterrupt(void)
{
	/* Clear SLEEPDEEP bit of Cortex System Control Register to ensure we are not entering deep sleep, just sleep mode */
	__vo uint32_t *ARMCM4_SystemControlReg = ARM_CM4_SCR_ADDR;
	uint32_t Sleep_Mask = 2;
	*ARMCM4_SystemControlReg &= ~(1 << Sleep_Mask);

#ifdef SEMIHOSTING_ENABLE
	printf("Processor entering sleep mode\n");
#endif
	/* Execute WFI instruction on processor to enter sleep mode */
	__asm volatile ("wfi");
	/* Processor clock will wake up after an interrupt to the NVIC */
}
#endif

void ADC_ApplicationEventCallBack(ADC_Handle_t *pADCHandle, uint8_t AppEvent)
{
	//User implementation of ADC_ApplicationEventCallBack API

	if( AppEvent == ADC_ERROR_STRT )
	{
		//Application attempting starting conversion with one already in progress
#ifdef SEMIHOSTING_ENABLE
		printf("ADC repeated start condition detected - suspending program...\n");
#endif
		while(1);
	}

	if( AppEvent == ADC_EVENT_AWD )
	{
		//Analog watch dog threshold triggered
		pADCHandle->pADC_DataBuffer--;
		uint16_t *pThresholdValue = pADCHandle->pADC_DataBuffer;

		if( *pThresholdValue > ( pADCHandle->pADCx->HTR ) )
		{
#ifdef SEMIHOSTING_ENABLE
			printf("Analog watch dog triggered - over voltage condition detected.\n");
#endif
		}
		else if( *pThresholdValue < ( pADCHandle->pADCx->LTR ) )
		{
#ifdef SEMIHOSTING_ENABLE
			printf("Analog watch dog triggered - under voltage condition detected.\n");
#endif
		}

#ifdef SEMIHOSTING_ENABLE
		float VoltsAWD = ( *pThresholdValue / (float) ADC_DIGITAL_RESOLUTION ) * ANALOG_SENSOR_SUPPLY_VOLTAGE;
		printf("\nCurrent voltage reading: %f\n\n", VoltsAWD);
		printf("Shutting down ADC...\n");
#endif
		ADC_PeripheralOnOffControl(pADCHandle->pADCx, DISABLE);

		while(1);
	}

	if( AppEvent == ADC_EVENT_OVR )
	{
		//Overrun triggered

#ifdef SEMIHOSTING_ENABLE
		printf("Overrun condition detected - suspending program...\n");
#endif

		while(1);
	}

	if( AppEvent == ADC_EVENT_EOC )
	{
		//For multiple channels, this implementation uses single conversion mode, manually changing channel selection between conversions

		// Stop the ADC after each conversion. The ADC peripheral clock is still on, it just goes into power down mode
		pADCHandle->pADCx->CR2 &= ~( 1 << ADC_CR2_ADON );

		// If we are done converting all channels, we need to reinitialize the order with user configuration values
		if( pADCHandle->ADC_SeqLen == 0 )
		{
			ADC_SequenceInit(pADCHandle);

			Flag_All_Values_Converted = 1;
		}
		else
		{
			// Change the channel selected to be converted in single channel mode to the next channel.
			pADCHandle->pADCx->SQR3 >>= 5;

			// Turn on ADC to keep converting channels
			TURN_ON_ADC();

			// Set ADC to start converting - if already converting, issue an application event
			if( ADC_GetFlagStatus(pADCHandle->pADCx, ADC_FLAG_STRT) )
				ADC_ApplicationEventCallBack(pADCHandle, ADC_ERROR_STRT);

			// Start the ADC again
			START_ADC();
		}
	}
}
