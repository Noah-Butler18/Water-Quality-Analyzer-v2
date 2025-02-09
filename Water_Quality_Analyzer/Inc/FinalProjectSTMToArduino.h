/*
 * FinalProjectSTMToArduino.h
 *
 * Project: Embedded Water Quality Analyzer
 * Author: Noah Butler
*/

#ifndef FINALPROJECTSTMTOARDUINO_H_
#define FINALPROJECTSTMTOARDUINO_H_

/*
 * HARDWARE CONNECTIONS:
 * --MCUs--
 * 		STM32F407
 *			GND <-> Breadboard GND rail
 *			+5V supply <-> Breadboard 5V supply rail
 *			+3.3V (Vdd) supply <-> Breadboard 3.3V supply rail
 *			PA1 <-> TDS sensor analog voltage (to ADC)
 *			PA2 <-> Turbidity sensor analog voltage (to ADC)
 *			PA3 <-> DQ (DS18B20) (hanging off 4.7kOhm pull up resistor connected to +Vdd)
 *			PB6 <-> SCLK (i2c to Arduino) ~~~ Use 5V to 3.3V logic level converter to interface between the 2 boards~~~
 *			PB7 <-> SDA (i2c to Arduino) ~~~ Use 5V to 3.3V logic level converter to interface between the 2 boards~~~
 *		Arduino
 *			GND <-> Breadboard GND rail
 *			+5V supply <-> Small +5V rail
 *			A0 <-> pH sensor analog voltage (to ADC)
 *			A1 <-> EC sensor analog voltage (to ADC)
 *			A5 <-> SCLK ~~~ Use 5V to 3.3V logic level converter to interface between the 2 boards~~~
 *			A4 <-> SDA ~~~ Use 5V to 3.3V logic level converter to interface between the 2 boards~~~
 *			Digital Pin 2 <-> BLE module UART RX
 *			Digital Pin 3 <-> BLE module UART TX
 * --Sensors--
 * 		TDS sensor
 * 			VCC <-> 3.3V
 * 			GND <-> GND
 * 			Analog signal <-> PA1 (STM32)
 * 		Turbidity sensor
 * 			VCC <-> 3.3V
 * 			GND <-> GND
 * 			Analog signal <-> PA2 (STM32)
 * 		DS18B20 (temperature sensor)
 * 			VCC <-> 3.3V
 * 			GND <-> GND
 * 			DQ <-> PA3 (STM32) (hanging off 4.7kOhm pull up resistor connected to +Vdd)
 * 		pH sensor
 * 			VCC <-> 5V (From Arduino supply)
 * 			GND <-> GND
 * 			GND <-> GND
 * 			Analog signal <-> A0 (Arduino)
 * 		EC sensor
 * 			VCC <-> 5V (From Arduino supply)
 * 			GND <-> GND
 * 			Analog signal <-> A1 (Arduino)
 * --Connectivity--
 * 		3.3V/5V Logic Level Shifter
 * 			GND <-> GND rail
 *			HV <-> +5V rail (supplied by STM32)
 * 			LV <-> +3.3V rail (supplied by STM32)
 * 			HV1 <-> A5 (Arduino I2C SCL)
 *			HV2 <-> A4 (Arduino I2C SDA)
 *			LV1 <-> PB6 (STM32 I2C SCL)
 *			LV2 <-> PB7 (STM32 I2C SDA)
 * 		BLE Module
 * 			GND <-> GND
 * 			VCC <-> 5V (From Arduino supply)
 * 			TXD (UART) <-> Arduino digital Pin 2 (UART RX)
 * 			RXD (UART) <-> Arduino digital Pin 3 (UART TX)
 */

#include "stm32f407vg.h"
#include "ds18b20_temp_sensor.h"

#define NUM_OF_ANALOG_CONVERSIONS				2			//TDS and Turbidity
#define ARM_CM4_SCR_ADDR						( (__vo uint32_t *) 0xE000ED10 )
#define ANALOG_SENSOR_SUPPLY_VOLTAGE			( (float) 3.3 )
#define WQE_ADC 								ADC1
#define ADC_DIGITAL_RESOLUTION					4095
#define ARDUINO_I2C_SLAVE_ADDRESS				(0x68)
#define I2C_BUFFER_SIZE							6
#define TIM_INTERRUPT_MAX_FREQ					2 // 	1/TIM_INTERRUPT_MAX_FREQ = time between interrupts

/*-------------------- START: MACRO FUNCTIONS --------------------*/
#define TURN_ON_ADC() 							( WQE_ADC->CR2 |= (1 << ADC_CR2_ADON) )
#define START_ADC() 							( WQE_ADC->CR2 |= (1 << ADC_CR2_SWSTART) )
/*-------------------- END: MACRO FUNCTIONS --------------------*/

/*-------------------- START: FUNCTION PROTOTYPES --------------------*/
void I2C_MasterSendDataToArduino(uint8_t *Buffer, uint32_t Length);
void DS18B20_MasterGetTemperature(uint8_t *BufferReceiveTemperature);
void I2C_WQEInitialize(void);
void GPIO_WQEInitialize(void);
void ADC_WQEInitialize(void);
uint16_t TDS_ConvertVoltageToPPM(float Voltage, float TemperatureCompensation);
uint16_t TDS_CalibratePPM(uint16_t UncalibratedTDSPPM);
float Turbidity_ConvertVoltageToPercentage(float Voltage);
void I2C_ConvertTurbidityPercentageToBytes(float TurbidityPercentage, uint8_t *Bufferi2c);
void I2C_ConvertTDSPPMToBytes(uint16_t TDSPPM, uint8_t *Bufferi2c);
/*-------------------- END: FUNCTION PROTOTYPES --------------------*/

/*-------------------- START: USER CONFIG ITEMS --------------------*/
/*
 * TODO: Comment out "#define SEMIHOSTING_ENABLE" if not using Semihosting
 *
 * -- NOTE: If semihosting is ENABLED, follow these steps:
 * 1. Add argument to linker: "-specs=rdimon.specs -lc -lrdimon" to link newlib nano standard library that supports semihosting (librdimon_nano.a)
 * 2. Src/syscalls.c must be excluded from build - system calls are implemented by librdimon_nano.a (library supports semihosting)
 * 3. Add "extern void initialise_monitor_handles();" to top of main.c and "iinitialise_monitor_handles(); somewhere in main() - inits semihosting
 * 4. Add printf() statements throughout code. All must end in newline character
 * 5. Build project
 * 6. Debug using OpenOCD configuration and "monitor arm semihosting enable" in startup
 *
 *  * -- NOTE: If semihosting is DISABLED, follow these steps:
 * 1. Remove argument to linker: "-specs=rdimon.specs -lc -lrdimon" to link newlib nano standard library (libc_nano.a)
 * 2. Src/syscalls.c must be included in build
 * 3. Remove "extern void initialise_monitor_handles();" to top of main.c and "iinitialise_monitor_handles();
 * 4. Remove printf() statements throughout code (unless you are using some other implementation of printf such as ITM->SWD)
 * 5. Build project
 * 6. Debug using STLINK GDB server configuration and remove "monitor arm semihosting enable" in startup if present
*/
#define SEMIHOSTING_ENABLE
#ifdef SEMIHOSTING_ENABLE

//Enable semihosting
extern void initialise_monitor_handles(void);

#endif

/*
 * TODO: Comment out "#define SLEEP_MODE_ENABLE" if not using power saving
 * Macro used to test power usage with power saving enabled vs disabled
 *
 * NOTE: To test power, connect an ammeter across the leads of JP1
 * 		see: https://www.st.com/resource/en/user_manual/um1472-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf
*/
//#define SLEEP_MODE_ENABLE
#ifdef SLEEP_MODE_ENABLE

void PWR_SleepUntilInterrupt(void);

#endif

/*
 * TODO: Comment out "#define TIMING_TESTING_ENABLE" if not using GPIO pins to test timings
 * Macro can be used in conjunction with a logic analyzer to test timings of ISR latency, comms, etc.
 * Any ISR/function/delay timing to measure should have its contents wrapped in the pin low and pin high macros
 * Measuring the time the pin is pulled low gives total duration
 *		Example:
 *			Start: Interrupt request happens and program enters ISR. In ISR:
 *				1. TESTING_PIN_WRITE_0();
 *				2. *ISR does stuff*
 *				3. TESTING_PIN_WRITE_1();
 *			End: ISR exits. Logic analyzer captures trace and software can be used to measure timing
 *
 * NOTE: Make sure IO pin is free:
 * 		https://www.st.com/resource/en/user_manual/um1472-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf
*/
#define TIMING_TESTING_ENABLE
#ifdef TIMING_TESTING_ENABLE

#define TIMING_TESTING_GPIO_PORT				( GPIOA )
#define TIMING_TESTING_GPIO_PIN					8

#define TESTING_PIN_WRITE_1() 		( TIMING_TESTING_GPIO_PORT->ODR |= ( 1 << TIMING_TESTING_GPIO_PIN ) )
#define TESTING_PIN_WRITE_0() 		( TIMING_TESTING_GPIO_PORT->ODR &= ~( 1 << TIMING_TESTING_GPIO_PIN ) )

#endif

/*-------------------- END: USER CONFIG ITEMS --------------------*/

#endif /* FINALPROJECTSTMTOARDUINO_H_ */
