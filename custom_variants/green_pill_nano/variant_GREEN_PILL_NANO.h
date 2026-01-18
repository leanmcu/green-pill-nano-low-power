/*
 *******************************************************************************
 * Copyright (c) 2020, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
/*
modified by LeanMCU to support Green Pill Nano board
*/
#pragma once

/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
#define PA10  0         // D0 - Serial RX / TIM1_CH3
#define PA9   1         // D1 - Serial TX / TIM1_CH2
#define PA0   PIN_A8    // D2 - TIM2_CH1
#define PA1   PIN_A9    // D3 - TIM2_CH3
#define PA3   4         // D4
#define PB10  5         // D5 - TIM3_CH3 / ADC_IN8
#define PB1   6         // D6 - TIM3_CH4
#define PA8   7         // D7 - TIM1_CH1
#define PB2   8         // D8
#define PB11  9         // D9 - TIM3_CH1 / ADC_IN6
#define PA15  10        // D10 - SPI1_NSS / TIM2_CH1
#define PB5   11        // D11 - SPI1_MOSI / TIM3_CH2
#define PB4   12        // D12 - SPI1_MISO
#define PB3   13        // D13 - SPI1_SCK / LED / TIM2_CH2
#define PA2   PIN_A0
#define PA5   PIN_A1
#define PA7   PIN_A2
#define PA4   PIN_A3
#define PB7   PIN_A4
#define PB6   PIN_A5
#define PA6   PIN_A6
#define PB0   PIN_A7

// Alternate pins number
#define PA2_ALT1 (PA2 | ALT1)
#define PA3_ALT1 (PA3 | ALT1)
#define PA6_ALT1 (PA6 | ALT1)
#define PA7_ALT1 (PA7 | ALT1)
#define PA14_ALT1 (PA14 | ALT1)
#define PB4_ALT1 (PB4 | ALT1)
#define PB5_ALT1 (PB5 | ALT1)

#define NUM_DIGITAL_PINS 22
#define NUM_ANALOG_INPUTS 10

// On-board LED pin number
#ifndef LED_BUILTIN
#define LED_BUILTIN PB3
#endif

// On-board user button
#ifndef USER_BTN
#define USER_BTN PNUM_NOT_DEFINED
#endif

// SPI definitions
#ifndef PIN_SPI_SS
#define PIN_SPI_SS            PA15
#endif
#ifndef PIN_SPI_SS1
#define PIN_SPI_SS1 PA15
#endif
#ifndef PIN_SPI_SS2
#define PIN_SPI_SS2 PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS3
#define PIN_SPI_SS3 PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_MOSI
#define PIN_SPI_MOSI          PB5
#endif
#ifndef PIN_SPI_MISO
#define PIN_SPI_MISO          PB4
#endif
#ifndef PIN_SPI_SCK
#define PIN_SPI_SCK           PB3
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
#define PIN_WIRE_SDA PB7
#endif
#ifndef PIN_WIRE_SCL
#define PIN_WIRE_SCL PB6
#endif

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
#define TIMER_TONE TIM6
#endif
#ifndef TIMER_SERVO
#define TIMER_SERVO TIM7
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
#define SERIAL_UART_INSTANCE 1
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
#define PIN_SERIAL_RX PA10
#endif
#ifndef PIN_SERIAL_TX
#define PIN_SERIAL_TX PA9
#endif

// Extra HAL modules
#if !defined(HAL_DAC_MODULE_DISABLED)
#define HAL_DAC_MODULE_ENABLED
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#ifndef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR Serial
#endif
#ifndef SERIAL_PORT_HARDWARE
#define SERIAL_PORT_HARDWARE Serial
#endif
#endif
