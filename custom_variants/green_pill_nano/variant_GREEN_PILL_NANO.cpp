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
#if defined(ARDUINO_GENERIC_L072CBTX) || defined(ARDUINO_GENERIC_L072CBUX) || \
    defined(ARDUINO_GENERIC_L072CZTX) || defined(ARDUINO_GENERIC_L072CZUX) || \
    defined(ARDUINO_GENERIC_L073CBTX) || defined(ARDUINO_GENERIC_L073CBUX) || \
    defined(ARDUINO_GENERIC_L073CZTX) || defined(ARDUINO_GENERIC_L073CZUX) || \
    defined(ARDUINO_GENERIC_L082CZUX) || defined(ARDUINO_GENERIC_L083CBTX) || \
    defined(ARDUINO_GENERIC_L083CZTX) || defined(ARDUINO_GENERIC_L083CZUX)
#include "pins_arduino.h"

// Digital PinName array
const PinName digitalPin[] = {
    PA_10,  // D0 - Serial RX / TIM1_CH3
    PA_9,   // D1 - Serial TX / TIM1_CH2
    PA_0,   // D2, A8 - TIM2_CH1
    PA_1,   // D3, A9 - TIM2_CH3
    PA_3,   // D4, A10
    PB_10,  // D5 - TIM3_CH3 / ADC_IN8
    PB_1,   // D6, A11 - TIM3_CH4
    PA_8,   // D7 - TIM1_CH1
    PB_2,   // D8
    PB_11,  // D9 - TIM3_CH1 / ADC_IN6
    PA_15,  // D10 - SPI1_NSS / TIM2_CH1
    PB_5,   // D11 - SPI1_MOSI / TIM3_CH2
    PB_4,   // D12 - SPI1_MISO
    PB_3,   // D13 - SPI1_SCK / LED / TIM2_CH2
    PA_2,   // A0
    PA_5,   // A!
    PA_7,   // A2
    PA_4,   // A3
    PB_7,   // A4
    PB_6,   // A5
    PA_6,   // A6  
    PB_0    // A7
  };

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
14,
15,
16,
17,
18,
19,
20,
21,
2,
3
};

#endif /* ARDUINO_GENERIC_* */
