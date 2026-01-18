/*
 * SPDX-License-Identifier: GPL-2.0-only OR LicenseRef-Commercial
 *
 * Copyright (c) 2025 LeanMCU
 *
 * This file is part of <PROJECT NAME>.
 *
 * Dual Licensing Notice:
 *
 * This source code is licensed under the terms of the GNU General Public
 * License version 2 only (GPL-2.0-only), as published by the Free Software
 * Foundation.
 *
 * Alternatively, this software may be licensed under a separate commercial
 * license obtained from the copyright holder. Use of this software in
 * proprietary or closed-source products, or under terms incompatible with
 * the GPL-2.0-only, requires a valid commercial license.
 *
 * Commercial licensing inquiries:
 *   leanmcu(at)gmail(dot)com
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License version 2 for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 along with this program. If not, see:
 *   https://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#include <Arduino.h>
#include "LeanMCU_Setup.h"
#include "LeanMCU_LowPower.h"
#include "STM32LowPower.h"
#include "STM32RTC.h"

#define BUTTON_PIN 3
void Set_Rtc(void);
void AlarmMatch(void *data);
void Button_ISR();

uint32_t _sleepDuration = 10; // sleep time in seconds
volatile bool _buttonPressed = false;
volatile uint32_t _nextAlarmTime = 0;
STM32RTC &_rtc = STM32RTC::getInstance();

void setup()
{
  Setup_Board();
  Disable_GPIO();
  Disable_Peripherals();
  Disable_RTC();
  // uncomment line below for periodic wake up every _sleepDuration seconds
  Set_Rtc();
  // uncomment the 2 lines below for button wake up from sleep mode
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), Button_ISR, FALLING);
  LowPower.begin();
  // uncomment the 2 lines below for periodic wake up every _sleepDuration seconds
  LowPower.enableWakeupFrom(&_rtc, AlarmMatch, &_sleepDuration);
  _rtc.setAlarmEpoch(_rtc.getEpoch() + _sleepDuration);
}

void loop()
{
  if (_buttonPressed)
  {
    _buttonPressed = false;
    // Indicate wakeup by button press
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    delay(115);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    delay(115);
    delay(1000);
  }
  else // periodic wakeup by RTC alarm
  {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(900);
  }

  pinMode(LED_BUILTIN, INPUT_ANALOG);
  _buttonPressed = false;
  // Enter deep sleep mode
  LowPower.deepSleep();
  // Clear Wakeup flag
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void Set_Rtc(void)
{
  _rtc.setClockSource(STM32RTC::LSE_CLOCK);
  _rtc.begin();
  _rtc.setTime(0, 0, 0);     // Set time to 00:00:00
  _rtc.setDate(1, 1, 1, 26); // Set date to 1st January 2026
  RTC_HandleTypeDef *hrtc = _rtc.getHandle();

  HAL_PWR_EnableBkUpAccess();                  // Enable backup domain access
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW); // Set LSE drive capability to low (saves power)
  __HAL_RTC_TAMPER1_DISABLE(hrtc);             // Disable Tamper1
  __HAL_RTC_TAMPER2_DISABLE(hrtc);             // Disable Tamper2 
  __HAL_RTC_TAMPER3_DISABLE(hrtc);             // Disable Tamper3
  __HAL_RTC_ALARMB_DISABLE(hrtc);              // Disable Alarm B
  HAL_PWR_DisableBkUpAccess();                 // Disable backup domain access after configuration
  HAL_RTCEx_DeactivateWakeUpTimer(hrtc);       // Disable the WakeUp timer
}

void AlarmMatch(void *data)
{
  uint32_t interval_sec = 0;
  uint32_t current_ms;

  if (data != NULL)
  {
    interval_sec = *(uint32_t *)data;
  }

  if (_nextAlarmTime == 0) // If this is the first run, initialize _nextAlarmTime
  {
    _nextAlarmTime = _rtc.getEpoch(&current_ms);
  }
  _nextAlarmTime += interval_sec;
  _rtc.setAlarmEpoch(_nextAlarmTime, STM32RTC::MATCH_DHHMMSS);
}

void Button_ISR()
{
  _buttonPressed = true;
}
