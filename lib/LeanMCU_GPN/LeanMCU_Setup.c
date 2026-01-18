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

#include "LeanMCU_Setup.h"

void Setup_Board(void)
{
#ifdef DEBUG
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();    // Enable debug mode in stop mode
    HAL_DBGMCU_EnableDBGStandbyMode(); // Enable debug mode in standby mode
    __HAL_RCC_DBGMCU_CLK_ENABLE();
    delay(1000); // Wait for serial monitor to open
#else
    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();    // Disable debug mode in stop mode
    HAL_DBGMCU_DisableDBGStandbyMode(); // Disable debug mode in standby mode
    __HAL_RCC_DBGMCU_CLK_DISABLE();
#endif
    HAL_PWREx_EnableUltraLowPower();      // Enable ULP
    HAL_PWREx_EnableFastWakeUp();         // fast wakeup
    __HAL_FLASH_SLEEP_POWERDOWN_ENABLE(); // Enable flash sleep power down

    // Enable TIM3 remap for PB5 PWM output
    RCC->IOPENR |= RCC_IOPENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->OR &= ~(3 << TIM3_OR_TI1_RMP_Pos);
    TIM3->OR |= (2 << TIM3_OR_TI1_RMP_Pos);
}