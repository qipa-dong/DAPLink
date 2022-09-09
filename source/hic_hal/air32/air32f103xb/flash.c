/**
 * @file    flash_hal_stm32f103xb.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "flash_hal.h"        // FlashOS Structures
#include "target_config.h"    // target_device
#include "air32f10x.h"
#include "util.h"
#include "string.h"
#include "target_board.h"

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    //
    // No special init required
    //
    return (0);
}

uint32_t UnInit(uint32_t fnc)
{
    //
    // No special uninit required
    //
    return (0);
}

uint32_t EraseChip(void)
{
    uint32_t ret = 0;  // O.K.
    if (g_board_info.target_cfg) {
        FLASH_Unlock();
        //bootloader, interface flashing only concerns 1 flash region
        for(uint32_t i=0;i< (g_board_info.target_cfg->flash_regions[0].end - g_board_info.target_cfg->flash_regions[0].start) % 0x400; i++)
        {
            if (FLASH_ErasePage(g_board_info.target_cfg->flash_regions[0].start + i*0x400) != FLASH_COMPLETE) {
                ret = 1;
                break;
            }
        }
        
        FLASH_Lock();
    }else{
        ret = 1;
    }
    return ret;
}

uint32_t EraseSector(uint32_t adr)
{
    uint32_t ret = 0;  // O.K.

    FLASH_Unlock();

    if (FLASH_ErasePage(adr) != FLASH_COMPLETE) {
        ret = 1;
    }

    FLASH_Lock();
    return ret;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t i;
    uint32_t ret = 0;  // O.K.

    FLASH_Unlock();

    util_assert(sz % 4 == 0);
    for (i = 0; i < sz / 4; i++) {
        if (FLASH_ProgramWord(adr + i * 4, buf[i]) != FLASH_COMPLETE) {
            ret = 1;
            break;
        }
    }

    FLASH_Lock();
    return ret;
}
