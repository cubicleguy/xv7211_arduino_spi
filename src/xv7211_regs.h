/**************************************************************************/
/*!
    @file     xv7211_regs.h

    Epson XV7211 specific register definitions

    @section  HISTORY

    v1.0 - First release restructure

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2025 Seiko Epson Corporation.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
    OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Gyro SPI
namespace XV7211_NS {

// Register Addresses
constexpr uint8_t ADDR_DSP_CTRL2 = 0x02;
constexpr uint8_t ADDR_STS_RD = 0x04;
constexpr uint8_t ADDR_STATUS = 0x05;
constexpr uint8_t ADDR_PROT_STATE = 0x06;
constexpr uint8_t ADDR_TEMP_RD = 0x08;
constexpr uint8_t ADDR_DAT_ACC_ON = 0x0A;
constexpr uint8_t ADDR_OUT_CTRL1 = 0x0B;
constexpr uint8_t ADDR_SEL_FSR = 0x0C;
constexpr uint8_t ADDR_PAGE_SEL = 0x0F;
constexpr uint8_t ADDR_DAT_LATCH_COM = 0x15;
constexpr uint8_t ADDR_PROT_SOFT_R = 0x1A;
constexpr uint8_t ADDR_SOFT_RESET = 0x1B;

// DSP_CTRL2
constexpr uint8_t CMD_NF_DISABLE = 0x00;
constexpr uint8_t CMD_NF_ENABLE = 0x01;

constexpr uint8_t CMD_LPF_ORDER_2ND = 0x00;
constexpr uint8_t CMD_LPF_ORDER_3RD = 0x01;
constexpr uint8_t CMD_LPF_ORDER_4TH = 0x02;

constexpr uint8_t CMD_LPFC_HZ_001 = 0x00;
constexpr uint8_t CMD_LPFC_HZ_010 = 0x01;
constexpr uint8_t CMD_LPFC_HZ_025 = 0x02;
constexpr uint8_t CMD_LPFC_HZ_050 = 0x03;
constexpr uint8_t CMD_LPFC_HZ_100 = 0x04;
constexpr uint8_t CMD_LPFC_HZ_200 = 0x05;
constexpr uint8_t CMD_LPFC_HZ_400 = 0x06;
constexpr uint8_t CMD_LPFC_HZ_500 = 0x07;

// STATUS / PROTSTATE
constexpr uint8_t CMD_SLP_OUT = 0x00;
constexpr uint8_t CMD_SLP_IN = 0x01;
constexpr uint8_t CMD_SLP_CTRL_ENB = 0x59;

// ADDR_OUT_CTRL1
constexpr uint8_t CMD_DIS_IMU_LATCH = 0x00;
constexpr uint8_t CMD_ENB_IMU_LATCH = 0x01;

constexpr uint8_t CMD_DIS_CMD_TRG = 0x00;
constexpr uint8_t CMD_ENB_CMD_TRG = 0x01;

constexpr uint8_t CMD_TFORMAT_128LSB = 0x00;
constexpr uint8_t CMD_TFORMAT_256LSB = 0x01;

constexpr uint8_t CMD_DATA_FORMAT_16B = 0x00;
constexpr uint8_t CMD_DATA_FORMAT_24B = 0x01;

// ADDR_SEL_FSR
constexpr uint8_t CMD_FSR_FULL = 0x00;
constexpr uint8_t CMD_FSR_QTR = 0x02;

// ADDR_DAT_LATCH_COM
constexpr uint8_t CMD_DAT_LATCH = 0x01;

// ADDR_PROT_SOFT_R / SOFTRESET
constexpr uint8_t CMD_SOFT_RESET_ENB = 0x59;
constexpr uint8_t CMD_SOFT_RESET = 0x01;

}  // namespace XV7211_NS