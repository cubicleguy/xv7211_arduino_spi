/**************************************************************************/
/*!
    @file     xv7211_spi.h

    Header for Epson XV7211 Class

    @section  HISTORY

    v1.0 - First release

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

#include "xv7211_regs.h"
#include "spi_com.h"

#include <cstring>

namespace XV7211_NS {

// Global angular read byte length
constexpr uint32_t GLOBAL_24B_LEN = 9;
constexpr uint32_t GLOBAL_16B_LEN = 6;

// Gyro scale factor depends on DataFormat bit 2 in OutCtrl1 register
// LSB/dps
constexpr float GYRO_SF16 = 1.0f / 264;
constexpr float GYRO_SF16DIV2 = 1.0f / (264 / 2);
constexpr float GYRO_SF24 = 1.0f / 67584;
constexpr float GYRO_SF24DIV2 = 1.0f / (67584 / 2);

// TempC scale factor depends on TFormat bit 3 in OutCtrl1 register
// LSB/degC
constexpr float TEMPC_SF1 = 1.0f / 128;
constexpr float TEMPC_SF2 = 1.0f / 256;

// Milliseconds, delay after power-on startup for serial interface
constexpr unsigned long TIF_DELAY_MS = 1;

// Milliseconds, delay after power-on startup for temperature output
constexpr unsigned long TSEN_DELAY_MS = 80;

// Milliseconds, delay after power-on startup for angular rate output
constexpr unsigned long TSTA_DELAY_MS = 250;

// Microseconds, delay after SLPIN command
constexpr unsigned long TSLPIN_DELAY_US = 10;

// Microseconds, delay after SLPOUT command
constexpr unsigned long TSLPOUT_DELAY_US = 10;

// Microseconds, delay after SWRESET command
constexpr unsigned long TSWRST_DELAY_US = 10;

constexpr uint16_t BIT_0 = (1);
constexpr uint16_t BIT_1 = (1 << 1);
constexpr uint16_t BIT_2 = (1 << 2);
constexpr uint16_t BIT_3 = (1 << 3);
constexpr uint16_t BIT_4 = (1 << 4);
constexpr uint16_t BIT_5 = (1 << 5);
constexpr uint16_t BIT_6 = (1 << 6);
constexpr uint16_t BIT_7 = (1 << 7);

}  // namespace XV7211_NS

// Contains Epson sensor initialization settings
struct InitOptions {
  // DSP_CTRL2
  int notch_filter = 0;
  int lpf_order = 0;
  int lpf_fc = 4;

  // OUT_CTRL1
  int mosi_latch_en = 0;
  int cmd_latch_en = 0;
  int tempc_format = 0;
  int data_format = 0;

  // SEL_FSR
  int sel_fsr = 0;
};

// Contains struct of raw sensor data
struct UnscaledData {
  int32_t gyro_x = 0;
  int32_t gyro_y = 0;
  int32_t gyro_z = 0;
};

// Contains struct of scaled sensor data
struct ScaledData {
  float gyro_x = 0.0;
  float gyro_y = 0.0;
  float gyro_z = 0.0;
};

//------------------------
// XV7211_SPI class
//------------------------

class XV7211_SPI : public SPI_EPSON_COM {
 public:
  XV7211_SPI(SPIClass& spiPort, uint32_t spiClkRate, int8_t ncs,
             uint8_t slave_id, Stream& consolePort);

  // Stores unscaled sensor data
  struct UnscaledData unscaled;

  // Stores scaled sensor data
  struct ScaledData scaled;

  float getGyroSf(void) { return _gyro_sf; };
  float getTempSf(void) { return _tempc_sf; };
  void configPrint(void);
  void headerPrint(uint8_t slave_id = 0);
  void scaledDataPrint(uint32_t sampleIndex);
  void unscaledDataPrint(uint32_t sampleIndex);
  boolean globalAngularRateRead(boolean verbose = false);
  float getNormalAngularRate(boolean verbose = false);
  float getTemperature(boolean verbose = false);

  void initOptions(const struct InitOptions&, boolean verbose = false);
  void registerDump(void);
  void sleepIn(boolean verbose = false);
  void sleepOut(boolean verbose = false);
  void softReset(boolean verbose = false);
  void triggerDataLatch(void);
  void commandDataLatch(boolean verbose = false);

 private:
  boolean _globalAngularRateRead(uint8_t* readBuf, const size_t length,
                                 boolean verbose = false);
  boolean _toUnscaled(struct UnscaledData& unscaledField,
                      const uint8_t* readBuf, const size_t length);
  void _toScaled(struct ScaledData& scaledField,
                 const struct UnscaledData& unscaledField);

  // byte buffer to store sensor read
  uint8_t _read_burst[XV7211_NS::GLOBAL_24B_LEN] = {};

  // length of byte_buffer in bytes
  uint8_t _read_burst_byte_len = 0;

  // Stores settings after initOptions()
  InitOptions _options;

  // Stores gyro scale factor after initOptions()
  float _gyro_sf = 0;

  // Stores temperature scale factor after initOptions()
  float _tempc_sf = 0;

  // Serial console
  uint8_t _slave_id = 0x01;

  // Serial console
  Stream& _consolePort;
};
