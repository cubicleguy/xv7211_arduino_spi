/**************************************************************************/
/*!
    @file     spi_com.h

    Header file for Epson SPI class

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

#include <Arduino.h>
#include <SPI.h>

//------------------------
// SPI_EPSON_COM driver class
//------------------------
class SPI_EPSON_COM {
 public:
  SPI_EPSON_COM(SPIClass& spiPort, uint32_t spiClkRate, int8_t ncs,
                uint8_t slave_id, Stream& consolePort);

  boolean begin(void);
  uint8_t SPItransfer(uint8_t x);
  void regWrite8(uint8_t addr, uint8_t value, boolean verbose = false);
  void regWrite8(uint8_t slave_id, uint8_t addr, uint8_t value,
                 boolean verbose = false);
  uint8_t regRead8(uint8_t addr, boolean verbose = false);
  uint8_t regRead8(uint8_t slave_id, uint8_t addr, boolean verbose = false);
  int16_t regRead16(uint8_t addr, boolean verbose = false);
  int16_t regRead16(uint8_t slave_id, uint8_t addr, boolean verbose = false);
  int32_t regRead24(uint8_t addr, boolean verbose = false);
  int32_t regRead24(uint8_t slave_id, uint8_t addr, boolean verbose = false);
  boolean regReadNByte(uint8_t* byteBuffer, uint8_t addr, uint8_t byteLen,
                       boolean verbose = false);

 private:
  SPIClass& _spiPort;
  uint32_t _spiClkRate = 5000000;
  int8_t _ncs = -1;
  uint8_t _slave_id = 0x01;
  Stream& _consolePort;
  boolean _initialised = false;
};
