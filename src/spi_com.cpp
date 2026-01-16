/**************************************************************************/
/*!
    @file     spi_com.cpp

    Epson SPI class

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

#include "spi_com.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
SPI_EPSON_COM::SPI_EPSON_COM(SPIClass& spiPort, uint32_t spiClkRate, int8_t ncs,
                             uint8_t slave_id, Stream& consolePort)
    :  // initializer list
      _spiPort(spiPort),
      _spiClkRate(spiClkRate),
      _ncs(ncs),
      _slave_id(slave_id),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Initializes SPI

*/
/**************************************************************************/
boolean SPI_EPSON_COM::begin(void) {
  if (_ncs == -1) {
    _consolePort.println("Error: nCS must be specified.");
    return false;
  }

  _consolePort.print("\nPlatform: ");
#if defined(__SAM3X8E__)
  _consolePort.println("Arduino DUE");
#elif defined(__MK66FX1M0__)
  _consolePort.println("Teensy 3.6");
#else
  _consolePort.println("Normal Arduino");
#endif
  pinMode(_ncs, OUTPUT);
  digitalWrite(_ncs, HIGH);
  _spiPort.begin();
  _spiPort.beginTransaction(SPISettings(_spiClkRate, MSBFIRST, SPI_MODE3));
  _consolePort.print("Open SPI Port for Epson device: ");
  _consolePort.print("nCS on ");
  _consolePort.print(_ncs, DEC);
  _consolePort.print(" @ ");
  _consolePort.print(_spiClkRate, DEC);
  _consolePort.println(" Hz");

  _initialised = true;
  return true;
}

/**************************************************************************/
/*!
    @brief  Wrapper for SPI transfers
    @param [in]  x (byte)
                 byte shifted out on MOSI
    @return      byte shifted in on MISO
*/
/**************************************************************************/
uint8_t SPI_EPSON_COM::SPItransfer(uint8_t x) { return _spiPort.transfer(x); }

/**************************************************************************/
/*!
    @brief  Writes an 8-bit value at the specific register address

    @param [in] addr
                The 8-bit register address
    @param [in] value
                The 8-bit value to write at address
    @param [in] verbose
                boolean to enabled debug output of register access
*/
/**************************************************************************/
void SPI_EPSON_COM::regWrite8(uint8_t addr, uint8_t value, boolean verbose) {
  // Set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address to be written
  // msb is set 0b for register write
  SPItransfer((addr | _slave_id << 5) & 0x7F);
  // Write data value
  SPItransfer(value);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);
  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print(((addr | _slave_id << 5) & 0x7F), HEX);
    _consolePort.print("] < 0x");
    _consolePort.println(value, HEX);
  }
}

/**************************************************************************/
/*!
    @brief  Writes an 8-bit value at the specific register address
            and slave_id

    @param [in] slave_id
                The 2-bit slave address applied to register address[6:5]
    @param [in] addr
                The 8-bit register address
    @param [in] value
                The 8-bit value to write at address
    @param [in] verbose
                boolean to enabled debug output of register access
*/
/**************************************************************************/
void SPI_EPSON_COM::regWrite8(uint8_t slave_id, uint8_t addr, uint8_t value,
                              boolean verbose) {
  // Set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address to be written
  // msb is set 0b for register write
  SPItransfer((addr | slave_id << 5) & 0x7F);
  // Write data value
  SPItransfer(value);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);
  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print(((addr | slave_id << 5) & 0x7F), HEX);
    _consolePort.print("] < 0x");
    _consolePort.println(value, HEX);
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 8bit value from the specified register address

    @param [in] addr
                The 8-bit register address
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 8-bit value retrieved from register
*/
/**************************************************************************/
uint8_t SPI_EPSON_COM::regRead8(uint8_t addr, boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address to be written
  // msb is set 1b for register read
  SPItransfer(addr | _slave_id << 5 | 0x80);
  // Initiate 16-bit dummy cycle to return data
  uint8_t readData = SPItransfer(0x80);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr | _slave_id << 5 | 0x80), HEX);
    _consolePort.print("] > 0x");
    _consolePort.println(readData, HEX);
  }
  // Return the data
  return readData;
}

/**************************************************************************/
/*!
    @brief  Reads an 8bit value from the specified register address and
            slave id

    @param [in] slave_id
                The 2-bit slave address applied to register address[6:5]
    @param [in] addr
                The 8-bit register address
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 8-bit value retrieved from register
*/
/**************************************************************************/
uint8_t SPI_EPSON_COM::regRead8(uint8_t slave_id, uint8_t addr,
                                boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address to be written
  // msb is set 1b for register read
  SPItransfer(addr | slave_id << 5 | 0x80);
  // Initiate 16-bit dummy cycle to return data
  uint8_t readData = SPItransfer(0x80);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr | slave_id << 5 | 0x80), HEX);
    _consolePort.print("] > 0x");
    _consolePort.println(readData, HEX);
  }
  // Return the data
  return readData;
}

/**************************************************************************/
/*!
    @brief  Reads an 16 bit value from the specified register address

    @param [in] winid
                The 8-bit window ID
    @param [in] addr
                The 8-bit register address
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 16-bit value retrieved from register
*/
/**************************************************************************/
int16_t SPI_EPSON_COM::regRead16(uint8_t addr, boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address
  // msb is set 1b for register read
  SPItransfer(addr | _slave_id << 5 | 0x80);
  // Initiate 16-bit dummy cycle to return data
  uint16_t readData = SPItransfer(0x00) << 8 | SPItransfer(0x00);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr | _slave_id << 5 | 0x80), HEX);
    _consolePort.print("] > 0x");
    _consolePort.println(readData, HEX);
  }
  // Return the data
  return (int16_t)readData;
}

/**************************************************************************/
/*!
    @brief  Reads an 16 bit value from the specified register address and
            slave id

    @param [in] slave_id
                The 2-bit slave address applied to register address[6:5]
    @param [in] winid
                The 8-bit window ID
    @param [in] addr
                The 8-bit register address
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 16-bit value retrieved from register
*/
/**************************************************************************/
int16_t SPI_EPSON_COM::regRead16(uint8_t slave_id, uint8_t addr,
                                 boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address
  // msb is set 1b for register read
  SPItransfer(addr | slave_id << 5 | 0x80);
  // Initiate 16-bit dummy cycle to return data
  uint16_t readData = SPItransfer(0x00) << 8 | SPItransfer(0x00);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr | slave_id << 5 | 0x80), HEX);
    _consolePort.print("] > 0x");
    _consolePort.println(readData, HEX);
  }
  // Return the data
  return (int16_t)readData;
}

/**************************************************************************/
/*!
    @brief  Reads an 24 bit value from the specified register address
                converted to 32-bit

    @param [in] addr
                The 8-bit register address
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 32-bit value converted from 24-bit retrieved from register
*/
/**************************************************************************/
int32_t SPI_EPSON_COM::regRead24(uint8_t addr, boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address
  // msb is set 1b for register read
  SPItransfer(addr | _slave_id << 5 | 0x80);
  // Initiate 24-bit dummy cycle to return data
  uint32_t readData =
    SPItransfer(0x00) << 24 | SPItransfer(0x00) << 16 | SPItransfer(0x00) << 8;
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr | _slave_id << 5 | 0x80), HEX);
    _consolePort.print("] > 0x");
    _consolePort.println(readData >> 8, HEX);
  }
  // Return the data
  return (int32_t)readData >> 8;
}

/**************************************************************************/
/*!
    @brief  Reads an 24 bit value from the specified register address and
            slave id converted to 32-bit

    @param [in] slave_id
                The 2-bit slave address applied to register address[6:5]
    @param [in] addr
                The 8-bit register address
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 32-bit value converted from 24-bit retrieved from register
*/
/**************************************************************************/
int32_t SPI_EPSON_COM::regRead24(uint8_t slave_id, uint8_t addr,
                                 boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address
  // msb is set 1b for register read
  SPItransfer(addr | slave_id << 5 | 0x80);
  // Initiate 24-bit dummy cycle to return data
  uint32_t readData =
    SPItransfer(0x00) << 24 | SPItransfer(0x00) << 16 | SPItransfer(0x00) << 8;
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr | slave_id << 5 | 0x80), HEX);
    _consolePort.print("] > 0x");
    _consolePort.println(readData >> 8, HEX);
  }
  // Return the data
  return (int32_t)readData >> 8;
}

/**************************************************************************/
/*!
    @brief  Reads a specified number of sequential 8-bit SPI cycles
            starting at the specified address.

    @param [out]  byteArrayOut (max 64 elements)
                  Array of return 8-bit values

    @param [in]  addr
                 First cycle SPI address
                 the transfer
    @param [in]  byteLen (in bytes, must be less than 128)
                 Specify the length of the burst read transfer in bytes

    @returns  true if successful, false if errors are detected
*/
/**************************************************************************/
boolean SPI_EPSON_COM::regReadNByte(uint8_t* byteBuffer, uint8_t addr,
                                    uint8_t byteLen, boolean verbose) {
  if (byteLen == 0) {
    _consolePort.println("byteLen must be greater than 0");
    return false;
  }
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the address, slave id = bit[6:5
  // msb is set 1b for register read
  SPItransfer(addr | 0x80);
  // Read SPI and store into byte array
  for (uint8_t i = 0; i < byteLen; i++) {
    byteBuffer[i] = SPItransfer(0x00);
  }
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("Bytes: ");
    for (uint8_t i = 0; i < byteLen; i++) {
      _consolePort.print(byteBuffer[i], HEX);
      _consolePort.print(", ");
    }
    _consolePort.println();
  }
  return true;
}
