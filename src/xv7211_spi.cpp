/**************************************************************************/
/*!
    @file     xv7211_spi.cpp

    Epson XV7211 Class

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

#include "xv7211_spi.h"

using namespace XV7211_NS;

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
// For three slave connection, set slave_id = 0
//   slave_id 1=XV7211 Frequency Code=H Custom Code=C
//   slave_id 2=XV7211 Frequency Code J Custom Code=D
//   slave_id 3=XV7211 Frequency Code L Custom Code=E
// Register writes apply the same write data to all three slaves
// Register reads are not supported
// Global Angular rate reads retriever data from all three slaves in sequence
// Normal Angular rate reads are not supported
//
// For single slave connection, set the slave_id according to the Frequency
// Code/ Custom Code
//   slave_id 1=XV7211 Frequency Code=H Custom Code=C
//   slave_id 2=XV7211 Frequency Code J Custom Code=D
//   slave_id 3=XV7211 Frequency Code L Custom Code=E
// Register writes go to the single slaves
// Register reads are from the single slave
// Global Angular rate reads are not supported
// Normal Angular rate reads sensor data from the single slave

XV7211_SPI::XV7211_SPI(SPIClass& spiPort, uint32_t spiClkRate, int8_t ncs,
                       uint8_t slave_id, Stream& consolePort)
    :  // initializer list
      SPI_EPSON_COM(spiPort, spiClkRate, ncs, slave_id, consolePort),
      _slave_id(slave_id),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief      Sends device info to console
*/
/**************************************************************************/
void XV7211_SPI::configPrint(void) {
  _consolePort.print("\n----------------------");
  _consolePort.print("\nXV7211 Slave ID: ");
  _consolePort.println(_slave_id, HEX);

  _consolePort.print("Notch Filter: ");
  if (_options.notch_filter) {
    _consolePort.println("On");
  } else {
    _consolePort.println("Off");
  }

  _consolePort.print("LPF Order: ");
  switch (_options.lpf_order) {
    case 0:
      _consolePort.println("2nd");
      break;
    case 1:
      _consolePort.println("3rd");
      break;
    case 2:
      _consolePort.println("4th");
      break;
    default:
      _consolePort.println("Invalid");
      break;
  }

  _consolePort.print("LPF Fc: ");
  switch (_options.lpf_fc) {
    case 0:
      _consolePort.println("1 Hz");
      break;
    case 1:
      _consolePort.println("10 Hz");
      break;
    case 2:
      _consolePort.println("25 Hz");
      break;
    case 3:
      _consolePort.println("50 Hz");
      break;
    case 4:
      _consolePort.println("100 Hz");
      break;
    case 5:
      _consolePort.println("200 Hz");
      break;
    case 6:
      _consolePort.println("400 Hz");
      break;
    case 7:
      _consolePort.println("500 Hz");
      break;
    default:
      _consolePort.println("Invalid");
      break;
  }

  _consolePort.print("EnbImuLatch: ");
  if (_options.mosi_latch_en) {
    _consolePort.println("Enable");
  } else {
    _consolePort.println("Disable");
  }

  _consolePort.print("EnbCmdTrg: ");
  if (_options.cmd_latch_en) {
    _consolePort.println("Enable");
  } else {
    _consolePort.println("Disable");
  }

  _consolePort.print("TFormat: ");
  if (_options.tempc_format) {
    _consolePort.println("256 LSB/degC");
  } else {
    _consolePort.println("128 LSB/degC");
  }

  _consolePort.print("DataFormat: ");
  if (_options.data_format) {
    _consolePort.println("24-bit");
  } else {
    _consolePort.println("16-bit");
  }

  switch (_options.sel_fsr) {
    case 0:
      _consolePort.println("FS=1");
      break;
    case 2:
      _consolePort.println("FS=1/4");
      break;
    default:
      _consolePort.println("Invalid");
      break;
  }
  _consolePort.print("Gyro SF: ");
  _consolePort.println(_gyro_sf, 8);
  _consolePort.print("TempC SF: ");
  _consolePort.println(_tempc_sf, 8);
  _consolePort.println("----------------------");
}

/**************************************************************************/
/*!
    @brief      Sends column headers to console

    @param [in]  slave_id
                 0=Gx Gy Gz 1=Gx 2=Gy 3=Gz
*/
/**************************************************************************/
void XV7211_SPI::headerPrint(uint8_t slave_id) {
  _consolePort.print("\nSample#");
  switch (slave_id) {
    case 0:
      _consolePort.println("\tGx\tGy\tGz");
      break;
    case 1:
      _consolePort.println("\tGx");
      break;
    case 2:
      _consolePort.println("\tGy");
      break;
    case 3:
      _consolePort.println("\tGz");
      break;
    default:
      _consolePort.println("Invalid");
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Sends a row of scale sample data to console

    @param [in]  sampleIndex
                 Current count of row
*/
/**************************************************************************/
void XV7211_SPI::scaledDataPrint(uint32_t sampleIndex) {
  _consolePort.print(sampleIndex);
  _consolePort.print(", ");
  _consolePort.print(scaled.gyro_x, 5);
  _consolePort.print(", ");
  _consolePort.print(scaled.gyro_y, 5);
  _consolePort.print(", ");
  _consolePort.println(scaled.gyro_z, 5);
}

/**************************************************************************/
/*!
    @brief  Sends a row of unscaled sample data to console

    @param [in]  sampleIndex
                 Current count of row
*/
/**************************************************************************/
void XV7211_SPI::unscaledDataPrint(uint32_t sampleIndex) {
  _consolePort.print(sampleIndex);
  _consolePort.print(", ");
  _consolePort.print(unscaled.gyro_x, DEC);
  _consolePort.print(", ");
  _consolePort.print(unscaled.gyro_y, DEC);
  _consolePort.print(", ");
  _consolePort.println(unscaled.gyro_z, DEC);
}

/**************************************************************************/
/*!
    @brief  Perform global angular rate read (three gyros)
            NOTE: Only supported when three different devices
                  (with different frequency codes) are connected in
                  multi-slave configuration
            Scaled gyro sensor data in deg/sec stored in struct scaled
            Unscaled gyro sensor data as integer stored in struct unscaled

    @param [in] boolean verbose
                Send register accesses to console for debugging

    @returns  true if successful, false if burst read contained errors
*/
/**************************************************************************/
boolean XV7211_SPI::globalAngularRateRead(boolean verbose) {
  if (!_globalAngularRateRead(_read_burst, _read_burst_byte_len, verbose)) {
    return false;
  }
  if (!_toUnscaled(unscaled, _read_burst, _read_burst_byte_len)) {
    return false;
  }
  _toScaled(scaled, unscaled);
  return true;
}

/**************************************************************************/
/*!
    @brief  Perform normal angular rate read (single gyro)

    @param [in] boolean verbose
                Send register accesses to console for debugging

    @returns  float angular rate data in deg/sec
*/
/**************************************************************************/
float XV7211_SPI::getNormalAngularRate(boolean verbose) {
  int32_t unscaled_gyro;
  if (_options.data_format) {
    unscaled_gyro = regRead24(ADDR_DAT_ACC_ON, verbose);
  } else {
    unscaled_gyro = regRead16(ADDR_DAT_ACC_ON, verbose);
  }
  return (float)unscaled_gyro * _gyro_sf;
}

/**************************************************************************/
/*!
    @brief  Read temperature sensor

    @param [in] boolean verbose
                Send register accesses to console for debugging

    @returns  float temperature in degrees C
*/
/**************************************************************************/
float XV7211_SPI::getTemperature(boolean verbose) {
  int16_t unscaled_tempc = regRead16(ADDR_TEMP_RD, verbose);
  float scaled_tempc = (float)unscaled_tempc * _tempc_sf;
  // For TFormat=1 need to add 25C
  if (_options.tempc_format) {
    scaled_tempc += 25.0;
  }
  return scaled_tempc;
}

/**************************************************************************/
/*!
    @brief  Programs the device registers from provided init_options struct

    @param [in] struct InitOptions
                Initialization settings

    @param [in] boolean verbose
                Send register accesses to console for debugging
*/
/**************************************************************************/
void XV7211_SPI::initOptions(const struct InitOptions& options,
                             boolean verbose) {
  uint8_t dsp_ctrl2 = (options.notch_filter & 0x01) << 6 |
                      (options.lpf_order & 0x03) << 4 | (options.lpf_fc & 0x0F);

  uint8_t out_ctrl1 = (options.mosi_latch_en & 0x01) << 7 |
                      (options.cmd_latch_en & 0x01) << 5 |
                      (options.tempc_format & 0x01) << 3 |
                      (options.data_format & 0x01) << 2 | 0x01;

  uint8_t sel_fsr = (options.sel_fsr & 0x03);

  regWrite8(ADDR_DSP_CTRL2, dsp_ctrl2, verbose);
  regWrite8(ADDR_OUT_CTRL1, out_ctrl1, verbose);
  regWrite8(ADDR_SEL_FSR, sel_fsr, verbose);

  _options.notch_filter = options.notch_filter & 0x01;
  _options.lpf_order = options.lpf_order & 0x03;
  _options.lpf_fc = options.lpf_fc & 0x0F;
  _options.mosi_latch_en = options.mosi_latch_en & 0x01;
  _options.cmd_latch_en = options.cmd_latch_en & 0x01;
  _options.tempc_format = options.tempc_format & 0x01;
  _options.data_format = options.data_format & 0x01;
  _options.sel_fsr = options.sel_fsr & 0x03;

  if (_options.data_format == 0 && _options.sel_fsr == 0) {
    _gyro_sf = GYRO_SF16;
  } else if (_options.data_format == 0 && _options.sel_fsr == 2) {
    _gyro_sf = GYRO_SF16DIV2;
  } else if (_options.data_format == 1 && _options.sel_fsr == 0) {
    _gyro_sf = GYRO_SF24;
  } else {
    _gyro_sf = GYRO_SF24DIV2;
  }

  _tempc_sf = _options.tempc_format ? TEMPC_SF2 : TEMPC_SF1;

  _read_burst_byte_len = options.data_format ? GLOBAL_24B_LEN : GLOBAL_16B_LEN;
}

/**************************************************************************/
/*!
    @brief      Read back all register values from the sensor and
                send to console
*/
/**************************************************************************/
void XV7211_SPI::registerDump(void) {
  if (_slave_id == 0) {
    _consolePort.println("Slave ID is 0x00, register reading is not supported");
    return;
  }

  _consolePort.println("\r\nRegister Dump:");
  regRead8(ADDR_DSP_CTRL2, true);
  regRead8(ADDR_STS_RD, true);
  regRead8(ADDR_STATUS, true);
  regRead16(ADDR_TEMP_RD, true);
  if (_options.data_format) {
    regRead24(ADDR_DAT_ACC_ON, true);
  } else {
    regRead16(ADDR_DAT_ACC_ON, true);
  }
  regRead8(ADDR_OUT_CTRL1, true);
  regRead8(ADDR_SEL_FSR, true);
  regRead8(ADDR_DAT_LATCH_COM, true);
}

/**************************************************************************/
/*!
    @brief  Enters Sleep mode

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

*/
/**************************************************************************/
void XV7211_SPI::sleepIn(boolean verbose) {
  regWrite8(ADDR_PROT_STATE, CMD_SLP_CTRL_ENB, verbose);
  regWrite8(ADDR_STATUS, CMD_SLP_IN, verbose);
  delayMicroseconds(TSLPIN_DELAY_US);
}

/**************************************************************************/
/*!
    @brief  Exits Sleep mode

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

*/
/**************************************************************************/
void XV7211_SPI::sleepOut(boolean verbose) {
  regWrite8(ADDR_PROT_STATE, CMD_SLP_CTRL_ENB, verbose);
  regWrite8(ADDR_STATUS, CMD_SLP_OUT, verbose);
  delayMicroseconds(TSLPOUT_DELAY_US);
}

/**************************************************************************/
/*!
    @brief  Software reset

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

*/
/**************************************************************************/
void XV7211_SPI::softReset(boolean verbose) {
  regWrite8(ADDR_PROT_SOFT_R, CMD_SOFT_RESET_ENB, verbose);
  regWrite8(ADDR_SOFT_RESET, CMD_SOFT_RESET, verbose);
  delayMicroseconds(TSWRST_DELAY_US);
  // Wait for startup time
  delay(TSTA_DELAY_MS);
}

/**************************************************************************/
/*!
    @brief  Trigger data latch using high pulse on MOSI (when nCS is high)
            NOTE: This is not implemented

*/
/**************************************************************************/
void XV7211_SPI::triggerDataLatch(void) {
  _consolePort.println("Warning: Not implemented.");
}

/**************************************************************************/
/*!
    @brief  Send latch command LOW -> HIGH to latch sensor data

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

*/
/**************************************************************************/
void XV7211_SPI::commandDataLatch(boolean verbose) {
  regWrite8(ADDR_DAT_LATCH_COM, CMD_DIS_CMD_TRG, verbose);
  regWrite8(ADDR_DAT_LATCH_COM, CMD_ENB_CMD_TRG, verbose);
}

/**************************************************************************/
/*!
    @brief  Performs a global angular rate read

    @param [out] pointer to 8-bit array (stores global read values)
    @param [int] length in bytes for 8-bit array
    @param [in]  boolean verbose
                 Send register accesses to console for debugging
    @returns    true if successful or false if errors detected

*/
/**************************************************************************/
boolean XV7211_SPI::_globalAngularRateRead(uint8_t* readBuf,
                                           const size_t length,
                                           boolean verbose) {
  if (_read_burst_byte_len == 0) {
    _consolePort.println("Error: Bypassing because burst length is invalid");
    _consolePort.println("Has sensor been initialized with initOptions()?");
    return false;
  }

  if ((uint8_t)length < _read_burst_byte_len) {
    _consolePort.print("Error: Bypassing because the array is smaller");
    _consolePort.print(" than the read burst length of: ");
    _consolePort.println(_read_burst_byte_len, DEC);
    return false;
  }

  // Read burst sensor data, slave_id bits[6:5] is 0b00
  regReadNByte(readBuf, ADDR_DAT_ACC_ON, _read_burst_byte_len, verbose);
  return true;
}

/**************************************************************************/
/*!
    @brief  Convert burst read sensor buffer data to unscaled integer fields

    @param [out] unscaledField
                 Reference to struct UnscaledData
    @param [in]  readBuf
                 Pointer to 8-bit Array
    @param [in]  length
                 Size of 8-bit Array in bytes
*/
/**************************************************************************/
boolean XV7211_SPI::_toUnscaled(struct UnscaledData& unscaledField,
                                const uint8_t* readBuf, const size_t length) {
  if (_read_burst_byte_len == 0) {
    _consolePort.println("Error: Bypassing because burst length is invalid");
    _consolePort.println("Has sensor been initialized with initOptions()?");
    return false;
  }
  if ((uint8_t)length < _read_burst_byte_len) {
    _consolePort.print("Error: Bypassing because the array is smaller");
    _consolePort.print(" than the read burst length of: ");
    _consolePort.println(_read_burst_byte_len, DEC);
    return false;
  }
  int32_t tmp_val;
  int i = 0;
  // If 24 bit
  if (_options.data_format) {
    // 24-bit converted to int32_t
    tmp_val = (int32_t)(readBuf[i] << 24) | (int32_t)(readBuf[i + 1] << 16) |
              (int32_t)(readBuf[i + 2] << 8);
    unscaledField.gyro_x = tmp_val >> 8;
    tmp_val = (int32_t)(readBuf[i + 3] << 24) |
              (int32_t)(readBuf[i + 4] << 16) | (int32_t)(readBuf[i + 5] << 8);
    unscaledField.gyro_y = tmp_val >> 8;
    tmp_val = (int32_t)(readBuf[i + 6] << 24) |
              (int32_t)(readBuf[i + 7] << 16) | (int32_t)(readBuf[i + 8] << 8);
    unscaledField.gyro_z = tmp_val >> 8;
  } else {
    // 16 bit converted to int32_t
    tmp_val = (int32_t)(readBuf[i] << 24) | (int32_t)(readBuf[i + 1] << 16);
    unscaledField.gyro_x = tmp_val >> 16;

    tmp_val = (int32_t)(readBuf[i + 2] << 24) | (int32_t)(readBuf[i + 3] << 16);
    unscaledField.gyro_y = tmp_val >> 16;

    tmp_val = (int32_t)(readBuf[i + 4] << 24) | (int32_t)(readBuf[i + 5] << 16);
    unscaledField.gyro_z = tmp_val >> 16;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Convert unscaled sensor data to scaled sensor data

    @param [out] scaledField
                 Reference to struct ScaledData
    @param [in]  unscaledField
                 Reference to struct UnscaledData
*/
/**************************************************************************/
void XV7211_SPI::_toScaled(struct ScaledData& scaledField,
                           const struct UnscaledData& unscaledField) {
  scaledField.gyro_x = (float)unscaledField.gyro_x * _gyro_sf;
  scaledField.gyro_y = (float)unscaledField.gyro_y * _gyro_sf;
  scaledField.gyro_z = (float)unscaledField.gyro_z * _gyro_sf;
}
