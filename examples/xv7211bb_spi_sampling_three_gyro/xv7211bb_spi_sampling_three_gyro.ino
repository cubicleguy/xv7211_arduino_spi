/***********************************************************************
  xv7211bb_spi_sampling_three_gyro.ino
  This Arduino sketch example performs sensor initialization for
  multi-slave three XV7211BB with F-code H J L and performs
  global angular read commands to read all three Epson XV7211 gyros
  and outputs to the serial console.

  For detailed information on the Epson XV7011, refer to the datasheet at
  ----> https://www.epsondevice.com/crystal/en/products/sensor/

  This test application assumes that the XV7211 SPI signals are connected
  to the Arduino (hardware SPI).
  The pins are mapped as shown below.

  Circuit Pinmapping:
  Arduino DUE    Teensy 3.6 Arduino Zero/Due         XV7211
  --------------------------------------------------------------
  SS:    pin 4   pin 4      pin 4                    pin 2
  MOSI:  SPI-4   pin 11     ICSP-4                   pin 1
  MISO:  SPI-1   pin 12     ICSP-1                   pin 9
  SCK:   SPI-3   pin 13     ICSP-3                   pin 10

CAUTION: The Epson device I/O interface is 1.65V to 3.6V CMOS.
         Be sure to NOT use only Arduino devices that are 5.0V I/O!

  NOTE: For Teensy 3.6, the SPI interface glitches during initialization.
        The errant glitch can cause SPI communication error during
        Arduino flash programming and reboot by "Program" pushbutton on
        Teensy 3.6.
        To recover, unplug and plug the USB cable btween PC and Teensy 3.6

BSD license, all text above must be included in any redistribution

************************************************************************

    @section LICENSE

    Software License Agreement (BSD License)

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

***********************************************************************/

#include <Arduino.h>
#include "xv7211_spi.h"

// SLAVE_ID is set according to the F-code of gyro sensor connected
// 0=Multi-slave three Gyro F-code H, J, L
// 1=Gyro F-code H
// 2=Gyro F-code J
// 3=Gyro F-code L
//
// NOTE: For multi-slave three gyros assumes:
// X axis = F-code H
// Y axis = F-code J
// Z axis = F-code L
#define SLAVE_ID 0
// Define SS pin on Arduino host
#define CS 4

// Create Epson gyro object with interface settings
XV7211_SPI su = XV7211_SPI(SPI, 5000000, CS, SLAVE_ID, Serial);

/*------------------------------------------------------------------------------
 * setup()
 *
 *------------------------------------------------------------------------------*/
void setup() {
  // Setup Serial Debug Console communications for 250000bps
  Serial.begin(250000);
  while (!Serial) {
    ;  // Wait for serial port to connect. Needed for native USB port only
  }

  if (!su.begin()) {
    Serial.println("Could not initialize");
  }

  // Software reset is recommneded
  Serial.println("**Reset**");
  su.softReset();

  // Create Epson init structure with settings
  InitOptions init_options;
  // Configure init_options
  // notch_filter 1=enable 0=disable
  init_options.notch_filter = 1;
  // lpf_order 0=2nd 1=3rd 2=4th
  init_options.lpf_order = 1;
  // lpf_fc 0=1 1=10 2=25 3=50 4=100 5=200 6=400 7=500 Hz
  init_options.lpf_fc = 1;
  // cmd_latch_en 0=cmd latch disabled 1=cmd latch enabled
  init_options.cmd_latch_en = 0;
  // tempc_format 0=7-bit resolution 1=8-bit resolution
  init_options.tempc_format = 1;
  // data_format 0=16-bit 1=24-bit
  init_options.data_format = 1;
  // sel_fsr 0=FS=1 2=FS=1/4
  init_options.sel_fsr = 0;

  // Pass settings to initialize
  // All three XV7211BB gyros will accept
  // the same initialization commands
  Serial.print("**Init Gyro**");
  su.initOptions(init_options);
}

/*------------------------------------------------------------------------------
 * loop()
 *
 *------------------------------------------------------------------------------*/
void loop() {
  // Display sensor settings
  su.configPrint();

  // Read global angular rate
  Serial.print("\nGlobalAngularRateRead\n=================");
  su.headerPrint();
  for (int i = 0; i < 1000; i++) {
    su.globalAngularRateRead();
    su.scaledDataPrint(i);
  }
  Serial.print("\nDone.");
  while (1) {
    // Wait forever
  }
}
