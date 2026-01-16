# Table of Contents

______________________________________________________________________

<!---toc start-->

- [Table of Contents](#table-of-contents)
- [Epson XV7211BB SPI Driver for Arduino](#epson-xv7211bb-spi-driver-for-arduino)
- [Hardware Considerations](#hardware-considerations)
- [Installation Instructions](#installation-instructions)
  - [1. Install Arduino IDE](#1-install-arduino-ide)
  - [2. If Applicable Install the Teensy 3.6 board](#2-if-applicable-install-the-teensy-36-board)
  - [3. If Applicable Install the Arduino SAM boards](#3-if-applicable-install-the-arduino-sam-boards)
  - [4. Install Epson XV7211_SPI Library with example sketches](#4-install-epson-xv7211_spi-library-with-example-sketches)
  - [5. Use the IDE to compile example sketches and upload to the board](#5-use-the-ide-to-compile-example-sketches-and-upload-to-the-board)
- [Example Serial Console Output](#example-serial-console-output)
  - [Single Gyro Example](#single-gyro-example)
  - [Three Gyro Example](#three-gyro-example)
- [Change Record:](#change-record)

<!---toc end-->

# Epson XV7211BB SPI Driver for Arduino

______________________________________________________________________

This is an example test library for the Epson XV7211BB
using the 4-wire SPI interface.
It is developed on the Arduino Zero or Teensy 3.6
development board (Teensyduino) and includes example applications that
can be used within the Arduino IDE.
The library requires one SPI port and one UART port on the Arduino (one UART for serial
console output, and one SPI for connection to Epson device).

For detailed information on the Epson Gyro Sensors, refer to the datasheet at
----> https://www.epsondevice.com/crystal/en/products/sensor/

For further information on the Arduino, refer to their website at
----> http://www.arduino.cc

This software is released under the BSD license (see license.txt).
All text must be included in any redistribution.

# Hardware Considerations

______________________________________________________________________

This library assumes that the user has the following:

- Epson XV7211BB gyro sensor device
- Arduino Teensy 3.6 or Arduino Zero (or compatible) development board
- Arduino IDE software v2.3.6 or greater
- This software package
- Micro USB cable to power/connect to development board and use Serial Monitor

The default configuration of the driver assumes that:

- Epson device is connected to `SPI`
- serial console output is connected to `Serial`

The following table shows the default pin mapping.

Circuit Pinmapping:

| Signal on Host | Teensy 3.6 | Arduino Zero/DUE | XV7211BB |
| -------------- | ---------- | ---------------- | -------- |
| SS             | pin 4      | pin 4            | pin 2    |
| MOSI           | pin 11     | SPI-4            | pin 1    |
| MISO           | pin 12     | SPI-1            | pin 9    |
| SCK            | pin 13     | SPI-3            | pin 10   |

**CAUTION**: The Epson device I/O interface is 1.65V to 3.6V CMOS.
Be sure to NOT use only Arduino devices that are 5.0V I/O!

# Installation Instructions

______________________________________________________________________

To use the Epson XV7211_SPI driver for Arduino and examples, the following steps are required.

1. Install Arduino IDE (if not already installed)
2. Install the Arduino board
3. Install Epson XV7211_SPI Library for Arduino and example sketches
4. Use the IDE to compile example sketches and upload to the Arduino board

## 1. Install Arduino IDE

______________________________________________________________________

The Epson XV7211BB library for Arduino is designed to work with the Arduino IDE.
The IDE requires a platform running Windows, Mac OS X, or Linux.
If you do not have the IDE installed on your development platform, please visit the Arduino
website and download the version of the IDE compatible with your operating system.
Once the IDE is installed on your development platform, proceed to the next step.
For specific requirements and installation instructions, refer to the Arduino website at www.arduino.cc.

## 2. If Applicable Install the Teensy 3.6 board

______________________________________________________________________

The default installation of Arduino IDE may not include support for the Teensy 3.6.
To confirm whether Teensy support is installed click on `Tools->Board->Boards Manager...` on the IDE menu.

Search for "Teensy" and confirm whether the `Teensy (for Arduino IDE 2.0.4 or later)` board package is installed. If the
package is not installed, install it the latest version. Once the Teensy package is installed,
proceed to the next step.

## 3. If Applicable Install the Arduino SAM boards

______________________________________________________________________

The default installation of Arduino IDE may not include support for the Arduino Zero.
To confirm whether Zero support is installed click on `Tools->Board->Boards Manager...` on the IDE menu.

Confirm whether the `Arduino SAM Boards (32-bits ARM Cortex-M0+)` board package is installed. If the
package is not installed, install the latest version that matches your version of the IDE.
Once the SAM Boards package is installed, proceed to the next step

## 4. Install Epson XV7211_SPI Library with example sketches

______________________________________________________________________

The Epson XV7211BB library for Arduino is available for install within the Arduino IDE if connected to the internet.
In the Arduino IDE click on `Tools->Manage Libraries->Library Manager search for Epson_SU_SPI...`.
Then select the Epson XV7211BB library for Arduino package to install the driver and examples.

The Epson XV7211BB library for Arduino is available as a .zip archive from https://github.com/cubicleguy/xv7211_spi/releases.
The IDE can directly import the driver from a .zip file, so on the IDE menu click on `Sketch->Include Library->Add .ZIP Library...`.
Then select the Epson XV7211BB library for Arduino ZIP package. This will install the driver and examples.

## 5. Use the IDE to compile example sketches and upload to the board

______________________________________________________________________

Before compiling the example sketches, set the Board and Port settings in the IDE.
The Board and Port settings tell the IDE which Arduino product is being used and how to communicate with it.
To set the Board, click `Tools->Board->select the Teensy 3.6, Arduino Zero, or other compatible board`.
To set the Port, click `Tools->Port->select the proper serial port`.

The port that your Arduino board is located on may differ according to the operating system on the development system.
For issues regarding USB port connections, please refer to the Arduino website at http://www.arduino.cc.

The following are examples sketches included in the library:

- xv7211bb_spi_sampling_single_gyro.ino is designed to demonstrate initializing and reading a single XV7211BB gyro.
- xv7211bb_spi_sampling_three_gyro.ino is similar but for multi-slave 3-gyro configuration of XV7211BB gyros F-code H, J, L.

To open the example sketches click on `File->Examples`, find the `xv7211_spi...`, and then select one of the example sketches.
Once the example sketch is loaded, it can be compiled and uploaded to the Arduino.
**NOTE:** The Upload stage will fail if the IDE `Board` and `Port` settings are not configured correctly.

If the upload to the Arduino completes successfully, the output from the example sketch can be viewed using
the Serial Monitor available in `Tools->Serial Monitor`.
**NOTE:** You may have to set the serial baudrate on the Serial Monitor to match the baudrate setting in the sketch.

# Example Serial Console Output

______________________________________________________________________

## Single Gyro Example

```
Platform: Normal Arduino
Open SPI Port for Epson device: nCS on 4 @ 5000000 Hz
**Reset**
**Init Gyro**
----------------------
XV7211 Slave ID: 1
Notch Filter: On
LPF Order: 3rd
LPF Fc: 50 Hz
EnbImuLatch: Disable
EnbCmdTrg: Disable
TFormat: 256 LSB/degC
DataFormat: 24-bit
FS=1
Gyro SF: 0.00001480
TempC SF: 0.00390625
----------------------

Normal Angular Rate Read & Temperature
=================
Sample#	Gyro	TempC
0	-0.02449	24.176
1	-0.03387	24.176
2	-0.04143	24.176
3	-0.04724	24.176
4	-0.05313	24.176
5	-0.05663	24.176
6	-0.05811	24.176
7	-0.05837	24.176
8	-0.05917	24.176
9	-0.06374	24.176
10	-0.07138	24.176
...
991	-0.07739	24.199
992	-0.07338	24.195
993	-0.07182	24.195
994	-0.07160	24.195
995	-0.07262	24.195
996	-0.07545	24.195
997	-0.07987	24.195
998	-0.08459	24.195
999	-0.08835	24.195

Done.
```

## Three Gyro Example

```
Platform: Normal Arduino
Open SPI Port for Epson device: nCS on 4 @ 5000000 Hz
**Reset**
**Init Gyro**
----------------------
XV7211 Slave ID: 0
Notch Filter: On
LPF Order: 3rd
LPF Fc: 10 Hz
EnbImuLatch: Disable
EnbCmdTrg: Disable
TFormat: 256 LSB/degC
DataFormat: 24-bit
FS=1
Gyro SF: 0.00001480
TempC SF: 0.00390625
----------------------

GlobalAngularRateRead
=================
Sample#	Gx	Gy	Gz
0, 0.05887, 0.15092, 0.01987
1, 0.05811, 0.15125, 0.02039
2, 0.05669, 0.15189, 0.02098
3, 0.05479, 0.15291, 0.02181
4, 0.05207, 0.15431, 0.02279
5, 0.04864, 0.15624, 0.02398
6, 0.04457, 0.15845, 0.02548
7, 0.03983, 0.16100, 0.02703
8, 0.03456, 0.16399, 0.02868
9, 0.02896, 0.16687, 0.03047
10, 0.02363, 0.16973, 0.03211
...
991, -0.07450, 0.20332, 0.06839
992, -0.07493, 0.20372, 0.06861
993, -0.07546, 0.20413, 0.06882
994, -0.07604, 0.20459, 0.06905
995, -0.07654, 0.20502, 0.06934
996, -0.07684, 0.20534, 0.06969
997, -0.07702, 0.20551, 0.07008
998, -0.07719, 0.20549, 0.07045
999, -0.07743, 0.20526, 0.07073

Done.
```

# Change Record:

| Date       | Ver  | Comment           |
| ---------- | ---- | ----------------- |
| 2025-12-09 | v1.0 | - Initial release |
