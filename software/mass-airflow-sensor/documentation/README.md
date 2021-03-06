<p align="center">
  <img src="../../../images/logo.png">
</p>

# DIY-Beatmungsgerät Mass Air Flow Sensor

The mass air flow sensor has started as part of the "DIY-Beatmungsgerät" project, which itself developed itself from the German hackathon event [#WirvsVirusHackathon](https://wirvsvirushackathon.org/) in March 2020.

The aim of this repository to provide data to use ready-made mass air flow sensors as well as build and verify DIY mass air flow sensors. The current DIY approach is based on analogue differential pressure sensors.

*The plans, documents and other materials ("Material") contained in this repository are simple prototypes. The Material is not intended to be used in or as a medical device. The Material has not been tested and has not been approved for use in humans or animals by any regulatory authority of any country.*

*By using the Material, you are agreeing to the following disclaimer.*

*The Material is offered as-is and as-available, and makes no representations or warranties of any kind whatever concerning the Material, whether express, implied, statutory, or other. This includes, without limitation, warranties of merchantability, fitness for a particular purpose, non-infringement, absence of latent or other defects, accuracy, or the presence or absence of errors, whether or not known or discoverable.*

*To the extent possible, in no event will the authors be liable to you on any legal theory (including, without limitation, negligence) or otherwise for any direct, special, indirect, incidental, consequential, punitive, exemplary, or other losses, costs, expenses, or damages arising out of the Material or use of the Material, even if the authors have been advised of the possibility of such losses, costs, expenses, or damages.*

*The disclaimer of warranties and limitation of liability provided above shall be interpreted in a manner that, to the extent possible, most closely approximates an absolute disclaimer and waiver of all liability.*

## Hardware setup

The hardware setup consists of replaceable components:

- The sensor controller (e.g. an Arduiono Leonardo) which reads flow values from one or several meters.
- The mass airflow meter, either being
  - a ready-made meter, such as an *Sensirion SFM3000 Low Pressure Drop Digital Flow Meter* or
  - a DIY meter, consisting of a differential pressure sensor + 3D printed tube + hardware to read values from the sensor (e.g. an Arduino Nano) and provide them over a digital interface.

<p align="center">
  <img src="./images/flow-meter-sketch_40p.png">
</p>


The digital interface between sensors/meters and sensor controller is a simple I2C interface, i.e. the signals SCL (serial clock), SDA (serial data) and a common GND is required. The two I2C lines should also have a pull-up resistor. In the setup shown above, the pull-up resistors are pulled up by the controller's VCC of 5 V.

The hardware setup is based on Arduino hardware, but in general other hardware with an I2C interface, a generic I2C library and a C++ compiler can theoretically be used, too.



## DIY mass air flow meter

Our prototype for a DIY mass air flow meter consists of:

1. **a 3D printed tube:**
   Currently two designs are available: the "Grid type" and the "Venturi type"); the model files are available within this repository. *Hint: Support for the "Grid type" is unlikely to be continued!* 

2. **an analog differential pressure sensor:**
   The current prototype uses a NXP MPXV5004DP (for 0 to 3.92 kPa). It's an 8-pin chip but uses only power supply (VCC + GND) and provides a measurement via its analog output V_OUT. The 3D models of the tube are designed to fit the sensor in middle of the tube.

3.  **a microcontroller board** used to sample the analog values and provide them via a compatible digital interface (I2C):
   The prototype is based on an Arduino Nano. We'v also used an Arduino Leonardo compatible board: the Beetle Board from DFRobot with an even more appropriate form factor (also ATmega32U4 with bootloader)!

<p align="center">
  <img src="./images/tube.png"><br />
    3D tube outline
</p>

<p align="center">
  <img src="./images/sensor-tube-types.png"><br />
    Left: "Grid type"; right: "Venturi type"
</p>



## DIY mass air flow meter - sensor code

### Design considerations

This project has so far been a prototype by some volunteers. It may be lacking some engineering and scientific background. To rework this prototype the following steps seem to make sense (need and should be discussed). A good analogon is an electric circuit where pressure is related to voltage and volume flow is related to volume flow.

1. Define the **peak volume flow** of your application. The peak volume flow may be derived from the maximum tidal volume and duration of the inspiration/expiration phases. Casually spoken: How fast is air delivered to the patient? Example: 200 liters per minute.
2. Define the **maximum pressure drop** allowed to be produced by the sensor at the peak volume flow, i.e. the resistance should not be too high as the pressure delivered to the patient is also reduced and might need to be compensated on the pressure source side. Example: 600 Pascal.
3.  **Select an appropriate differential pressure sensor** to cover the maximum pressure drop in a reasonable measurement range.
   1. Which kind of transducer is used - what is the output? Does it provide a digital interface (e.g. I2C or SPI) or an analogue voltage? How is the resolution?
   2. In case of an analogue voltage: What is the reference voltage, i.e. the measurement range of your ADC? How is the resolution? Is it sufficient to measure the voltage directly or is an operational amplifier required - or a voltage divider? Adding more circuitry (e.g. power stabilization/regulation, op-amp) also adds cost and requires space.
   3. How should the analogue low pass filter be designed?
4. **Calculate your geometry using the Venturi tube design**: the outer diameter might already be given by the tube sizes to adapt the meter for the rest of the system. The inner diameter is then determined: the peak volume flow shall lead to the maximum pressure drop allowed.

Disclaimer: The prototype is not based on these design steps!

### Overview

The sensor code is used to:

- Cyclically read the analog input (i.e. sampling).
- Convert the measured voltage which represents the differential pressure to volume flow.
  This contains internal calculation of the volume flow value or mapping via a look-up table.
- Provide the measurement via the I2C digital interface for readout.

Additional features:

- Provide the compatible I2C digital interface for readout of serial number.
- CRC calculation for the data packets to be interface compatible.
- Offset voltage calibration via internal EEPROM at startup.
- Verification of real-time conditions via an additional GPIO pin (for external timing measurements with an oscilloscope or logic analyzer).

The source code can be found [in this subdirectory of this repository](../sensor-software).



### Mapping/calculations

It's especially complicated to map the differential pressure to volume flow. Assuming formulas for the "Venturi type" tube, we can estimate a quadratic relation between volume flow and differential pressure:

<p align="center">
  <img src="./images/dv-dp-formula1.png">
</p>

Or:

<p align="center">
  <img src="./images/dv-dp-formula2.png">
</p>


<p align="center">
  <img src="./images/flow-calculation_60p.png">
</p>

However, when the DIY meter is compared with a reference meter, it shows that the constant factor K is around 15.0 and not as calculated around 6.7. This may be due to non-linearities, but the reason is currently not known for sure.

The MATLAB/Octave script can be found [here inside our repository](../sensor-software/VenturiCalcFlow.m).

### Known problems/issues

* [AnalogRead](https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/) accuracy in the Arduino platform: As stated [here](https://forum.arduino.cc/index.php?topic=109672.0), the "readings of the ADCs are always relative to the Vcc (usually 5V) if not configured for AREF. So if your Vcc varies your readings of the ADC will change appropriately". Some platforms take their Vcc directly from the USB port if not powered by an external power supply.
  In our current approach this may lead to inaccurate readings of the differential pressure sensor and therefore also wrong volume flow values. There's two possible solutions:
  1. Use of an accurate external power supply or a good on-board power regulator to provide a stable 5.0 Vcc voltage.
  2. [Configuring the reference voltage used for analog input](https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/) in the Arduino code: there's a variety of reference voltages to select from. "The default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards)." But: this depends on the chosen Arduino hardware. E.g. the value INTERNAL means something different for ATmega168/ATmega328P (1.1 volts) and ATmega32U4/ATmega8 (2.56 volts).
       The **Arduino Uno** is based on an ATmega328P, so 1.1 volts would be used. That's not a good fit, as the differential pressure sensor has a constant offset voltage of around 1.0 volts at 0 kPa differential pressure and the usable range is very small (from 1.0 to 1.1 volts).
       The **Arduino Leonardo** board is based on an ATmega32U4, so 1.1 volts would be used. That does **not** seem to be a good fit, as the differential pressure sensor has a constant offset voltage of around 1.0 volts at 0 kPa differential pressure and the usable range is very small (from 1.0 to 1.1 volts).
       The Arduino Leonardo board is based on an ATmega32u4, so 2.56 volts would be used here. Depending on the differential pressure sensor this could give readings between 0 and ~1500 Pa (from 1.0 to 2.56 volts).
* Vcc inaccuracy and drift might be problems related to the differential pressure sensor.



### About the sensor code

There are different pre-processor defines which influence how the source code is pre-processed and compiled.

You can:

- select between different models for the differential pressure sensor (currently between an NXP MPXV5004DP and an NXP MP3V5004DP),
- select betwenn our two 3D tube types: "Venturi type" and "Grid type",
- define if you want to use additional outputs for real-time verification,
- define if you want to use the built-in LED to indicate that the sensor is in measuring mode,
- (de)activate debug mode for more verbose logging via UART,
- change the measurement intervals (i.e. sampling time),
- define the sensor's bus address,
- write the offset voltage to the EEPROM (should be done only once).

The debug mode allows to dump raw and converted/calculated values to the serial console:

<p align="center">
  <img src="./images/sensor-debugging_60p.png">
</p>

Hint: The Arduino IDE offers a pretty neat feature to directly plot values from the serial console: the Serial Plotter. However, this is not optimal due to scaling of the curves. You might want to comment out several source code lines with *"debugPrint"* commands.

<p align="center">
  <img src="./images/sensor-debugging-plot_60p.png">
</p>


### Procedure to define offset voltage

Make the following changes to the code:

Uncomment the line reading:

```
eepromWriteOffsetVoltage( 0.0f );
```

Make sure to `#define DEBUG`.

Comment the line reading: `#define DEBUG_PRINT_STATUS`.

Make sure that the value initially reads `0.0f`.

Change the serial debug output to print only voltage values:

```
debugPrint( fVoltage ); /* value of interest for initial offset calibration */
debugPrintln("");
```

Compile and run the code on the sensor. Open the Serial Monitor in the IDE and collect the raw voltage values. Use a tool to calculate the mean value. Subtract `1000.0` (1000 millivolts, i.e. 1 volt) from the value and store the result in the call of `eepromWriteOffsetVoltage()`. E.g. a mean value of `1044.7` leads to:

```eepromWriteOffsetVoltage( 44.7f );```

Compile and run the code once more. You should now read a mean value of 1000 millivolts.

Revert your changes to the source code, especially do not forget to comment `eepromWriteOffsetVoltage()` out!



### Supported sensor commands + readout messages

The sensor supports the following 2 byte (16 bit) commands. Sending those commands leads to a mode change. In ever mode, the sensor will respond with differently formatted readout messages.

The commands listed in the first three rows are intended to be compatible with Sensirion flow meters:

| Command code | Command description                                          | Continous readout in that mode                               |
| ------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 0x10 0x00    | Start measurement                                            | 3 bytes:<br />2 bytes data + 1 byte CRC                      |
| 0x20 0x00    | Soft reset command                                           | N/A                                                          |
| 0x31 0xAE    | Read serial number                                           | 6 bytes:<br />2 bytes data + 1 byte CRC + <br />2 bytes data + 1 byte CRC |
| 0x42 0x00    | Start raw measurement, raw voltage                           | **DRAFT**<br />5 bytes:<br />4 bytes data (single-precision 32-bit floating point according to IEEE754) + 1 byte CRC;<br />unit: mV |
| 0x42 0x01    | Start raw measurement, <br />raw voltage offset removed/compensated | **DRAFT**<br />5 bytes:<br />4 bytes data (single-precision 32-bit floating point according to IEEE754) + 1 byte CRC;<br />unit: mV |
| 0x42 0x02    | Start raw measurement, <br />raw differential pressure       | **DRAFT**<br />5 bytes:<br />4 bytes data (single-precision 32-bit floating point according to IEEE754) + 1 byte CRC;<br />unit: kPa |
| 0x42 0x03    | Start raw measurement, <br />raw volume flow                 | **DRAFT**<br />5 bytes:<br />4 bytes data (single-precision 32-bit floating point according to IEEE754) + 1 byte CRC;<br />unit: liters per minute |
| 0x42 0x04    | Start raw measurement, <br />offset voltage                  | **DRAFT**<br />5 bytes:<br />4 bytes data (single-precision 32-bit floating point according to IEEE754) + 1 byte CRC;<br />unit: mV |
| 0x42 0x05    | Start raw measurement, <br />raw frequency                   | **DRAFT**<br />5 bytes:<br />4 bytes data (single-precision 32-bit floating point according to IEEE754) + 1 byte CRC;<br />unit: Hertz |




## Mass air flow meter - controller code

The sensor code is used to:

- Manage one or several sensor instances with different I2C addresses.
- Set the sensors into measuring mode.
- Cyclically read flow meter values from the I2C bus and convert them to floating point values in standard liters per minute.
- Provide the data via serial output (or other interface, e.g. store them in a data lake, etc.), so that they can be plotted/displayed or used by another higher-level component or implementation to control/monitor a ventilator.
  **Disclaimer:** This project is not intended to be used in a medical device!

Additional features:

- Read the serial numbers from the sensors.
- Perform soft reset of the sensors.
- Provide mechanism to check real-time requirements via an additional GPIO pin (for external timing measurements with an oscilloscope or logic analyzer).
- Debug the hardware setup by analyzing the return codes of the sensor library.
- As multiple sensors/meters are supported, it can be used to compare DIY solutions with ready-made reference sensors, i.e. support of verification.
- Detect breath cycles by analyzing the volume flow over time (integrate volume flow to calculate absolute volume for inspiration/expiration).
  **Disclaimer:** This project is not intended to be used in a medical device!

The source code can be found [in this subdirectory of this repository](../controller-software).

<p align="center">
  <img src="./images/multiple-sensors.png">
</p>


## High-level logic analyzer/decoder

We've added a high-level logic analyzer (also known as decoder) which allows to analyze the I2C protocol being used by the flow meters (commands and data between the meter and its controller):

<p align="center">
  <img src="./images/high-level-logic-analyzer-decoder.png">
</p>
It is based on the Saleae Logic2 application which uses their logic analyzer traces/dumps.

The source code can be found [in this subdirectory of this repository](../sensor-decoder).

Note: The analyzer is still work in progress. It might be extended to support verification of real-time requirements.



# References & ressources

## Similar projects

* [Robert L. Read](https://twitter.com/RobertLeeRead) hast compiled a [spreadsheet](https://docs.google.com/spreadsheets/d/1inYw5H4RiL0AC_J9vPWzJxXCdlkMLPBRdPgEVKF8DZw/edit#gid=1022772303) of open source ventilator projects and modules such as mass air flow sensors
* "Freeflow flow sensor" on [hackaday.io](https://hackaday.io/project/170928-freeflow-flow-sensor) as well as on [github](https://github.com/sglow/freeflow)
* "VISP - Ventilator Inline Sensor Package" on [hackaday.io](https://hackaday.io/project/170622-visp-ventilator-inline-sensor-package)
* "VentMon - Inline Ventilator Test Fixture and Monitor" on [github](https://github.com/PubInv/ventmon-ventilator-inline-test-monitor)
* "OsciBreath" on [github](https://github.com/FDX-Fluid-Dynamix/OsciBreath) uses an microphone- and FFT-based approach to determine volume flow 
* [Uni Marburg The Breathing Project](https://www.uni-marburg.de/de/fb13/halbleiterphotonik/the-breathing-project/the-breathing-project-1)

## Flow measurement basics

* [Sensirion Application Note for SDP600 and SDP1000 Series: Measuring Flow in a Bypass Configuration](https://www.sos.sk/a_info/resource/c/sensirion/SDPxxx_Bypass_Configuration.pdf)
* Robert Haas' [PipeFlow project on github](https://github.com/Haasrobertgmxnet/PipeFlow/tree/master/PipeFlow); more documentation with formulas in the [PipeFlow wiki](https://github.com/Haasrobertgmxnet/PipeFlow/wiki)

