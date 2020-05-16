# IoT oximeter

## Introduction

The World Health Organization (WHO) said "The oxygen saturation in healthy patients of any age should be 95% or above." ([source)](https://www.who.int/patientsafety/safesurgery/pulse_oximetry/who_ps_pulse_oxymetry_training_manual_en.pdf).

In the context of covid-19 I decided to make a connected oximeter to check heart rate and oxygen saturation (spo2). The data are sent and displayed in a mobile ready dashboard with QuickSight from Amazon Web Services.

You can find the full project [here](https://www.hackster.io/martin-cornu/iot-oximeter-for-covid-19-538346).

The IoT oximeter is built on Arduino mkrfox1200 (SAMD21 MCU). Heart rate and Spo2 are measured from MAX30102 sensor and data are displayed on an I2C OLED screen.

The data are sent over the Sigfox LPWA Network and the device can run for years with two AA cells.

## Circuit

Here is the circuit:

![circuit](https://github.com/martincornu/pulse-oximeter-arduino/blob/develop/hardware/sigfox-oximeter-proto_bb.png)

By default the interrupt (INT) pin of the MAX30102 is connected to the pin 10. You can change it in the code. 

Both OLED and MAX30102 work with I2C protocol. You can plug SDA and SCL pins of the MKRFOX1200 to the breadboard and then plug both OLED and MAX30102 pins. Same for VCC and GND. 

If you want an autonomous oximeter you can use an external power instead of USB power. Plug the positive pin of external power to VIN (5V maximum) and  ground to GND.

## The program

The program is easily customizable by commenting #define at the top. You can choose to use or not sigfox, OLED display and between continuous or single measurement mode.

In order to run the MAX30102 driver on the mkrfox1200 board (with SAMD21 MCU) we need a different library than the original MAX one because they written it for ATmega MCU (like on Arduino UNO). I found this great [library](https://github.com/aromring/MAX30102_by_RF) written by Fraczkiewicz, R.

- Download the files on the repository
- Copy these files algorithm_by_RF.cpp, algorithm_by_RF.h, max30102.cpp and max30102.h one level above where your Arduino .ino file is located.
- If you wanna use the OLED display you can download the SSD1306Ascii library directly from the Arduino library manager.

You can now upload the code in the Arduino and try your connected oximeter! In debug mode (uncomment DEBUG define) the measure starts after pressing any key in the serial monitor.