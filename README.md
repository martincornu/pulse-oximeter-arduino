# IoT oximeter

## Introduction

The covid-19 challenge by Hackster.io gave me the idea to make a connected oximeter. 

It allows to check our heart rate and oxygen saturation (spo2). The World Health Organization (WHO) says "The oxygen saturation in healthy patients of any age should be 95% or above." in this [document](https://www.who.int/patientsafety/safesurgery/pulse_oximetry/who_ps_pulse_oxymetry_training_manual_en.pdf).

The IoT oximeter is build on Arduino mkrfox1200 (SAMD21 MCU). Heart rate and Spo2 are measured from MAX30102 sensor and data are display on an I2C OLED screen. 

The data are sent over the Sigfox LPWA Network. No need to pair a smartphone, it is easy to use and the device can run for months/years with two AA cells (power consumption analysis is coming...).

## Circuit

Here is the circuit. Both OLED and MAX30102 are communicating with I2C. You can find the source file in the repo at hardware/sigfox-oximeter-proto.fzz

![](D:\Projets\oximeter\pulse-oximeter-arduino\hardware\sigfox-oximeter-proto_bb.png)

## The program

You can customize the program by comment/uncomment the #define at the beginning of the .ino file.

If you wanna use OLED screen then you need to download the SSD1306Ascii library (available in the Arduino Library manager).

Regarding the MAX30102 sensor I successfully ran it on SAMD21 MCU thanks to this great [repository](https://github.com/aromring/MAX30102_by_RF). It propose a re-written MAX30102 drivers and an algorithm with more accuracy measurement. All details are explained on this [instructable](https://www.instructables.com/id/Pulse-Oximeter-With-Much-Improved-Precision/).