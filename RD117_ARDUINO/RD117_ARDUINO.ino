/********************************************************
*
* Project: MAXREFDES117#
* Filename: RD117_ARDUINO.ino
* Description: This module contains the Main application for the MAXREFDES117 example program.
*
* Revision History:
*\n 1-18-2016 Rev 01.00 GL Initial release.
*\n 12-22-2017 Rev 02.00 Significantlly modified by Robert Fraczkiewicz
*\n 08-22-2018 Rev 02.01 Added conditional compilation of the code related to ADALOGGER SD card operations
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"

#define DEBUG // Uncomment for debug output to the Serial stream
//#define TEST_MAXIM_ALGORITHM // Uncomment if you want to include results returned by the original MAXIM algorithm
//#define SAVE_RAW_DATA // Uncomment if you want raw data coming out of the sensor saved to SD card. Red signal first, IR second.

#ifdef TEST_MAXIM_ALGORITHM
  #include "algorithm.h" 
#endif

// Interrupt pin
const byte oxiInt = 10; // pin connected to MAX30102 INT

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;

void setup() {

  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

  Wire.begin();

#if defined(DEBUG)
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
#endif

  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);

  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();  //initialize the MAX30102
  old_n_spo2=0.0;

  while(Serial.available()==0)  //wait until user presses a key
  {
    Serial.println(F("Press any key to start conversion"));
    delay(1000);
  }
  uch_dummy=Serial.read();
#ifdef TEST_MAXIM_ALGORITHM
  Serial.print(F("Time[s]\tSpO2\tHR\tSpO2_MX\tHR_MX\tClock\tRatio\tCorr"));
#else // TEST_MAXIM_ALGORITHM
  Serial.print(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr"));
#endif // TEST_MAXIM_ALGORITHM
#ifdef SAVE_RAW_DATA
  int32_t i;
  // These are headers for the red signal
  for(i=0;i<BUFFER_SIZE;++i) {
    Serial.print("\t");
    Serial.print(i);
  }
  // These are headers for the infrared signal
  for(i=0;i<BUFFER_SIZE;++i) {
    Serial.print("\t");
    Serial.print(i);
  }
#endif // SAVE_RAW_DATA
  Serial.println("");
  
  timeStart=millis();
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
     
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
#ifdef DEBUG
    Serial.print(i, DEC);
    Serial.print(F("\t"));
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print(F("\t"));
    Serial.print(aun_ir_buffer[i], DEC);    
    Serial.println("");
#endif // DEBUG
  }

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
  elapsedTime=millis()-timeStart;
  millis_to_hours(elapsedTime,hr_str); // Time in hh:mm:ss format
  elapsedTime/=1000; // Time in seconds

#ifdef DEBUG
  Serial.println("--RF--");
  Serial.print(elapsedTime);
  Serial.print("\t");
  Serial.print(n_spo2);
  Serial.print("\t");
  Serial.print(n_heart_rate, DEC);
  Serial.print("\t");
  Serial.println(hr_str);
  Serial.println("------");
#endif // DEBUG

#ifdef TEST_MAXIM_ALGORITHM
  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using MAXIM's method
  float n_spo2_maxim;  //SPO2 value
  int8_t ch_spo2_valid_maxim;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate_maxim; //heart rate value
  int8_t  ch_hr_valid_maxim;  //indicator to show if the heart rate calculation is valid
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2_maxim, &ch_spo2_valid_maxim, &n_heart_rate_maxim, &ch_hr_valid_maxim); 
#ifdef DEBUG
  Serial.println("--MX--");
  Serial.print(elapsedTime);
  Serial.print("\t");
  Serial.print(n_spo2_maxim);
  Serial.print("\t");
  Serial.print(n_heart_rate_maxim, DEC);
  Serial.print("\t");
  Serial.println(hr_str);
  Serial.println("------");
#endif // DEBUG
#endif // TEST_MAXIM_ALGORITHM

  //save samples and calculation result to SD card
#ifdef TEST_MAXIM_ALGORITHM
  if(ch_hr_valid && ch_spo2_valid || ch_hr_valid_maxim && ch_spo2_valid_maxim) {
#else   // TEST_MAXIM_ALGORITHM
  if(ch_hr_valid && ch_spo2_valid) { 
#endif // TEST_MAXIM_ALGORITHM

    Serial.print(elapsedTime);
    Serial.print("\t");
    Serial.print(n_spo2);
    Serial.print("\t");
    Serial.print(n_heart_rate, DEC);
    Serial.print("\t");
#ifdef TEST_MAXIM_ALGORITHM
    Serial.print(n_spo2_maxim);
    Serial.print("\t");
    Serial.print(n_heart_rate_maxim, DEC);
    Serial.print("\t");
#endif //TEST_MAXIM_ALGORITHM
    Serial.print(hr_str);
    Serial.print("\t");
    Serial.print(ratio);
    Serial.print("\t");
    Serial.print(correl);
#ifdef SAVE_RAW_DATA
    // Save raw data for unusual O2 levels
    for(i=0;i<BUFFER_SIZE;++i)
    {
      Serial.print(F("\t"));
      Serial.print(aun_red_buffer[i], DEC);
    }
    for(i=0;i<BUFFER_SIZE;++i)
    {
      Serial.print(F("\t"));
      Serial.print(aun_ir_buffer[i], DEC);    
    }
#endif // SAVE_RAW_DATA
    Serial.println("");

    old_n_spo2=n_spo2;
  }
}

void millis_to_hours(uint32_t ms, char* hr_str)
{
  char istr[6];
  uint32_t secs,mins,hrs;
  secs=ms/1000; // time in seconds
  mins=secs/60; // time in minutes
  secs-=60*mins; // leftover seconds
  hrs=mins/60; // time in hours
  mins-=60*hrs; // leftover minutes
  itoa(hrs,hr_str,10);
  strcat(hr_str,":");
  itoa(mins,istr,10);
  strcat(hr_str,istr);
  strcat(hr_str,":");
  itoa(secs,istr,10);
  strcat(hr_str,istr);
}
