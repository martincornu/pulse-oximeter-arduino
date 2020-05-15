/********************************************************
*
* Project: IoT oximeter
* Filename: pulse-oximeter-arduino.ino
* Description: This program measure hear rate and spo2 from max30102 sensor and send data to Sigfox.
*				It is easily customizable by commenting the different #define.
*
* Revision History:
*\n 1-18-2016 Rev 01.00 GL Initial release.
*\n 12-22-2017 Rev 02.00 Significantlly modified by Robert Fraczkiewicz (max algorithm)
*\n 14-11-2020 Rev 03.00 Modified by Martin Cornu (oled, sigfox)
* ------------------------------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"

//#define DEBUG // Uncomment for debug output to the Serial stream
#define SEND_DATA_SIGFOX  //Uncomment if you want raw data to be send over sigfox network
#define ONE_SHOT  //Uncomment if you want a single measure + send and then stop program
#define USE_OLED  //Uncomment if you want to use oled display

#ifdef SEND_DATA_SIGFOX
  #define SPO2_MAX    100
  #define HR_MAX      250
  #define RATIO_MAX   1
  #define CORREL_MAX  1
#endif

#ifdef USE_OLED
  #define OLED_I2C_ADDR 0x3C
#endif

#ifdef SEND_DATA_SIGFOX
  #include <ArduinoLowPower.h>
  #include <SigFox.h>
#endif

#ifdef USE_OLED
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiWire.h"
#endif

// Interrupt pin
const byte oxiInt = 10; // pin connected to MAX30102 INT

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;

#ifdef SEND_DATA_SIGFOX
  typedef struct __attribute__ ((packed)) sigfox_message {
    uint16_t spo2;
    uint8_t heart_rate;
    uint16_t  moduleTemp;
  } SigfoxMessage;
  
  // stub for message which will be sent
  SigfoxMessage msg;
#endif // SEND_DATA_SIGFOX

#ifdef USE_OLED
  SSD1306AsciiWire oled;
#endif

void setup() {
  uint8_t uch_maxinit_ret = 0;
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

  Wire.begin();

#if defined(DEBUG)
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
#endif

#ifdef SEND_DATA_SIGFOX
  if (!SigFox.begin()) {
    // Something is really wrong, try rebooting
    // Reboot is useful if we are powering the board using an unreliable power source
    // (eg. solar panels or other energy harvesting methods)
    reboot();
  }
  //Send module to standby until we need to send a message
  SigFox.end();
  #ifdef DEBUG
    // Enable debug prints and LED indication if we are testing
    SigFox.debug();
  #endif // DEBUG
#endif // SEND_DATA_SIGFOX

#ifdef USE_OLED
  oled.begin(&Adafruit128x64, OLED_I2C_ADDR);
  oled.setFont(Adafruit5x7);
#endif

  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  uch_maxinit_ret = (uint8_t) maxim_max30102_init();  //initialize the MAX30102
  
  if (0 == uch_maxinit_ret) {
    #ifdef DEBUG
    Serial.println("Sensor MAX30102 init FAIL.");
    #endif
    #ifdef USE_OLED  
    oled.clear();
    oled.println();
    oled.println();
    oled.set2X();
    oled.println("Sensor init");
    oled.println("fail");
    oled.set1X();
    oled.println("Please reboot");
    #endif // USE_OLED 
  }
  else {
    #ifdef DEBUG
    Serial.println("Sensor MAX30102 init SUCCESS.");
    #endif
    #ifdef USE_OLED  
    oled.clear();
    oled.println();
    oled.println();
    oled.set2X();
    oled.println("Measure");
    oled.println("in");
    oled.println("progress..");
    #endif // USE_OLED 
  }

  old_n_spo2=0.0;

#ifdef DEBUG
  while(Serial.available()==0)  //wait until user presses a key
  {
    Serial.println(F("Press any key to start conversion"));
    delay(1000);
  }
  uch_dummy=Serial.read();
  Serial.print(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr"));
  Serial.println("");
#endif // DEBUG
  
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

  //display and/or send data if they are valid
  if(ch_hr_valid && ch_spo2_valid) { 
  #ifdef DEBUG
    Serial.print(elapsedTime);
    Serial.print("\t");
    Serial.print(n_spo2);
    Serial.print("\t");
    Serial.print(n_heart_rate, DEC);
    Serial.print("\t");
    Serial.print(hr_str);
    Serial.print("\t");
    Serial.print(ratio);
    Serial.print("\t");
    Serial.print(correl);
  #endif // DEBUG

  #ifdef USE_OLED  
    oled.clear();
    oled.set1X();
    oled.println("MEASURE DONE");
    oled.println();
    oled.set2X();
    oled.print("Spo2:");
    oled.println(n_spo2);
    oled.print("HR:");
    oled.println(n_heart_rate);;
  #endif // USE_OLED 
    
  #ifdef SEND_DATA_SIGFOX
    oled.set1X();
    oled.println();
    oled.print("Sending data...");
    msg.spo2 = (uint16_t) (n_spo2 * 100);
    msg.heart_rate = (uint8_t) n_heart_rate;
    
    // Start the sigfox module
    SigFox.begin();
    // Wait at least 30ms after first configuration (100ms before)
    delay(100);
    // We can only read the module temperature before SigFox.end()
    msg.moduleTemp = (uint16_t) (SigFox.internalTemperature() * 100);
    // Clears all pending interrupts
    SigFox.status();
    delay(1);
    SigFox.beginPacket();
    SigFox.write((uint8_t*)&msg, sizeof(msg));
    SigFox.endPacket();
    SigFox.end(); 

    //Display values sent to sigfox
  #ifdef DEBUG
    Serial.println();
    Serial.println("Sigfox attributs after conversion to uint:");
    Serial.print("SPO2: "); Serial.println(msg.spo2, DEC);
    Serial.print("HR: "); Serial.println(msg.heart_rate, DEC);
    Serial.print("moduleTemp: "); Serial.println(msg.moduleTemp, DEC);
  #endif //DEBUG

  // Display send info on oled
  #ifdef USE_OLED  
    oled.setCursor(0, oled.row());
    oled.clearToEOL();
    oled.setCursor(0, oled.row());
    oled.print("Data sent");
  #endif // USE_OLED
  #ifdef DEBUG
    Serial.println("Data sent");
  #endif // DEBUG  
  #endif //SEND_DATA_SIGFOX 

  #ifdef ONE_SHOT
    // spin forever
    while (1) {};
  #endif
  #ifdef DEBUG
    Serial.println("");
  #endif
  
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

void reboot() {
  NVIC_SystemReset();
  while (1);
}
