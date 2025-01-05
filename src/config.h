#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/pcnt.h"        //Needed for flow sensor
#include "esp32-hal-adc.h"      //Needed for ADC
#include "stdint.h"             //Needed for uint
#include "MQUnifiedsensor.h"
#include <U8g2lib.h>            //Needed for display
#include <DallasTemperature.h>  //Needed for DS18B20
#include <OneWire.h>            //Needed for DS18B20, DHT22 and AGS02MA
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <sstream>
#include <iomanip>          //Needed to use  setprecision	
#include <nvs_flash.h>      //Used for storing timestmp
#include <WiFi.h>
#include <HTTPClient.h>
#include "BluetoothSerial.h"
// #include "SD_MMC.h"
#include <SPI.h>
#include <SD.h>
#include "Adafruit_AGS02MA.h"        //Asair AGS02MA TVOC Gas Sensor

void Counting(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray_WiFi(void *parameter);

extern uint8_t SD_CS_PIN;
void AGS02MA_Init();
extern Adafruit_AGS02MA ags;

extern OneWire oneWire;                 // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
extern DallasTemperature sensors;       // Pass our oneWire reference to Dallas Temperature.
extern DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

/*      Display      */
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C 
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled;
void init_displays();

/* voltage sensor */
float readVoltage();

/*      DS18B20 sensor       */
extern uint8_t DS18B20_PIN;
void printDS18B20Address();
//void AllDS18B20Sensors();

/*      MQ-7 MQ-8 sensor       */
void mq7_init(MQUnifiedsensor& MQ7);
void mq8_init(MQUnifiedsensor& MQ8);


/* Volt sensor */
extern uint8_t voltPin;

/* Flow sensor */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num);
extern uint8_t NTC_PIN;
extern volatile float flowRate, flowRate2;

/* NTC */
float Read_NTC();
extern float steinhart, temp_flow;
extern uint8_t NTC_PIN, TEMPERATURENOMINAL;
extern int serialResistance;
extern const uint8_t NUMSAMPLES;
extern uint16_t nominalResistance, bCoefficient;

/*      Current Sensor   */
extern uint8_t CurrentPin;
float CurrentSensor_724();

void printLocalTime();
void getTime_WiFi();
uint64_t getSavedTimestamp();
extern uint64_t savedTimestamp;
#endif // CONFIG_H
