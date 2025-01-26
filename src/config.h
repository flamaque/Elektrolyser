#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/pcnt.h"   //Needed for flow sensor
#include "esp32-hal-adc.h" //Needed for ADC
#include "stdint.h"        //Needed for uint
#include "MQUnifiedsensor.h"
#include <U8g2lib.h>           //Needed for display
#include <DallasTemperature.h> //Needed for DS18B20
#include <OneWire.h>           //Needed for DS18B20, DHT22 and AGS02MA
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <sstream>
#include <iomanip>     //Needed to use  setprecision
#include <nvs_flash.h> //Used for storing timestmp
#include <WiFi.h>
#include "esp_wifi.h"
#include <HTTPClient.h>
// #include "SD_MMC.h"
#include <SPI.h>
#include <SD.h>
#include "Adafruit_AGS02MA.h" //Asair AGS02MA TVOC Gas Sensor
#include <HardwareSerial.h>   //Needed for GSM
#include "esp_bt.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_task_wdt.h"      //Needed for Watchdog timer for Bluetooth task
#include "ESP_PH.h" // library for the PH sensor
#include "DFRobot_ESP_EC.h" // library for the conductivity sensor

void Counting(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray_WiFi(void *parameter);
void BluetoothListen(void *parameter);

/*      GSM     */
void initialize_gsm();
void readGsmResponse();
String readGsmResponse3();
void getTime();
String getDateTime_SIM7600();
void saveTimestamp(uint64_t timestamp_ms);
uint64_t getSavedTimestamp_GSM();
uint64_t convertToUnixTimestamp(String date, String time);
extern const String apn, apn_User, apn_Pass;
extern String date_getTime, response, datetime_gsm;
extern uint64_t savedTimestamp, timestamp_ms, unixTimestamp;
extern char httpapi[]; // Removed because in main.cpp it's declared as String
extern HardwareSerial gsmSerial;

void AGS02MA_Init();
extern Adafruit_AGS02MA ags;

extern OneWire oneWire;                 // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
extern DallasTemperature sensors;       // Pass our oneWire reference to Dallas Temperature.
extern DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

/*      Display      */
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled;
void init_displays();
// For GSMSerial output on OLED
extern U8G2LOG u8g2log;
extern volatile uint8_t stateBigOled, stateDebug;

/* voltage sensor */
float readVoltage();

/*      DS18B20 sensor       */
extern uint8_t DS18B20_PIN;
void printDS18B20Address();
// void AllDS18B20Sensors();

/*      MQ-7 MQ-8 sensor       */
void mq7_init(MQUnifiedsensor &MQ7);
void mq8_init(MQUnifiedsensor &MQ8);

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
uint64_t getSavedTimestamp_WiFi();
extern uint64_t savedTimestamp;

extern uint8_t SD_CS_PIN, GSM_RX_PIN, GSM_TX_PIN, Pin_MQ8, DHT_SENSOR_PIN, DS18B20_PIN, flowSensorPin, EC_PIN, PH_PIN, NTC_PIN, voltPin, CurrentPin;
extern String mobileNumber;
extern float flowSensorCalibration;
extern uint8_t h2Amount, DFcoAmount, AGS02MAAmount, flowRateAmount, temperatureAmount, humidityAmount, phValueAmount, ecValueAmount, ds18b20Amount, voltAmount, powerAmount, acsAmount, TempFlowAmount;
void processLine(String line);
void read_configuration();
void printVariables();
void initSD();

/*      Bluetooth      */
extern BLEServer* pServer;
extern BLEService* pService;
extern BLECharacteristic* pCharacteristic;
extern bool deviceConnected;
// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//Suited for Serial Port Profile (SPP) service
#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

extern String message;
extern char incomingChar;
extern bool bluetooth_connected;
void initBLE();

/*      Ph Sensor         */
extern ESP_PH ph;
extern uint8_t PH_PIN;
float pH();
//float readTemperature();

/*      Conductivity Sensor   */
extern DFRobot_ESP_EC ec;
//extern float voltage_cond, temperature_cond;
extern uint8_t EC_PIN; // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
float Cond();

#endif // CONFIG_H
