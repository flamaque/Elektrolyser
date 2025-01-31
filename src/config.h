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
#include "SPIFFS.h" //For saving the config.txt to filesystem
#include "DHT.h"

struct Configuration {
    String ssid;
    String password;
    String httpapi;
    String connectionMode;  // Can be "GSM" or "WiFi"
    String apn;
    String apn_User;
    String apn_Pass;
    String mobileNumber;
    bool gsmMode;
    bool wifiMode;
};
extern struct Configuration configuration;

struct ConfigPin {
    uint8_t GSM_RX_PIN;
    uint8_t GSM_TX_PIN;
    uint8_t Pin_MQ8;
    uint8_t DHT_SENSOR_PIN;
    uint8_t DS18B20_PIN;
    uint8_t flowSensorPin;
    uint8_t EC_PIN;
    uint8_t PH_PIN;
    uint8_t voltPin;
    uint8_t CurrentPin;
    uint8_t SW_420_Pin;
};
extern struct ConfigPin configPin;

struct ConfigNumeric {
    uint8_t temperatureAmount;
    uint8_t humidityAmount;
    uint8_t h2Amount;
    uint8_t flowRateAmount;
    uint8_t phValueAmount;
    uint8_t ecValueAmount;
    uint8_t ds18b20Amount;
    uint8_t voltAmount;
    uint8_t powerAmount;
    uint8_t acsAmount;
    uint8_t SW420Amount;
    float flowSensorCalibration;
};
extern struct ConfigNumeric configNumeric;

extern uint8_t numMeasurements;
struct ConfigInterval {
    uint8_t dht22_tempInterval;
    uint8_t dht22_humInterval;
    uint8_t h2Interval;
    uint8_t flowRateInterval;
    uint8_t phValueInterval;
    uint8_t ecValueInterval;
    uint8_t ds18b20Interval;
    uint8_t voltInterval;
    uint8_t powerInterval;
    uint8_t acsInterval;
    uint8_t SW420Interval;
};
extern struct ConfigInterval configInterval;
void initializeConfigInterval();

void Counting(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray_WiFi(void *parameter);
void BluetoothListen(void *parameter);

/*      WiFi    */
void printLocalTime();
void getTime_WiFi();
uint64_t getSavedTimestamp_WiFi();

/*      GSM     */
void initialize_gsm();
void readGsmResponse();
String readGsmResponse3();
String readGsmResponse5();
uint64_t parseResponse(String resp);
// String getDateTime_SIM7600();
void IsGSMConnected();
void saveTimestamp(uint64_t timestamp_ms);
void getDateTime_GSM();
uint64_t getSavedTimestamp_GSM();
uint64_t convertToUnixTimestamp(String date, String time);
extern time_t timestamp;
extern String date_getTime, response, datetime_gsm;
extern uint64_t timestamp_ms, unixTimestamp;
extern char httpapi[]; // Removed because in main.cpp it's declared as String
extern char DateTimeBuffer[50], TimeBuffer[25], TimeBufferDis[25], TimeBufferFinal[32];
extern HardwareSerial gsmSerial;
extern bool gsmConnected;

/*      DS18B20 sensor       */
extern OneWire oneWire;                 // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
extern DallasTemperature sensors;       // Pass our oneWire reference to Dallas Temperature.
extern DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

/*      Display      */
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled;
void init_displays();
// For GSMSerial output on OLED
extern U8G2LOG u8g2log;
extern volatile uint8_t GSMOutputToOLED, stateDebug;

/* voltage sensor */
float readVoltage();

/*  DHT22 sensor    */

/*      DS18B20 sensor       */
void printDS18B20Address();
// void AllDS18B20Sensors();

/*      MQ-8 sensor       */
void mq8_init(MQUnifiedsensor &MQ8);

/* Volt sensor */

/* Flow sensor */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num);
extern volatile float flowRate;

/*      Current Sensor   */
extern uint8_t CurrentPin;
float CurrentSensor_724();

// extern uint8_t SD_CS_PIN, GSM_RX_PIN, GSM_TX_PIN, Pin_MQ8, DHT_SENSOR_PIN, DS18B20_PIN, flowSensorPin, EC_PIN, PH_PIN, voltPin, CurrentPin, SW_420_Pin;

// extern uint8_t SW420Amount, h2Amount, flowRateAmount, temperatureAmount, humidityAmount, phValueAmount, ecValueAmount, ds18b20Amount, voltAmount, powerAmount, acsAmount;
void processLine(const String &line);
bool read_configuration();
void printVariables();
void initSD();
extern const uint8_t SD_CS_PIN;

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
// extern uint8_t PH_PIN;
float pH();
//float readTemperature();

/*      Conductivity Sensor   */
extern DFRobot_ESP_EC ec;
//extern float voltage_cond, temperature_cond;
// extern uint8_t EC_PIN; // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
float Cond();

#endif // CONFIG_H
