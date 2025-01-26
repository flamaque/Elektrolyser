#include "config.h"
// String httpapi = "http://jrbubuntu.ddns.net:5000/api/telemetry"; // Not yet tested as String
char httpapi[] = "http://145.131.6.212/api/v1/HR/gl3soo07qchjimbsdwln/telemetry";
const char *serverName = "http://jrbubuntu.ddns.net:5000/api/telemetry";
uint64_t savedTimestamp;

SemaphoreHandle_t i2c_mutex, OneLogMutex, fileMutex = NULL; // Handler for the log.txt file
bool MeasurementReady = false;

/*      GSM Module Setup     */
HardwareSerial gsmSerial(1); // Use UART1
uint8_t GSM_RX_PIN = 44;     // SIM7600E TX
uint8_t GSM_TX_PIN = 43;     // SIM7600E RX
String mobileNumber = "+31614504288";
const String apn = "data.lycamobile.nl";
const String apn_User = "lmnl";
const String apn_Pass = "plus";

/*      Bluetooth                       */
String message = "";
String BtConnected;

/*      Setup Wifi                      */
// const char *ssid = "H369A2606A5";
// const char *password = "9EAD76CC35C6";
// const char *ssid = "Ziggo0542186";
// const char *password = "bu6cjcwg3fpuhwwN";
const char *ssid = "JRB";
const char *password = "driekeerraden";
// const char *ssid = "Loading...";
// const char *password = "driekeerraden123";

const char *ntpServer = "pool.ntp.org"; // NTP server to request epoch time
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
String Connected = "";

bool gsmMode, wifiMode = false;

/*      SD card                         */
// SPIClass spi = SPIClass(HSPI);                   // VSPI
uint8_t SD_CS_PIN = 46;
// #define SD_CS_PIN 46
// Add these definitions at the top
// #define SD_MOSI 11  // SPI MOSI
// #define SD_MISO 13  // SPI MISO
// #define SD_SCK  12  // SPI CLK

// /*      MQ-7 CO sensor                  */
// uint8_t Pin_MQ7 = 2;
// MQUnifiedsensor MQ7("ESP32", 5, 12, Pin_MQ7, "MQ-7");
/*      MQ-8 H2 sensor                   */
uint8_t Pin_MQ8 = 1;
MQUnifiedsensor MQ8("ESP32", 5, 12, Pin_MQ8, "MQ-8");
/*      DF Robot CO sensor              */
uint8_t Pin_CO = 15;
// uint8_t CO_Value = 0;
uint32_t CO_Value = 0; // Changed to uint32_t because uint8_t was giving a weird character in the JSON file

/*      EC sensor              */
uint8_t EC_PIN = 39;

/*      PH sensor              */
ESP_PH ph;
uint8_t PH_PIN = 3;

/*      Asair AGS02MA TVOC Gas Sensor   */
Adafruit_AGS02MA ags;
uint32_t resistance, AGS02MA_Value;

/*      DHT22 - Temperature and Humidity */
#include "DHT.h"
uint8_t DHT_SENSOR_PIN = 6; // 25; Changed for trying WiFi
#define DHT_SENSOR_TYPE DHT22
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

/*      Setup Temperature sensor        */
uint8_t DS18B20_PIN = 4; // 26; Changed for trying WiFi

/*      Setup Flowsensor                */
#define PCNT_INPUT_SIG_IO1 flowSensorPin  // Pulse Input GPIO for PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO2 flowSensor2Pin // Pulse Input GPIO for PCNT_UNIT_1
#define PCNT_UNIT1 PCNT_UNIT_0
#define PCNT_UNIT2 PCNT_UNIT_1

volatile float flowRate = 0.00;
uint8_t flowSensorPin = 5;
float flowSensorCalibration = 21.00;
float frequency1 = 0.0;

uint8_t voltPin = 15;
// uint8_t NTC_PIN = 3;

/*          Current sensor                  */
uint8_t CurrentPin = 8;

float t, phvalue, stroom, Volt, DS18B20_1, DS18B20_2, DS18B20_3, DS18B20_4, DS18B20_5, DS18B20_6, humidity, ecValue, ppmCO, ppmH, power;
float *ds18b20Sensors[] = {&DS18B20_1, &DS18B20_2, &DS18B20_3, &DS18B20_4, &DS18B20_5};

/*          Test for Array of JSON Objects         */
// Ctrl + d for multiple cursors
int currentMeasurementIndex = 0;
int numMeasurements_Placeholder = 0;

uint8_t h2Amount = 5;
uint8_t coAmount = 5;
uint8_t DFcoAmount = 5;
uint8_t AGS02MAAmount = 5;
uint8_t flowRateAmount = 5;
uint8_t flowRate2Amount = 5;
uint8_t temperatureAmount = 5;
uint8_t humidityAmount = 4;
uint8_t phValueAmount = 2;
uint8_t ecValueAmount = 2;
uint8_t ds18b20Amount = 8;
uint8_t voltAmount = 5;
uint8_t powerAmount = 5;
uint8_t acsAmount = 5;
uint8_t TempFlowAmount = 5;

const uint8_t numMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, h2Amount, coAmount, voltAmount, TempFlowAmount, powerAmount, DFcoAmount, AGS02MAAmount});
const uint8_t totMeasurements = temperatureAmount + phValueAmount + humidityAmount + ecValueAmount + flowRateAmount + flowRate2Amount + acsAmount + ds18b20Amount + h2Amount + coAmount + voltAmount + TempFlowAmount + powerAmount + DFcoAmount + AGS02MAAmount;

const uint8_t dht22_tempInterval = numMeasurements / temperatureAmount;
const uint8_t AGS02MAInterval = numMeasurements / AGS02MAAmount;
const uint8_t DFcoInterval = numMeasurements / DFcoAmount;
const uint8_t phValueInterval = numMeasurements / phValueAmount;
const uint8_t dht22_humInterval = numMeasurements / humidityAmount;
const uint8_t ecValueInterval = numMeasurements / ecValueAmount;
const uint8_t flowRateInterval = numMeasurements / flowRateAmount;
const uint8_t flowRate2Interval = numMeasurements / flowRate2Amount;
const uint8_t acsValueFInterval = numMeasurements / acsAmount;
const uint8_t ds18b20Interval = numMeasurements / ds18b20Amount;
const uint8_t voltInterval = numMeasurements / voltAmount;
const uint8_t powerInterval = numMeasurements / powerAmount;
const uint8_t h2Interval = numMeasurements / h2Amount;
const uint8_t coInterval = numMeasurements / coAmount;
const uint8_t FlowTempinterval = numMeasurements / TempFlowAmount;

TaskHandle_t Task_Bluetooth, Task_Counting, Task_Display, Task_Measuring, Task_sendArrayWifi = NULL;

void sendArray_WiFi(void *parameter)
{
  vTaskDelay(200 / portTICK_PERIOD_MS);
  Serial.println("Now running sendArray WiFi task.");
  unsigned long previousTime = 0;
  for (;;)
  {
    // Wait for measurements to be ready
    while (!MeasurementReady)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      unsigned long currentTime = millis();
      unsigned long timeBetweenUsage = currentTime - previousTime;
      previousTime = currentTime;

      WiFiClient client;
      HTTPClient http;
      http.begin(client, serverName);
      http.addHeader("Content-Type", "application/json");
      if (xSemaphoreTake(OneLogMutex, portMAX_DELAY) == pdTRUE)
      {
        Serial.println("Received OneLogMutex and inside if statement");
        File OneLog = SD.open("/One_Measurement.txt", FILE_READ);
        if (!OneLog)
        {
          Serial.println("Error opening OneLog file");
          Serial.println("Retrying.");
          vTaskDelay(500 / portTICK_PERIOD_MS);
          File OneLog = SD.open("/One_Measurement.txt", FILE_READ);
          // return;
        }

        size_t fileSize = OneLog.size();
        Serial.println();
        Serial.println("Buffer size in sendArray: " + String(fileSize));

        int httpResponseCode = http.sendRequest("POST", &OneLog, fileSize);

        if (httpResponseCode > 0)
        {
          String response = http.getString();
          Serial.println(httpResponseCode);
          Serial.println(response);
        }
        else
        {
          Serial.print("Error on sending POST: ");
          Serial.println(httpResponseCode);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
        OneLog.close();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        http.end();
        xSemaphoreGive(OneLogMutex);

        int seconds = timeBetweenUsage / 1000;
        int minutes = seconds / 60;
        int remainingSeconds = seconds % 60;
        Serial.println("Time between usage: " + String(minutes) + " min " + String(remainingSeconds) + " sec.");
        unsigned long currentNowTime = millis();
        Serial.println("Time to send data: " + String(currentTime - currentNowTime));

        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        size_t freeHeap = xPortGetFreeHeapSize();
        Serial.print("SendArray stack high water mark: ");
        Serial.println(highWaterMark);
        Serial.print("Free heap size SendArray: ");
        Serial.println(freeHeap);
      }
      else
      {
        Serial.println("Failed to take OneLogMutex.");
      }
      MeasurementReady = false;
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    else
    {
      Serial.println("WiFi not connected. Retrying in 5 seconds.");
      WiFi.reconnect();
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(30000 / portTICK_PERIOD_MS);
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    size_t freeHeap = xPortGetFreeHeapSize();
    Serial.print("SendArray stack high water mark: ");
    Serial.println(highWaterMark);
    Serial.print("Free heap size SendArray: ");
    Serial.println(freeHeap);
  }
  Serial.println("SendArray task has ended.");
}

void Measuring(void *parameter)
{
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Inside Measuring task.");
  Serial.println("MaxMeasurements: " + String(numMeasurements));

  TickType_t measureTime, stopTime, startTimeMeasurement, startTime;
  numMeasurements_Placeholder = 20;
  for (;;)
  {
    startTimeMeasurement = xTaskGetTickCount();
    JsonDocument doc;
    JsonArray measurementsArray = doc.to<JsonArray>();
    std::stringstream ss;
    ss.str("");

    Serial.println("Now running for " + String(numMeasurements_Placeholder) + " measurements.");
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    size_t freeHeap = xPortGetFreeHeapSize();
    size_t startingHeap = freeHeap;

    Serial.println("highWaterMark and freeHeap at beginning: ");
    Serial.print("MeasuringTask begining high water mark: " + String(highWaterMark) + " freeHeap: " + String(freeHeap));
    Serial.println("");

    for (int i = 0; i < numMeasurements_Placeholder; i++)
    {
      JsonObject measurement = measurementsArray.add<JsonObject>();
      measurement["ts"] = savedTimestamp + millis();
      JsonObject values = measurement["values"].to<JsonObject>();
      values.clear();

      if (i % dht22_tempInterval == 0)
      {
        t = dht_sensor.readTemperature();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << t;
        values["T_g"] = isnan(t) || isinf(t) ? "0.0" : ss.str();
      }

      if (i % ds18b20Interval == 0)
      {
        sensors.requestTemperatures();
        uint8_t numberOfDevices = sensors.getDeviceCount();
        for (uint8_t j = 0; j < numberOfDevices && j < 5; j++)
        {
          if (sensors.getAddress(tempDeviceAddress, j))
          {
            float tempC = sensors.getTempC(tempDeviceAddress);
            ss.str("");
            ss << std::fixed << std::setprecision(3) << tempC;
            values["T" + String(j + 1)] = isnan(tempC) || isinf(tempC) ? "0.0" : ss.str();
            *ds18b20Sensors[j] = tempC; // save tempC to corresponding sensor variable
          }
        }
      }

      if (i % acsValueFInterval == 0)
      {
        stroom = CurrentSensor_724();
        vTaskDelay(100);
        ss.str("");
        ss << std::fixed << std::setprecision(2) << stroom;
        values["A"] = isnan(stroom) || isinf(stroom) ? "0.00" : ss.str();
      }

      if (i % dht22_humInterval == 0)
      {
        humidity = dht_sensor.readHumidity();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << humidity;
        values["Hum"] = isnan(humidity) || isinf(humidity) ? "0.00" : ss.str();
      }

      if (i % flowRateInterval == 0)
      {
        ss.str("");
        ss << std::fixed << std::setprecision(2) << flowRate;
        values["Flow1"] = isnan(flowRate) || isinf(flowRate) ? "0.00" : ss.str();
      }

      if (i % h2Interval == 0)
      {
        MQ8.update();
        ppmH = MQ8.readSensor();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ppmH;
        values["H2"] = isnan(ppmH) || isinf(ppmH) ? "0.00" : ss.str();
      }

      if (i % voltInterval == 0)
      {
        Volt = readVoltage();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << Volt;
        values["V"] = isnan(Volt) || isinf(Volt) ? "0.00" : ss.str();
      }

      if (i % powerInterval == 0)
      {
        power = Volt * stroom;
        ss.str("");
        ss << std::fixed << std::setprecision(2) << power;
        values["P"] = isnan(power) || isinf(power) ? "0.00" : ss.str();
      }

      /*

            if (i % flowRate2Interval == 0)
            {
              ss.str("");
              ss << std::fixed << std::setprecision(2) << flowRate2;
              values["Flow2"] = isnan(flowRate2) || isinf(flowRate2) ? "0.00" : ss.str();
            }

            if (i % DFcoInterval == 0)
            {
              CO_Value = analogRead(Pin_CO);
              ss.str("");
              ss << std::fixed << std::setprecision(2) << CO_Value;
              // values["DFCO"] = "0.0"; //Changed because previous code was not working. It entered "�" in the JSON file.
              values["DFCO"] = isnan(CO_Value) || isinf(CO_Value) ? "0.0" : ss.str();
            }

            if (i % AGS02MAInterval == 0)
            {
              // uint32_t AGS02MA_Value = AGS.readPPB();
              // Serial.print("PPB:\t");
              // Serial.print(AGS02MA_Value);
              // Serial.print("\t");
              // Serial.print(AGS.lastStatus(), HEX);
              // Serial.print("\t");
              // Serial.print(AGS.lastError(), HEX);
              // Serial.println();

              // xSemaphoreTake(i2c_mutex, portMAX_DELAY);
              // resistance = ags.getGasResistance();
              // AGS02MA_Value = ags.getTVOC();

              // if (resistance == 0) {
              //   Serial.println(F("Failure reading resistance, I2C communications issue?"));
              // } else {
              //   float kohm = resistance / 1000.0;
              //   Serial.print(F("Gas resistance: "));
              //   Serial.print(kohm);
              //   Serial.println(" Kohms");
              // }

              // if (AGS02MA_Value == 0) {
              //   Serial.println(F("Failure reading TVOC, I2C communications issue?"));
              // } else {
              //   Serial.print(F("TVOC: "));
              //   Serial.print(AGS02MA_Value);
              //   Serial.println(" ppb");
              // }
              // xSemaphoreGive(i2c_mutex);
            }

            if (i % FlowTempinterval == 0)
            {
              temp_flow = Read_NTC(); // Read temperature
              ss.str("");
              ss << std::fixed << std::setprecision(1) << temp_flow;
              values["FT"] = isnan(temp_flow) || isinf(temp_flow) ? "0.00" : ss.str();
            }

            if (i % coInterval == 0)
            {
              MQ7.update();
              ppmCO = MQ7.readSensor();
              ss.str("");
              ss << std::fixed << std::setprecision(3) << ppmCO;
              values["CO"] = isnan(ppmCO) || isinf(ppmCO) ? "0.00" : ss.str();
            }

      */
      if (doc.isNull())
      {
        Serial.println("JSON allocation failed");
        Serial.println("Current Measurement index: " + String(currentMeasurementIndex));
        // return;
      }

      if (doc.overflowed())
      {
        Serial.println("JSON overflowed, not enough memory. At Measurement: " + String(i));
        Serial.println("doc entries: " + String(doc.size()) + " size : " + String(measureJson(doc)));
        Serial.println("Free heap MeasuringTask: " + String(xPortGetFreeHeapSize()) + " highWater mark: " + String(uxTaskGetStackHighWaterMark(NULL)));
        // Serial.printf("ESP.getFreeHeap JSON: %d, Heap fragmentation:  %d%%\n", ESP.getFreeHeap(), 100 - (ESP.getMaxAllocHeap() * 100) / ESP.getFreeHeap());
        Serial.println("Current Measurement index: " + String(currentMeasurementIndex));
        Serial.println("Breaking for loop");
        Serial.println("");
        break;
        // return;
      }

      if (i % 10 == 0)
      {
        Serial.println("Measurement: " + String(i) + " entries: " + String(doc.size()) + " size: " + String(measureJson(doc)));
        Serial.println("Free heap MeasuringTask: " + String(xPortGetFreeHeapSize()) + " highWater mark: " + String(uxTaskGetStackHighWaterMark(NULL)));
        Serial.printf("Free heap ESP.getFreeHeap JSON: %d, Heap fragmentation:  %d%%\n", ESP.getFreeHeap(), 100 - (ESP.getMaxAllocHeap() * 100) / ESP.getFreeHeap());
        Serial.println("");
      }

      currentMeasurementIndex++;
    }

    highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    freeHeap = xPortGetFreeHeapSize();
    size_t endingHeap = freeHeap;
    Serial.println("After for loop, freeHeap: " + String(freeHeap) + " highWater mark: " + String(highWaterMark));
    Serial.println(String(numMeasurements_Placeholder) + " measurements took: " + String(startingHeap - endingHeap) + " bytes");
    Serial.println("");

    stopTime = xTaskGetTickCount();
    measureTime = stopTime - startTimeMeasurement;
    int seconds = measureTime / 1000;
    int minutes = seconds / 60;
    int remainingSeconds = seconds % 60;
    Serial.println(String(numMeasurements_Placeholder) + " measurements take: " + String(minutes) + " min " + String(remainingSeconds) + " sec.");

    if (currentMeasurementIndex >= (numMeasurements - 1))
    {
      numMeasurements_Placeholder += 10;
      // printCMD();
#ifdef DEBUG_MODE_2
      Serial.println("Measurement duration: " + String(measureTime));
      Serial.println("All of the following durations are singular, to obtain the total time you need to multiply by the number of measurements");
      Serial.println("Temp duration:   " + String(endTimeDHTtemp - startDHTtempTime) + "| Humi duration: " + String(endTimeDHThum - startDHThumTime) + "| DS18B20 duration:  " + String(endTimeDS18B20 - startDS18B20Time) + "| Power duration:  " + String(endTimePower - startPowerTime));
      Serial.println("pH duration:     " + String(endTimepH - startpHTime) + "| EC duration:   " + String(endTimeEC - startECtime) + "| Flowrate duration: " + String(endTimeFlow1 - startFlowRateTime) + "| Flowrate2 duration: " + String(endTimeFlow2 - startFlowRate2Time));
      Serial.println("Stroom duration: " + String(endTimeStroom - startStroomTime) + "| Volt duration: " + String(endTimeVolt - startVoltTime) + "| MQ8 H2 duration:   " + String(endTimeH2 - startH2time) + "| MQ7 Co duration:    " + String(endTimeCO - startCOtime));
      Serial.println("FlowTemp duration: " + String(endTimeFlowTemp - startFlowTempTime));
      Serial.println();
#endif

      Serial.printf("Free heap before writing to SD card: %d\n, Heap fragmentation:  %d%%\n", ESP.getFreeHeap(), 100 - (ESP.getMaxAllocHeap() * 100) / ESP.getFreeHeap());
      vTaskDelay(50 / portTICK_PERIOD_MS);
      if (ESP.getFreeHeap() < 1000)
      {
        Serial.println("Free heap below 1000 bytes. Not writing to SD card.");
        Serial.printf("Free heap: %d, Heap fragmentation:  %d%%\n", ESP.getFreeHeap(), 100 - (ESP.getMaxAllocHeap() * 100) / ESP.getFreeHeap());
      }

      if (xSemaphoreTake(OneLogMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
      {
        if (doc != nullptr)
        {
          File file = SD.open("/One_Measurement.txt", FILE_WRITE);
          if (file)
          {
            Serial.println("doc entries: " + String(doc.size()) + " size : " + String(measureJson(doc)));

            serializeJson(doc, file);
            Serial.println("Onelog data in MeasuringTask: ");
            serializeJsonPretty(doc, Serial);
            file.close();
            Serial.println("Data written to one measurement file.");
          }
          else
          {
            Serial.println("Error opening one measurement file for writing.");
          }

          vTaskDelay(50 / portTICK_PERIOD_MS);
          Serial.printf("Free heap after serialize JSON: %d\n", ESP.getFreeHeap());
          Serial.printf("Heap fragmentation after serialize JSON: %d%%\n", 100 - (ESP.getMaxAllocHeap() * 100) / ESP.getFreeHeap());

          vTaskDelay(50 / portTICK_PERIOD_MS);
          xSemaphoreGive(OneLogMutex);
          MeasurementReady = true;

          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
      }
      else
      {
        Serial.println("Measuring Task: Could not take OneLogMutex.");
      }

      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE)
      {
        if (doc != nullptr)
        {
          // logMeasurement((&doc)->as<String>().c_str());
          File dataFile = SD.open("/log.txt", FILE_APPEND);
          if (dataFile)
          {
            dataFile.println("");
            String datetime_gsm = "21-10-2024 11:23:45";
            dataFile.println(datetime_gsm);
            serializeJson(doc, dataFile);
            dataFile.println("");
            dataFile.close();
            Serial.println("Data written to log file.");
          }
          else
          {
            Serial.println("Error opening log file for writing.");
          }
          xSemaphoreGive(fileMutex);
        }
      }
      else
      {
        Serial.println("Measuring Task: logMeasurement could not take fileMutex");
      }
    }

    currentMeasurementIndex = 0;
  }
  Serial.println("Measuring task has ended.");
}

void Counting(void *parameter)
{
  vTaskDelay(50 / portTICK_PERIOD_MS);
  Serial.println("Counting task has started.");
  for (;;)
  {
    int16_t count1 = 0, count2 = 0, count3 = 0;
    static int16_t last_count1 = 0, last_count2 = 0, last_count3 = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = millis();

    pcnt_get_counter_value(PCNT_UNIT1, &count1);
    pcnt_get_counter_value(PCNT_UNIT2, &count2);
    uint32_t elapsed_time = current_time - last_time; // Time in milliseconds

    // Calculate frequency for PCNT_UNIT1
    if (elapsed_time > 0)
    {
      int16_t pulses1 = count1 - last_count1;
      frequency1 = (float)pulses1 / (elapsed_time / 1000.0); // Frequency in Hz
      flowRate = frequency1 / flowSensorCalibration;
      // Update last count for unit 1
      last_count1 = count1;
    }

    // Update last time
    last_time = current_time;

    vTaskDelay(1100 / portTICK_PERIOD_MS); // 1000
  }
  Serial.println("Counting task has ended.");
}

void DisplayMeasurements(void *parameter)
{
  Serial.println("Inside Display Measurements task.");
  Serial.println("SavedTimestamp in DisplayMeasurements: " + String(savedTimestamp));

  char TX_RX[32];
  char definiedGSMTypeDis[32];
  char Pin_MQ8Dis[32];
  char DHT_SENSOR_PINDis[32];
  char DS18B20_PINDis[32];
  char flowSensorPinDis[32];
  char EC_PINDis[32];
  char CurrentPinDis[32];
  char PH_PINDis[32];
  char voltPinDis[32];
  // char CO_PinDis[32];
  // char Pin_MQ7Dis[32];
  // char flowSensor2PinDis[32];
  // char NTC_PINDis[32];

  for (;;)
  {
    time_t timestamp = (savedTimestamp + millis()) / 1000; // savedTimestamp + millis() / 1000;  // Convert to seconds
    struct tm *timeinfo;
    char buffer[25];
    timeinfo = gmtime(&timestamp); // Convert timestamp to timeinfo struct
    // strftime(buffer, sizeof(buffer), "%d %b  %H:%M:%S", timeinfo);
    strftime(buffer, sizeof(buffer), "%d %b  %H:%M", timeinfo); // Removed seconds because it's unnecessary and won't tik properly

    String timeDis = String(buffer);
    String flowDis = "Flow: " + String(flowRate) + " L/min";
    String humidityDis = "Hum_: " + String(humidity) + " %";
    String tempDis = "Temp: " + String(t) + " °C";
    String h2Dis = "H2__: " + String(ppmH) + " ppm";
    String DS18B20_A_B = "A: " + String(DS18B20_1) + " B:" + String(DS18B20_2);
    String DS18B20_C_D = "C: " + String(DS18B20_3) + " D:" + String(DS18B20_4);
    String DS18B20_E_F = "E: " + String(DS18B20_5) + " F:";
    String DS18B20_1_Dis = "DS_1: " + String(DS18B20_1) + " °C";
    String DS18B20_2_Dis = "DS_2: " + String(DS18B20_2) + " °C";
    String DS18B20_3_Dis = "DS_3: " + String(DS18B20_3) + " °C";
    String DS18B20_4_Dis = "DS_4: " + String(DS18B20_4) + " °C";
    String DS18B20_5_Dis = "DS_5: " + String(DS18B20_5) + " °C";
    String Current_Dis = "Amp_: " + String(stroom) + " A";
    String pH_Dis = "pH__: " + String(phvalue) + "";
    String VoltDis = "Volt: " + String(Volt) + " V";
    String ecDis = "EC__: " + String(ecValue) + " ms/cm";
    String powerDis = "Power: " + String(power) + " W";
    // String temp_flowDis = "Tflow: " + String(temp_flow) + " °C";
    // String CODFDis = "CODF: " + String(CO_Value) + " ppm";
    // String AGS02MADis = "VTOC: " + String(AGS02MA_Value) + " ppb";
    // String flowDis2 = "Flow2: " + String(flowRate2) + " L/min";
    // String coDis = "CO__: " + String(ppmCO) + " ppm";
    String amountOfMeasurementsDis = "Amount: " + String(numMeasurements_Placeholder);
    String BluetoothConnectedDis = "BT: " + String(BtConnected);
    if (deviceConnected)
    {
      BtConnected = "Connected";
    }
    else if (!deviceConnected)
    {
      BtConnected = "Disconnected";
    }

    String ConnectedDis = "WiFi: " + String(Connected);
    if (WiFi.status() != WL_CONNECTED)
    {
      Connected = "Disconnected";
    }
    else if (WiFi.status() == WL_CONNECTED)
    {
      Connected = "Connected";
    }

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    bigOled.firstPage();
    do
    {
      bigOled.setFont(u8g2_font_04b_03b_tr);
      bigOled.setDisplayRotation(U8G2_R1);
      bigOled.drawStr(0, 8, timeDis.c_str());
      bigOled.drawStr(0, 16, VoltDis.c_str());
      bigOled.drawStr(0, 24, Current_Dis.c_str());
      bigOled.drawStr(0, 32, powerDis.c_str());
      bigOled.drawStr(0, 40, pH_Dis.c_str());
      bigOled.drawStr(0, 48, ecDis.c_str());
      bigOled.drawStr(0, 56, humidityDis.c_str());
      bigOled.drawStr(0, 64, tempDis.c_str());
      bigOled.drawStr(0, 72, DS18B20_A_B.c_str());
      bigOled.drawStr(0, 80, DS18B20_C_D.c_str());
      // bigOled.drawStr(0, 88, DS18B20_E_F.c_str());
      bigOled.drawStr(0, 88, h2Dis.c_str());
      bigOled.drawStr(0, 104, amountOfMeasurementsDis.c_str());
      bigOled.drawStr(0, 112, BluetoothConnectedDis.c_str());
      if (wifiMode)
      {
        bigOled.drawStr(0, 120, ConnectedDis.c_str());
      }
      else if (gsmMode)
      {
        String gsmStatus = "GSM: ";
        // Check if registered and connected
        gsmSerial.println("AT+CREG?");
        if (response.indexOf("+CREG: 2,1") != -1 || response.indexOf("+CREG: 2,5") != -1)
        {
          gsmStatus += "Connected";
        }
        else
        {
          gsmStatus += "Disconnected";
        }
        bigOled.drawStr(0, 120, gsmStatus.c_str());
      }

      // bigOled.drawStr(0, 104, coDis.c_str());
      // bigOled.drawStr(0, 112, AGS02MADis.c_str());
      // bigOled.drawStr(0, 112, flowDis.c_str());
      // bigOled.drawStr(0, 120, CODFDis.c_str());
      // bigOled.drawStr(0, 120, flowDis2.c_str());
      // bigOled.drawStr(0, 128, temp_flowDis.c_str());
    } while (bigOled.nextPage());
    xSemaphoreGive(i2c_mutex);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  Serial.println("Display task has ended.");
}

void BluetoothListen(void *parameter)
{
  Serial.println("Inside BLE task.");
  String lastCommand = "";
  const int bufferSize = 8192;
  uint8_t *buffer = (uint8_t *)malloc(bufferSize);

  if (!buffer)
  {
    Serial.println("Failed to allocate buffer");
  }

  for (;;)
  {
    if (deviceConnected)
    {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0)
      {
        String command = String(rxValue.c_str());
        command.trim();

        if (command != lastCommand)
        {
          Serial.println("Received command: " + command);
          lastCommand = command;

          if (command == "config" || command == "log" || command == "one_log")
          {
            unsigned long startTime = millis();

            if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
            {
              const char *filename = "";
              if (command == "config")
                filename = "/config.txt";
              else if (command == "log")
                filename = "/log.txt";
              else if (command == "one_log")
                filename = "/One_Measurement.txt";

              File file = SD.open(filename, FILE_READ);
              size_t fileSize = file.size();
              Serial.println(String(filename) + " size: " + String(fileSize) + " bytes");
              if (file)
              {
                size_t bytesRead;
                while ((bytesRead = file.read(buffer, bufferSize)) > 0 && deviceConnected)
                {
                  for (size_t i = 0; i < bytesRead; i += 512) //100
                  {
                    if (!deviceConnected) {
                        Serial.println("Client disconnected, stopping transfer");
                        file.close();
                        xSemaphoreGive(fileMutex);
                        break;
                    }

                    const size_t chunkSize = min(512, (int)(bytesRead - i)); // 100 works fine
                    pCharacteristic->setValue(&buffer[i], chunkSize);
                    pCharacteristic->notify();
                    vTaskDelay(1 / portTICK_PERIOD_MS); // was 20
                  }
                }
                file.close();
                Serial.println("File sent successfully: " + String(filename));
              }
              xSemaphoreGive(fileMutex);
            }

            unsigned long transferTime = millis() - startTime;
            int seconds = transferTime / 1000;
            int minutes = seconds / 60;
            int remainingSeconds = seconds % 60;
            Serial.println("Time taken to send file: " + String(minutes) + " min " + String(remainingSeconds) + " sec.");
          }
          else if (command == "api_jrb")
          {
            strcpy(httpapi, "http://jrbubuntu.ddns.net:5000/api/telemetry");
            Serial.println("API endpoint changed to JRB");
          }
          else if (command == "api_mont")
          {
            strcpy(httpapi, "http://145.131.6.212/api/v1/HR/gl3soo07qchjimbsdwln/telemetry");
            Serial.println("API endpoint changed to Montaigne");
          }
        }
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  free(buffer);
}

/*
void BluetoothListen(void *parameter) {
    Serial.println("Inside BLE task.");
    String lastCommand = "";

    for (;;) {
        if (deviceConnected) {
            std::string rxValue = pCharacteristic->getValue();
            if (rxValue.length() > 0) {
                String command = String(rxValue.c_str());
                command.trim();

                if (command != lastCommand) {
                    Serial.println("Received command: " + command);
                    lastCommand = command;

                    if (command == "config" || command == "log" || command == "one_log") {
                        if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) {
                            const char* filename = "";
                            if (command == "config") filename = "/config.txt";
                            else if (command == "log") filename = "/log.txt";
                            else if (command == "one_log") filename = "/One_Measurement.txt";

                            File file = SD.open(filename, FILE_READ);
                            if (file) {
                                const int bufferSize = 2048; //512 works fine but crashes on "log"
                                uint8_t buffer[bufferSize];
                                size_t bytesRead;

                                while ((bytesRead = file.read(buffer, bufferSize)) > 0) {
                                    for (size_t i = 0; i < bytesRead; i += 20) {
                                        size_t chunkSize = min(20, (int)(bytesRead - i));
                                        pCharacteristic->setValue(&buffer[i], chunkSize);
                                        pCharacteristic->notify();
                                        delay(20);
                                    }
                                }
                                file.close();
                                Serial.println("File sent successfully: " + String(filename));
                            }
                            xSemaphoreGive(fileMutex);
                        }
                    }
                    else if (command == "api_jrb") {
                        strcpy(httpapi, "http://jrbubuntu.ddns.net:5000/api/telemetry");
                        Serial.println("API endpoint changed to JRB");
                    }
                    else if (command == "api_mont") {
                        strcpy(httpapi, "http://145.131.6.212/api/v1/HR/gl3soo07qchjimbsdwln/telemetry");
                        Serial.println("API endpoint changed to Montaigne");
                    }
                }
            }
        }
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
*/

void setup()
{
  gsmMode = false;
  wifiMode = true;
  if (gsmMode == false)
  {
    wifiMode = true;
  }
  else if (wifiMode == false)
  {
    gsmMode = true;
  }
  Serial.begin(115200);
  Serial.println("Free heap at start of setup: " + String(ESP.getFreeHeap()));
  if (setCpuFrequencyMhz(240))
  {
    Serial.println("CPU frequency set to: 240 MHz");
  }
  else
  {
    Serial.println("Failed to set CPU frequency.");
  }

  init_displays();
  // Bluetooth
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  initBLE();

  vTaskDelay(500 / portTICK_PERIOD_MS);
  bigOled.firstPage();
  do
  {
    bigOled.setFont(u8g2_font_tinytim_tr); // u8g2_font_ncenB08_tr
    bigOled.drawStr(0, 20, "Hogeschool");
    bigOled.drawStr(0, 40, "Rotterdam");
    bigOled.drawStr(0, 60, "Elektrotechniek.");
    bigOled.drawStr(0, 70, "#Jeej");
    if (gsmMode)
    {
      bigOled.drawStr(0, 80, "GSM Mode");
      bigOled.drawStr(0, 90, "Connecting");
      bigOled.drawStr(0, 90, "to network...");
    }
    else if (wifiMode)
    {
      bigOled.drawStr(0, 80, "WiFi Mode");
      bigOled.drawStr(0, 90, "Connecting");
      bigOled.drawStr(0, 100, "to WiFi...");
    }
  } while (bigOled.nextPage());
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  if (wifiMode)
  {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    // Disable WiFi events
    // WiFi.disconnect(true);
    // Disable powersafe
    WiFi.setSleep(true);
    // Connect to WiFi
    Serial.println("WiFi mode selected, setting up WiFi.");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nConnected to WiFi");
      Serial.println(WiFi.localIP());
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      getTime_WiFi();
      savedTimestamp = getSavedTimestamp_WiFi();
    }
    else
    {
      Serial.println("\nWiFi connection failed");
      wifiMode = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  else if (gsmMode)
  {
    Serial.println("GSM mode selected, setting up GSM.");
    gsmSerial.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN, false); // 38400 Initialize gsmSerial with appropriate RX/TX pins
    vTaskDelay(5000 / portTICK_PERIOD_MS);                              // Give some time for the serial communication to establish
    gsmSerial.println("AT");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+IPR=115200");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+CNMP=2");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+CREG=2");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    initialize_gsm();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+CREG?");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+CTZU=1");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+CTZR=1");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    nvs_flash_init();
    // getTime();
    getDateTime_SIM7600();
    // savedTimestamp = getSavedTimestamp_GSM();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  else
  {
    Serial.println("No mode selected.");
  }

  SPI.begin(); // SCK, MISO, MOSI, CS
  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("SD Card initialization failed!");
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      Serial.println("No SD card attached");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    initSD();
    // return;
  }
  else
  {
    Serial.println("SD Card initialized successfully.");

    uint8_t cardType = SD.cardType();
    Serial.println("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
      Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
      Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
      Serial.println("SDHC");
    }
    else
    {
      Serial.println("UNKNOWN");
    }
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // savedTimestamp = 1729408564;
  Serial.println("Saved timestamp in setup: " + String(savedTimestamp));
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // AGS02MA_Init();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // mq7_init(MQ7);
  // vTaskDelay(10 / portTICK_PERIOD_MS);
  mq8_init(MQ8);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  dht_sensor.begin();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  printDS18B20Address();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  interrupts();
  analogReadResolution(12); // ADC_ATTENDB_MAX
  // if (adcAttachPin(NTC_PIN) != true)
  // {
  //   Serial.println("Failed to attach ADC to NTC_PIN, current GPIO: " + String(NTC_PIN));
  // }
  /* Flow sensor */
  pcnt_example_init(PCNT_UNIT1, PCNT_INPUT_SIG_IO1);

  fileMutex = xSemaphoreCreateMutex();
  if (fileMutex == NULL)
  {
    Serial.println("Failed to create file mutex."); // Handle the error appropriately
  }
  OneLogMutex = xSemaphoreCreateMutex();
  if (OneLogMutex == NULL)
  {
    Serial.println("Failed to create OneLogMutex."); // Handle the error appropriately
  }
  i2c_mutex = xSemaphoreCreateMutex();
  if (i2c_mutex == NULL)
  {
    Serial.println("Failed to create i2c_mutex"); // Handle the error appropriately
  }

  xTaskCreatePinnedToCore(BluetoothListen, "Listen to Bluetooth", 10240, NULL, 3, &Task_Bluetooth, 1); //4096 worked 6144 worked as well. As well did 10240
  xTaskCreatePinnedToCore(DisplayMeasurements, "Display Measurements", 4096, NULL, 0, &Task_Display, 0);
  xTaskCreatePinnedToCore(Counting, "Count pulses", 1024, NULL, 2, &Task_Counting, 1); // 1024
  xTaskCreatePinnedToCore(Measuring, "Measuring", 8192, NULL, 3, &Task_Measuring, 1);  // 6144 maar crashed wanneer JSON te groot wordt
  vTaskDelay(500 / portTICK_PERIOD_MS);
  if (wifiMode)
  {
    xTaskCreatePinnedToCore(sendArray_WiFi, "Send Array", 4096, NULL, 3, &Task_sendArrayWifi, 0); // 6144
  }
  else if (gsmMode)
  {
    // xTaskCreatePinnedToCore(sendArray_GSM, "Send Array", 4096, NULL, 3, &Task_sendArrayGSM, 0); // 6144
  }
  size_t free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  Serial.println("Free heap size: " + String(free_size) + ", largest free block: " + String(largest_free_block));
  size_t free_size32 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
  size_t largest_free_block32 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
  Serial.println("Free 32 bit heap size: " + String(free_size32) + ", largest 32 free block: " + String(largest_free_block32));
  size_t heap = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

  vTaskDelay(500 / portTICK_PERIOD_MS);
  Serial.println("Free heap at end of setup: " + String(ESP.getFreeHeap()));
}

void loop()
{
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
