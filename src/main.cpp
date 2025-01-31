#include "config.h"
struct ConfigInterval configInterval;
struct ConfigPin configPin;
uint8_t ShowJSON = 0;
uint8_t ShowDebug = 0;
uint8_t OledScherm = 0;
time_t timestamp;
struct tm *timeinfo;
char TimeBuffer[25];
char TimeBufferDis[25];
char TimeBufferFinal[32];
uint64_t timestamp_ms = 0;

SemaphoreHandle_t OneLogMutex, fileMutex = NULL; // Handler for the One_Measurement.txt and log.txt file
bool MeasurementReady = false;
char DateTimeBuffer[50];
/*      GSM Module Setup     */
HardwareSerial gsmSerial(1); // Use UART1
bool gsmConnected;

/*      Bluetooth                       */
String message = "";
String BtConnected;

const char *ntpServer = "pool.ntp.org"; // NTP server to request epoch time
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
String Connected = "";

/*      SD card                         */
// SPIClass spi = SPIClass(HSPI);                   // VSPI
const uint8_t SD_CS_PIN = 46;

/*      MQ-8 H2 sensor                   */
// MQUnifiedsensor MQ8("ESP32", 5, 12, configPin.Pin_MQ8, "MQ-8"); //configPin.Pin_MQ8 not working
MQUnifiedsensor MQ8("ESP32", 5, 12, 1, "MQ-8"); 

/*      sw-420 Vibration sensor           */
String SW_420_State;

/*      PH sensor              */
ESP_PH ph;

/*      DHT22 - Temperature and Humidity */
#define DHT_SENSOR_TYPE DHT22
// DHT dht_sensor(configPin.DHT_SENSOR_PIN, DHT_SENSOR_TYPE); //configPin.DHT_SENSOR_PIN not working
DHT dht_sensor(6, DHT_SENSOR_TYPE); 

/*      Setup Flowsensor                */
uint8_t PCNT_INPUT_SIG_IO1 = configPin.flowSensorPin; // Pulse Input GPIO for PCNT_UNIT_0
#define PCNT_UNIT1 PCNT_UNIT_0
volatile float flowRate = 0.00;
float frequency1 = 0.0;

float t, phvalue, stroom, Volt, DS18B20_1, DS18B20_2, DS18B20_3, DS18B20_4, DS18B20_5, humidity, ecValue, ppmH, power;
float *ds18b20Sensors[] = {&DS18B20_1, &DS18B20_2, &DS18B20_3, &DS18B20_4, &DS18B20_5};

int currentMeasurementIndex = 0;
int numMeasurements_Placeholder = 0;
uint8_t numMeasurements;

TaskHandle_t Task_Bluetooth, Task_Counting, Task_Display, Task_Measuring, Task_sendArrayWifi, Task_sendArrayGSM = NULL;

void sendArray_GSM(void *parameter)
{
  // Initialize watchdog timer for this task
  esp_task_wdt_init(30, true); // 30 second timeout
  esp_task_wdt_add(NULL);      // Add current task to WDT watch
  vTaskDelay(200 / portTICK_PERIOD_MS);
  Serial.println("Now running sendArray GSM task.");
  unsigned long previousTime = 0;

  for (;;)
  {
    esp_task_wdt_reset(); // Reset watchdog timer
    while (MeasurementReady)
    {
      if (xSemaphoreTake(OneLogMutex, portMAX_DELAY) == pdTRUE)
      {
        GSMOutputToOLED = 0; // Don't print gsm output to display
        checkDataConnection();
        File OneLog = SD.open("/One_Measurement.txt", FILE_READ);
        if (!OneLog)
        {
          Serial.println("Error opening OneLog file");
          xSemaphoreGive(OneLogMutex);
          return;
        }

        unsigned long currentTime = millis();
        unsigned long timeBetweenUsage = currentTime - previousTime;
        previousTime = currentTime;

        // Reset WDT before HTTP operations
        esp_task_wdt_reset();

        // SIM7600 HTTP POST sequence
        gsmSerial.println("AT+HTTPTERM"); // Terminate any existing HTTP sessions
        readGsmResponse();

        gsmSerial.println("AT+HTTPINIT"); // Initialize HTTP service
        readGsmResponse();
        esp_task_wdt_reset();
        gsmSerial.println("AT+HTTPPARA=\"URL\",\"" + String(configuration.httpapi) + "\"");
        readGsmResponse();

        gsmSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
        readGsmResponse();
        esp_task_wdt_reset();
        // Set data length and timeout
        String httpDataCommand = "AT+HTTPDATA=" + String(OneLog.size()) + ",30000"; // Waits 30 seconds, maybe change to max 65535
        gsmSerial.println(httpDataCommand);
        readGsmResponse();
        // Wait for "DOWNLOAD" prompt
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Send data in chunks
        const size_t bufferSize = 1024;
        char buffer[bufferSize];
        size_t bytesRead;

        while ((bytesRead = OneLog.readBytes(buffer, bufferSize)) > 0)
        {
          gsmSerial.write(buffer, bytesRead);
          vTaskDelay(10 / portTICK_PERIOD_MS); // Add small delay between chunks
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        readGsmResponse();

        // Execute POST request
        gsmSerial.println("AT+HTTPACTION=1");
        String response = readGsmResponse3();

        // Check response
        if (response.indexOf("+HTTPACTION: 1,200") != -1)
        {
          Serial.println("POST successful");
          // Read response data
          gsmSerial.println("AT+HTTPREAD");
          readGsmResponse();
        }
        else
        {
          Serial.println("POST failed with response: " + response);
        }

        // Cleanup
        gsmSerial.println("AT+HTTPTERM");
        readGsmResponse();

        OneLog.close();
        xSemaphoreGive(OneLogMutex);

        int seconds = timeBetweenUsage / 1000;
        int minutes = seconds / 60;
        int remainingSeconds = seconds % 60;
        Serial.println("Time between usage: " + String(minutes) + " min " + String(remainingSeconds) + " sec.");
        GSMOutputToOLED = 0; // Print gsm output to display
      }
      else
      {
        Serial.println("Failed to take OneLogMutex.");
      }
      esp_task_wdt_reset();
      MeasurementReady = false;
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

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
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      unsigned long currentTime = millis();
      unsigned long timeBetweenUsage = currentTime - previousTime;
      previousTime = currentTime;

      WiFiClient client;
      HTTPClient http;
      http.begin(client, configuration.httpapi);
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

    // vTaskDelay(30000 / portTICK_PERIOD_MS); //Not sure why this is here
    if (ShowDebug == true)
    {
      UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
      size_t freeHeap = xPortGetFreeHeapSize();
      Serial.print("SendArray stack high water mark: ");
      Serial.println(highWaterMark);
      Serial.print("Free heap size SendArray: ");
      Serial.println(freeHeap);
    }
  }
  Serial.println("SendArray task has ended.");
}

void Measuring(void *parameter)
{
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Inside Measuring task.");
  Serial.println("MaxMeasurements: " + String(numMeasurements));

  TickType_t measureTime, stopTime, startTimeMeasurement, startTime;
  numMeasurements_Placeholder = 100;
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
      measurement["ts"] = timestamp_ms + millis();
      JsonObject values = measurement["values"].to<JsonObject>();
      values.clear();

      if (i % configInterval.dht22_tempInterval == 0)
      {
        t = dht_sensor.readTemperature();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << t;
        values["T_g"] = isnan(t) || isinf(t) ? "0.0" : ss.str();
      }

      if (i % configInterval.ds18b20Interval == 0)
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

      if (i % configInterval.acsInterval == 0)
      {
        stroom = CurrentSensor_724();
        vTaskDelay(100);
        ss.str("");
        ss << std::fixed << std::setprecision(2) << stroom;
        values["A"] = isnan(stroom) || isinf(stroom) ? "0.00" : ss.str();
      }

      if (i % configInterval.dht22_humInterval == 0)
      {
        humidity = dht_sensor.readHumidity();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << humidity;
        values["Hum"] = isnan(humidity) || isinf(humidity) ? "0.00" : ss.str();
      }

      if (i % configInterval.flowRateInterval == 0)
      {
        ss.str("");
        ss << std::fixed << std::setprecision(2) << flowRate;
        values["Flow1"] = isnan(flowRate) || isinf(flowRate) ? "0.00" : ss.str();
      }

      if (i % configInterval.h2Interval == 0)
      {
        MQ8.update();
        ppmH = MQ8.readSensor();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ppmH;
        values["H2"] = isnan(ppmH) || isinf(ppmH) ? "0.00" : ss.str();
      }

      if (i % configInterval.voltInterval == 0)
      {
        Volt = readVoltage();
        ss.str("");
        ss << std::fixed << std::setprecision(2) << Volt;
        values["V"] = isnan(Volt) || isinf(Volt) ? "0.00" : ss.str();
      }

      if (i % configInterval.powerInterval == 0)
      {
        power = Volt * stroom;
        ss.str("");
        ss << std::fixed << std::setprecision(2) << power;
        values["P"] = isnan(power) || isinf(power) ? "0.00" : ss.str();
      }

      if (i % configInterval.phValueInterval == 0)
      {
        phvalue = pH();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << phvalue;
        values["pH"] = isnan(phvalue) || isinf(phvalue) ? "0.00" : ss.str();
      }

      if (i % configInterval.ecValueInterval == 0)
      {
        ecValue = Cond();
        ss.str("");
        ss << std::fixed << std::setprecision(3) << ecValue;
        values["Cond"] = isnan(ecValue) || isinf(ecValue) ? "0.00" : ss.str();
      }

      if (i % configInterval.SW420Interval == 0)
      {
        int SW420_HighLow = digitalRead(configPin.SW_420_Pin);
        if (SW420_HighLow == HIGH)
        {
          SW_420_State = " moving";
        }
        else if (SW420_HighLow == LOW)
        {
          SW_420_State = " still";
        }
      }

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
            if (ShowJSON)
            {
              serializeJsonPretty(doc, Serial);
            }
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

          vTaskDelay(50 / portTICK_PERIOD_MS);
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
            dataFile.print(TimeBufferFinal);
            dataFile.println("");
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
    int16_t count1 = 0;
    static int16_t last_count1 = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = millis();

    pcnt_get_counter_value(PCNT_UNIT1, &count1);
    uint32_t elapsed_time = current_time - last_time; // Time in milliseconds

    // Calculate frequency for PCNT_UNIT1
    if (elapsed_time > 0)
    {
      int16_t pulses1 = count1 - last_count1;
      frequency1 = (float)pulses1 / (elapsed_time / 1000.0); // Frequency in Hz
      flowRate = frequency1 / configNumeric.flowSensorCalibration;
      // Update last count for unit 1
      last_count1 = count1;
    }

    // Update last time
    last_time = current_time;

    vTaskDelay(200 / portTICK_PERIOD_MS); // was 1000
  }
  Serial.println("Counting task has ended.");
}

void DisplayMeasurements(void *parameter)
{
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  Serial.println("Inside Display Measurements task.");
  Serial.println("SavedTimestamp in DisplayMeasurements: " + String(timestamp_ms));

  char TX_RX[32];
  char Pin_MQ8Dis[32];
  char DHT_SENSOR_PINDis[32];
  char DS18B20_PINDis[32];
  char flowSensorPinDis[32];
  char EC_PINDis[32];
  char CurrentPinDis[32];
  char PH_PINDis[32];
  char voltPinDis[32];
  snprintf(TX_RX, sizeof(TX_RX), "RX : %d TX : %d", configPin.GSM_RX_PIN, configPin.GSM_TX_PIN);
  snprintf(Pin_MQ8Dis, sizeof(Pin_MQ8Dis), "MQ8   : %d   %f", configPin.Pin_MQ8, configNumeric.h2Amount);
  snprintf(DHT_SENSOR_PINDis, sizeof(DHT_SENSOR_PINDis), "DHT22 : %d   %f", configPin.DHT_SENSOR_PIN, configNumeric.temperatureAmount);
  snprintf(DS18B20_PINDis, sizeof(DS18B20_PINDis), "DS18  : %d   %f", configPin.DS18B20_PIN, configNumeric.ds18b20Amount);
  snprintf(flowSensorPinDis, sizeof(flowSensorPinDis), "Flow  : %d   %f", configPin.flowSensorPin, configNumeric.flowRateAmount);
  snprintf(EC_PINDis, sizeof(EC_PINDis), "EC    : %d   %f", configPin.EC_PIN, configNumeric.ecValueAmount);
  snprintf(CurrentPinDis, sizeof(CurrentPinDis), "A     : %d   %f", configPin.CurrentPin, configNumeric.acsAmount);
  snprintf(PH_PINDis, sizeof(PH_PINDis), "PH    : %d   %f", configPin.PH_PIN, configNumeric.phValueAmount);
  snprintf(voltPinDis, sizeof(voltPinDis), "V     : %d   %f", configPin.voltPin, configNumeric.voltAmount);

  for (;;)
  {
    String timeDis = String(TimeBufferDis);
    String flowDis = "Flow: " + String(flowRate) + " L/min";
    String humidityDis = "Hum_: " + String(humidity) + " %";
    String tempDis = "Temp: " + String(t) + " °C";
    String h2Dis = "H2__: " + String(ppmH) + " ppm";
    String DS18B20_A_B = "A: " + String(DS18B20_1) + " B:" + String(DS18B20_2);
    String DS18B20_C_D = "C: " + String(DS18B20_3) + " D:" + String(DS18B20_4);
    String Current_Dis = "Amp_: " + String(stroom) + " A";
    String pH_Dis = "pH__: " + String(phvalue) + "";
    String VoltDis = "Volt: " + String(Volt) + " V";
    String ecDis = "EC__: " + String(ecValue) + " ms/cm";
    String powerDis = "Power: " + String(power) + " W";
    String SW_420DIS = "SW_420: " + SW_420_State;
    String amountOfMeasurementsDis = String(currentMeasurementIndex) + " of " + String(numMeasurements_Placeholder);
    String BluetoothConnectedDis = "BT: " + String(BtConnected);
    if (deviceConnected)
    {
      BtConnected = "Connected";
    }
    else if (!deviceConnected)
    {
      BtConnected = "Disconnected";
    }

    String WiFiStatus = "WiFi: " + String(Connected);
    String gsmStatus = "GSM: " + String(Connected);
    if (configuration.wifiMode)
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        Connected = "Disconnected";
      }
      else if (WiFi.status() == WL_CONNECTED)
      {
        Connected = "Connected";
      }
    }
    else if (configuration.gsmMode)
    {
      if (gsmConnected)
      {
        Connected = "Connected";
      }
      else if (!gsmConnected)
      {
        Connected = "Disconnected";
      }
    }

    if(OledScherm==0){
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
      bigOled.drawStr(0, 56, flowDis.c_str());
      bigOled.drawStr(0, 64, humidityDis.c_str());
      bigOled.drawStr(0, 72, tempDis.c_str());
      bigOled.drawStr(0, 80, DS18B20_A_B.c_str());
      bigOled.drawStr(0, 88, DS18B20_C_D.c_str());
      bigOled.drawStr(0, 96, h2Dis.c_str());
      // bigOled.drawStr(0, 104, SW_420DIS.c_str());
      bigOled.drawStr(0, 112, amountOfMeasurementsDis.c_str());
      bigOled.drawStr(0, 120, BluetoothConnectedDis.c_str());
      if (configuration.wifiMode)
      {
        bigOled.drawStr(0, 128, WiFiStatus.c_str());
      }
      else if (configuration.gsmMode)
      {
        bigOled.drawStr(0, 128, gsmStatus.c_str());
      }

    } while (bigOled.nextPage());
    }
    else if(OledScherm==1){
    bigOled.firstPage();
    do
    {
      bigOled.setFont(u8g2_font_04b_03b_tr);
      bigOled.setDisplayRotation(U8G2_R1);
      bigOled.drawStr(0, 8, timeDis.c_str());
      bigOled.drawStr(0, 16, "Pinout & Freq.");
      if (configuration.wifiMode)
      {
        bigOled.drawStr(0, 24, WiFiStatus.c_str());
        bigOled.drawStr(0, 32, ("SSID: " + String(configuration.ssid)).c_str());
      }
      else if (configuration.gsmMode)
      {
        bigOled.drawStr(0, 24, gsmStatus.c_str());
        bigOled.drawStr(0, 32, TX_RX);
      }
      bigOled.drawStr(0, 40, voltPinDis);
      bigOled.drawStr(0, 48, CurrentPinDis);
      bigOled.drawStr(0, 50, PH_PINDis);
      bigOled.drawStr(0, 58, EC_PINDis);
      bigOled.drawStr(0, 64, flowSensorPinDis);
      bigOled.drawStr(0, 72, DHT_SENSOR_PINDis);
      bigOled.drawStr(0, 80, DS18B20_PINDis);
      bigOled.drawStr(0, 88, Pin_MQ8Dis);
      } while (bigOled.nextPage());
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("Display task has ended.");
}

void BluetoothListen(void *parameter)
{
  Serial.println("Inside BLE task.");
  // Initialize watchdog timer for this task
  esp_task_wdt_init(30, true); // 30 second timeout
  esp_task_wdt_add(NULL);      // Add current task to WDT watch
  uint8_t TimeToReset=0;
  String lastCommand = "";
  String receivedData = "";
  bool receivingFile = false;
  String currentFileName = "";
  const int bufferSize = 8192;
  uint8_t *buffer = (uint8_t *)malloc(bufferSize);

  if (!buffer)
  {
    Serial.println("Failed to allocate buffer");
  }

  for (;;)
  {
    esp_task_wdt_reset(); // Reset the WDT timer

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
          unsigned long startTime = millis();

          if (command.startsWith("SIZE:"))
          {
            if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
            {
              String requestedFile = command.substring(5); // Get filename after "SIZE:"
              String filename = "/" + requestedFile;       // Add leading slash
              String response;
              File file = SD.open(filename, FILE_READ);
              if (file)
              {
                size_t fileSize = file.size();
                String response;

                if (fileSize < 1024)
                {
                  response = "Size of " + filename + ": " + String(fileSize) + " bytes\n";
                }
                else if (fileSize < 1024 * 1024)
                {
                  float sizeKB = fileSize / 1024.0;
                  response = "Size of " + filename + ": " + String(sizeKB, 2) + " KB\n";
                }
                else if (fileSize < 1024 * 1024 * 1024)
                {
                  float sizeMB = fileSize / (1024.0 * 1024.0);
                  response = "Size of " + filename + ": " + String(sizeMB, 2) + " MB\n";
                }
                else
                {
                  float sizeGB = fileSize / (1024.0 * 1024.0 * 1024.0);
                  response = "Size of " + filename + ": " + String(sizeGB, 2) + " GB";
                }

                Serial.println(response);
                pCharacteristic->setValue(response.c_str());
                pCharacteristic->notify();
                file.close();
              }
              else
              {
                String response = "File " + filename + " not found.\n";
                Serial.println(response);
                pCharacteristic->setValue(response.c_str());
                pCharacteristic->notify();
              }
              xSemaphoreGive(fileMutex);
            }
          }

          else if (command.equalsIgnoreCase("START_FILE") && command.startsWith("START_FILE:"))
          {
            String filename = command.substring(11); // Get filename after "START_FILE:"
            String response = "Starting to receive file: " + filename + "\n";
            Serial.println(response);
            pCharacteristic->setValue(response.c_str());
            pCharacteristic->notify();
            // Check if config.txt exists
            if (filename == "config.txt")
            {
              File configFileCheck = SD.open(filename, FILE_READ);
              if (configFileCheck)
              {
                Serial.println("Found existing config.txt");
                // Rename the old file to configOld.txt
                if (!SD.rename("/config.txt", "/configOld.txt"))
                {
                  Serial.println("Failed to rename config.txt to configOld.txt");
                }
                else
                {
                  Serial.println("Renamed config.txt to configOld.txt successfully");
                }
                configFileCheck.close();
              }
              else
              {
                Serial.println("No existing config.txt found");
              }
            }
            currentFileName = command.substring(11);
            receivingFile = true;
            receivedData = "";
            pCharacteristic->setValue("Ready to receive file\n");
            pCharacteristic->notify();
          }
          else if (command.equalsIgnoreCase("END_FILE") && receivingFile)
          {
            if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE)
            {
              File file = SD.open(currentFileName, FILE_WRITE);
              if (file)
              {
                file.print(receivedData);
                file.close();
                Serial.println("File: " + currentFileName + " saved successfully");
                pCharacteristic->setValue(("File: " + currentFileName + " saved successfully\n").c_str());
                pCharacteristic->notify();
              }
              xSemaphoreGive(fileMutex);
            }
            receivingFile = false;
            receivedData = "";
          }
          else if (receivingFile)
          {
            receivedData += command + "\n";
          }

          else if (command.equalsIgnoreCase("config") || command.equalsIgnoreCase("log") || command.equalsIgnoreCase("one_log"))
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
                for (size_t i = 0; i < bytesRead; i += 512) // 100
                {
                  if (!deviceConnected)
                  {
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
                if(TimeToReset == 15000)
                {
                  TimeToReset = 0;
                  esp_task_wdt_reset();
                }
                TimeToReset += millis();
              }
              file.close();
              Serial.println("File sent successfully: " + String(filename));
              pCharacteristic->setValue(("File sent successfully: " + String(filename) + "\n").c_str());
              pCharacteristic->notify();
            }
            else
            {
              Serial.println("Error opening file: " + String(filename));
              pCharacteristic->setValue(("Error opening file: " + String(filename) + "\n").c_str());
              pCharacteristic->notify();
            }
            xSemaphoreGive(fileMutex);
            unsigned long transferTime = millis() - startTime;
            int seconds = transferTime / 1000;
            int minutes = seconds / 60;
            int remainingSeconds = seconds % 60;
            Serial.println("Time taken to send file: " + String(minutes) + " min " + String(remainingSeconds) + " sec.");
            pCharacteristic->setValue(("Time taken to send file: " + String(minutes) + " min " + String(remainingSeconds) + " sec.\n").c_str());
            pCharacteristic->notify();
          }
          else if (command.equalsIgnoreCase("restore_config"))
          {
            Serial.println("Restoring config.txt from configOld.txt");
            pCharacteristic->setValue("Restoring config.txt from configOld.txt\n");
            pCharacteristic->notify();
            File configOldFile = SD.open("/configOld.txt", FILE_READ);
            if (configOldFile)
            {
              // Create or overwrite the main config file
              File configFile = SD.open("/config.txt", FILE_WRITE);
              if (configFile)
              {
                // Copy contents
                while (configOldFile.available())
                {
                  configFile.write(configOldFile.read());
                }
                configFile.close();
                Serial.println("Restored config.txt from configOld.txt");
                pCharacteristic->setValue("Restored config.txt from configOld.txt\n");
                pCharacteristic->notify();
              }
            }
            else
            {
              Serial.println("Old config not found.");
            }
          }
          else if (command.equalsIgnoreCase("restore_default_config"))
          {
            Serial.println("Restoring config.txt from configOld.txt");
            pCharacteristic->setValue("Restoring config.txt from configOld.txt\n");
            pCharacteristic->notify();
            File defaultConfig = SD.open("/Defaults/config.txt", FILE_READ);
            if (defaultConfig)
            {
              // Create or overwrite the main config file
              File configFile = SD.open("/config.txt", FILE_WRITE);
              if (configFile)
              {
                // Copy contents
                while (defaultConfig.available())
                {
                  configFile.write(defaultConfig.read());
                }
                configFile.close();
                pCharacteristic->setValue("Config file restored successfully\n");
              }
              defaultConfig.close();
            }
            else
            {
              Serial.println("Failed to restore config.txt from configOld.txt");
              Serial.println("Please restore config.txt manually.");
              pCharacteristic->setValue("Failed to restore config.txt from configOld.txt\n");
              pCharacteristic->notify();
              pCharacteristic->setValue("Please restore config.txt manually.\n");
              pCharacteristic->notify();
            }
          }
          else if (command.equalsIgnoreCase("Save_Config"))
          {
            // Open source file from SD card
            File sourceFile = SD.open("/config.txt", FILE_READ);
            if (sourceFile)
            {
              Serial.println("Saving config.txt to SPIFFS");
              pCharacteristic->setValue("Saving config.txt to SPIFFS\n");
              pCharacteristic->notify();
              // Open destination file in SPIFFS
              File destFile = SPIFFS.open("/config.txt", FILE_WRITE);
              if (destFile)
              {
                // Copy file contents
                while (sourceFile.available())
                {
                  destFile.write(sourceFile.read());
                }
                destFile.close();
                pCharacteristic->setValue("Config saved to SPIFFS successfully\n");
                pCharacteristic->notify();
              }
              sourceFile.close();
            }
            else
            {
              Serial.println("Failed to open config file for reading.");
              pCharacteristic->setValue("Failed to open config file for reading.\n");
              pCharacteristic->notify();
            }
          }
          else if (command == "restart")
          {
            Serial.println("Restarting...");
            pCharacteristic->setValue("Restarting...\n");
            pCharacteristic->notify();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            ESP.restart();
          }
          else if (command.equalsIgnoreCase("showjson"))
          {
            ShowJSON = !ShowJSON;
            String response = "ShowJSON = " + String(ShowJSON) + "\n";
            Serial.println(response);
            pCharacteristic->setValue(response.c_str());
            pCharacteristic->notify();
          }
          else if (command.equalsIgnoreCase("ShowDebug"))
          {
            ShowDebug = !ShowDebug;
            Serial.println("ShowDebug = " + String(ShowDebug));
            pCharacteristic->setValue(("ShowDebug = " + String(ShowDebug) + "\n").c_str());
            pCharacteristic->notify();
          }
          else if (command.equalsIgnoreCase("ShowDebug2"))
          {
            
          }
          else if (command.equalsIgnoreCase("ShowGSMOutput"))
          {
            GSMOutputToOLED = !GSMOutputToOLED;
            Serial.println("GSMOutputToOLED = " + String(GSMOutputToOLED));
            pCharacteristic->setValue(("GSMOutputToOLED = " + String(GSMOutputToOLED) + "\n").c_str());
            pCharacteristic->notify();
          }  
          else if (command.equalsIgnoreCase("Screen"))
          {
            OledScherm = !OledScherm;
            Serial.println("Oled scherm: " + String(OledScherm));
            pCharacteristic->setValue(("Oled scherm: " + String(OledScherm) + "\n").c_str());
            pCharacteristic->notify();
          }
        }
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS); // was 50
  }
  free(buffer);
}

void setup()
{
  // gsmMode = true;
  // wifiMode = false;
  // if (gsmMode == false)
  // {
  //   wifiMode = true;
  // }
  // else if (wifiMode == false)
  // {
  //   gsmMode = true;
  //   vTaskDelay(5000 / portTICK_PERIOD_MS);
  // }
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
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    // return;
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);
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
  vTaskDelay(50 / portTICK_PERIOD_MS);
  init_displays();

  // Bluetooth
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  initBLE();

  vTaskDelay(100 / portTICK_PERIOD_MS);
  bigOled.firstPage();
  do
  {
    bigOled.setFont(u8g2_font_tinytim_tr); // u8g2_font_ncenB08_tr
    bigOled.drawStr(0, 20, "Hogeschool");
    bigOled.drawStr(0, 30, "Rotterdam");
    bigOled.drawStr(0, 50, "WTB &");
    bigOled.drawStr(0, 60, "Elektro");
    if (configuration.gsmMode)
    {
      bigOled.drawStr(0, 80, "GSM Mode");
      bigOled.drawStr(0, 90, "Connecting");
      bigOled.drawStr(0, 100, "to network...");
    }
    else if (configuration.wifiMode)
    {
      bigOled.drawStr(0, 80, "WiFi Mode");
      bigOled.drawStr(0, 90, "Connecting");
      bigOled.drawStr(0, 100, "to WiFi...");
    }
  } while (bigOled.nextPage());
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  if (!read_configuration())
  {
    Serial.println("Failed to read configuration. Please add manually or via Bluetooth.");
    time_t timestamp = timestamp_ms; // Tried without the calculation
    // struct tm *timeinfo;
    char buffer[25];
    xTaskCreatePinnedToCore(BluetoothListen, "Listen to Bluetooth", 10240, NULL, 3, &Task_Bluetooth, 1); // 4096 worked 6144 worked as well. As well did 10240
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (1)
    {
      timeinfo = gmtime(&timestamp);                              // Convert timestamp to timeinfo struct
      strftime(buffer, sizeof(buffer), "%d %b  %H:%M", timeinfo); // Removed seconds because it's unnecessary and won't tik properly
      String timeDis = String(buffer);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      bigOled.firstPage();
      do
      {
        bigOled.setFont(u8g2_font_04b_03b_tr);
        bigOled.drawStr(0, 0, timeDis.c_str());
        bigOled.drawStr(0, 20, "Failed to");
        bigOled.drawStr(0, 40, "read config");
        bigOled.drawStr(0, 60, "file. Please");
        bigOled.drawStr(0, 70, "add manually");
        bigOled.drawStr(0, 80, "using Micro ");
        bigOled.drawStr(0, 90, "SD Card or");
        bigOled.drawStr(0, 100, "via Bluetooth: ");
        bigOled.drawStr(0, 110, "restore_config");
      } while (bigOled.nextPage());
    }
  }
  printVariables();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  initializeConfigInterval();
  if (configuration.wifiMode)
  {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    // Disable WiFi events
    WiFi.disconnect(true);
    // Disable powersafe
    WiFi.setSleep(true); 
    // Connect to WiFi
    Serial.println("WiFi mode selected, setting up WiFi.");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    WiFi.begin(configuration.ssid, configuration.password);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000)
    {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Serial.println("Connecting to WiFi...");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nConnected to WiFi");
      Serial.println(WiFi.localIP());
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      getTime_WiFi();
      timestamp_ms = getSavedTimestamp_WiFi();
      Serial.println("Saved timestamp in setup: " + String(timestamp_ms));
    }
    else
    {
      Serial.println("\nWiFi connection failed");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  else if (configuration.gsmMode)
  {
    Serial.println("GSM mode selected, setting up GSM.");
    gsmSerial.begin(115200, SERIAL_8N1, configPin.GSM_RX_PIN, configPin.GSM_TX_PIN, false); // 38400 Initialize gsmSerial with appropriate RX/TX pins
    vTaskDelay(5000 / portTICK_PERIOD_MS);                              // Give some time for the serial communication to establish
    gsmSerial.println("AT");
    readGsmResponse();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+IPR=115200");
    readGsmResponse();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+CREG=2");
    readGsmResponse();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    initialize_gsm();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    nvs_flash_init();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    getDateTime_GSM();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // Check if registered and connected
    IsGSMConnected();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("Saved timestamp in setup: " + String(unixTimestamp));
    timestamp_ms = unixTimestamp;
  }
  else
  {
    Serial.println("No mode selected.");
  }

    Serial.println("dht22_tempInterval: " + String(configInterval.dht22_tempInterval));
    Serial.println("dht22_humidityInterval: " + String(configInterval.dht22_humInterval));
    Serial.println("mq8_interval: " + String(configInterval.h2Interval));
    Serial.println("flowRateInterval: " + String(configInterval.flowRateInterval));
    Serial.println("ds18b20_interval: " + String(configInterval.ds18b20Interval));
    Serial.println("sw_420_interval: " + String(configInterval.SW420Interval));
    Serial.println("acsInterval: " + String(configInterval.acsInterval));
    Serial.println("voltInterval: " + String(configInterval.voltInterval));
    Serial.println("powerInterval: " + String(configInterval.powerInterval));
    Serial.println("phValueInterval: " + String(configInterval.phValueInterval));
    Serial.println("ecValueInterval: " + String(configInterval.ecValueInterval));

    Serial.println("temperatureAmount: " + String(configNumeric.temperatureAmount));
    Serial.println("humidityAmount: " + String(configNumeric.humidityAmount));
    Serial.println("h2Amount: " + String(configNumeric.h2Amount));
    Serial.println("flowRateAmount: " + String(configNumeric.flowRateAmount));
    Serial.println("ds18b20Amount: " + String(configNumeric.ds18b20Amount));
    Serial.println("SW420Amount: " + String(configNumeric.SW420Amount));
    Serial.println("acsAmount: " + String(configNumeric.acsAmount));
    Serial.println("voltAmount: " + String(configNumeric.voltAmount));
    Serial.println("powerAmount: " + String(configNumeric.powerAmount));
    Serial.println("phValueAmount: " + String(configNumeric.phValueAmount));
    Serial.println("ecValueAmount: " + String(configNumeric.ecValueAmount));

  vTaskDelay(10 / portTICK_PERIOD_MS);
  mq8_init(MQ8);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  dht_sensor.begin();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  printDS18B20Address();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  pinMode(configPin.SW_420_Pin, INPUT); // Pin for tilt sensor
  interrupts();
  analogReadResolution(12); // ADC_ATTENDB_MAX
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

  xTaskCreatePinnedToCore(BluetoothListen, "Listen to Bluetooth", 10240, NULL, 3, &Task_Bluetooth,1); // 4096 worked 6144 worked as well. As well did 10240
  xTaskCreatePinnedToCore(DisplayMeasurements, "Display Measurements", 4096, NULL, 0, &Task_Display, 0);
  xTaskCreatePinnedToCore(Counting, "Count pulses", 1024, NULL, 2, &Task_Counting, 1); 
  xTaskCreatePinnedToCore(Measuring, "Measuring", 8192, NULL, 3, &Task_Measuring, 1); 
  vTaskDelay(500 / portTICK_PERIOD_MS);
  if (configuration.wifiMode)
  {
    xTaskCreatePinnedToCore(sendArray_WiFi, "Send Array WiFi", 4096, NULL, 3, &Task_sendArrayWifi, 0); // 6144
  }
  else if (configuration.gsmMode)
  {
    xTaskCreatePinnedToCore(sendArray_GSM, "Send Array GSM", 4096, NULL, 3, &Task_sendArrayGSM, 0); // 6144
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
  time_t timestamp = (timestamp_ms + millis()) / 1000; // savedTimestamp + millis() / 1000;  // Convert to seconds
  int milliseconds = (timestamp_ms + millis()) % 1000;
  timeinfo = gmtime(&timestamp);                              // Convert timestamp to timeinfo struct
  strftime(TimeBuffer, sizeof(TimeBuffer), "%d %b %Y %H:%M:%S", timeinfo); 
  snprintf(TimeBufferFinal, sizeof(TimeBufferFinal), "%s.%03d", TimeBuffer, milliseconds); 
  strftime(TimeBufferDis, sizeof(TimeBufferDis), "%d %b  %H:%M", timeinfo); // Removed seconds because it's unnecessary and won't tik properly
  vTaskDelay(250 / portTICK_PERIOD_MS);  
}

