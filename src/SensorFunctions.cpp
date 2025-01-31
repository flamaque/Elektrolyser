#include "config.h"

/**
 * @brief Initializes PCNT (Pulse Counter) for flow sensor
 * @param unit PCNT unit to configure
 * @param pulse_gpio_num GPIO pin number for pulse input
 */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num)
{
  /* Prepare configuration for the PCNT unit */
  pcnt_config_t pcnt_config = {
      // Set PCNT input signal GPIO
      .pulse_gpio_num = pulse_gpio_num,
      // No control GPIO needed
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      // What to do on the positive / negative edge of pulse input?
      .pos_mode = PCNT_COUNT_INC, // Count up on the positive edge
      .neg_mode = PCNT_COUNT_DIS, // Ignore negative edge
      // Set the maximum and minimum limit values to watch
      .counter_h_lim = 0,
      .counter_l_lim = 0,
      .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };
  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(unit);
}

/**
 * @brief Reads the voltage
 * @return voltage
 */
float readVoltage()
{
  const int numSamples = 100;
  const float adcVoltageScale = 3.3 / 4095.0;
  float voltageSum = 0.0;
  float R1 = 1000.0;
  float R2 = 10000.0;

  for (int i = 0; i < numSamples; i++)
  {
    int adc = analogRead(configPin.voltPin);
    voltageSum += adc * adcVoltageScale;
    vTaskDelay(2 / portTICK_PERIOD_MS); // Small delay to allow for better averaging
  }

  float voltage = (voltageSum / numSamples) * (R2 / (R1 + R2));
  return voltage;
}

/*      DS18B20 sensor            */
OneWire oneWire(configPin.DS18B20_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);    // Pass our oneWire reference to Dallas Temperature.
int numberOfDevices;                    // Number of temperature devices found
DeviceAddress tempDeviceAddress;        // We'll use this variable to store a found device address
void printDS18B20Address()
{
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++)
  {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      // Print the address
      for (uint8_t j = 0; j < 8; j++)
      {
        if (tempDeviceAddress[j] < 16)
          Serial.print("0");
        Serial.print(tempDeviceAddress[j], HEX);
      }
      Serial.println();
    }
    else
    {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
      Serial.println();
    }
  }
  sensors.setResolution(11); // Set the resolution to 11 bits for 0.5°C resolution
}

/*       MQ-8 sensor            */
float RatioMQ8CleanAir = 70.0;
void mq8_init(MQUnifiedsensor &MQ8)
{
  // Hydrogen
  MQ8.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ8.setA(976.97);
  MQ8.setB(-0.688); // Configure the equation to to calculate H2 concentration
  MQ8.init();
  Serial.print("Calibrating MQ8 please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ8.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ8.calibrate(RatioMQ8CleanAir);
    Serial.print(".");
  }
  MQ8.setR0(calcR0 / 10);
  Serial.println("R0 for MQ8 calculation done!.");

  if (isinf(calcR0))
  {
    Serial.println("MQ8 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  }
  if (calcR0 == 0)
  {
    Serial.println("MQ8 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  }
  /*****************************  MQ CAlibration ********************************************/
  Serial.println("MQ8 initialized!");
  MQ8.serialDebug(true);
}

/*              Setup Currentsensor    */
float CurrentSensor_724()
{
  float current_voltage, current = 0.0;

  float R1 = 3300.0; // 1000.0;
  float R2 = 6800.0; // 2000.0;
  float RatioVolDiv = (R1 + R2) / R2;
  const int numSamples = 100;
  long adc_voltage_sum = 0;

  // Read ADC value multiple times to average
  for (int i = 0; i < numSamples; i++)
  {
    int adc = analogReadMilliVolts(configPin.CurrentPin);
    adc_voltage_sum += adc;             // 3.3
    vTaskDelay(2 / portTICK_PERIOD_MS); // Small delay to allow for better averaging
  }
  // Serial.println("ADC voltage raw: " + String(analogReadRaw(configPin.CurrentPin)));
  // Serial.println("ADC mV Quick: " + String(analogReadMilliVolts(configPin.CurrentPin)));
  //  Average the ADC voltage
  float adc_voltage = (adc_voltage_sum / numSamples); // * (3300 / 4095.0);
  // Serial.println("ADC voltage Quick: " + String(adc_voltage));
  //  Serial.println("ADC voltage: " + String(adc_voltage));

  // Calculate the sensor voltage
  current_voltage = adc_voltage * RatioVolDiv;
  // Serial.println("Current voltage Quick: " + String(current_voltage));

  // Measure this value when no current is flowing to calibrate zeroCurrentVoltage
  // float zeroCurrentVoltage = 0.48; // Use the previously measured value or measure again

  // float sensitivity = 0.066; // Change this value based on your specific ACS712 model

  float zeroCurrentVoltage = 2500; // 2500; //Or 1.58V after voltage divider
  float sensitivity = 40;          // 0.040; //ACS724 sensitivity Change this value based on your specific ACS712 model

  // Calculate the current
  current = (current_voltage - zeroCurrentVoltage) / sensitivity;
  // Serial.println("Current: " + String(current));
  // printf("R1: %f, R2: %f, sampling: %d, ADC voltage: %f\n", R1, R2, numSamples, adc_voltage);

  return current;
}

/*      Display Setup               */
U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/10, /* data=*/9);
#define U8LOG_WIDTH 12 // 25
#define U8LOG_HEIGHT 6 // 8+
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
U8G2LOG u8g2log;

void init_displays()
{
  bigOled.setI2CAddress(0x3C * 2);
  bigOled.setBusClock(400000); // 400000
  bigOled.begin();
  bigOled.clearBuffer();
  bigOled.setFont(u8g2_font_6x12_mf); // set the font for the terminal window
  bigOled.setDisplayRotation(U8G2_R1);
  u8g2log.begin(bigOled, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(2);          // set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(1);                // 0: Update screen with newline, 1: Update screen for every char
  Serial.println("Displays initialized!"); // Serial.println("Big Display height: " + bigOled.getDisplayHeight() + " Big Display Width: "  + bigOled.getDisplayHeight());
}

/*      Conductivity Sensor   */
DFRobot_ESP_EC ec;
volatile float voltage_cond, temperature_cond = 25; // variable for storing the potentiometer value
float ecValueFloat = 0;
float Cond()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) // time interval: 1s
  {
    timepoint = millis();
    voltage_cond = analogRead(configPin.EC_PIN) / 4095.0 * 3300;
    // Serial.println("voltage: " + String(voltage_cond, 4));
    // temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    // Serial.println("voltage_cond: " String(temperature_cond, 1));
    // Serial.println("^C");

    ecValueFloat = ec.readEC(voltage_cond, temperature_cond); // convert voltage to EC with temperature compensation
                                                              // Serial.println("EC: " + String(measurement.ecValue, 4) + " ms/cm");
  }
  ec.calibration(voltage_cond, temperature_cond); // calibration process by Serail CMD

  // Serial.print(ecValue,2);  Serial.println("ms/cm");
  return ecValueFloat;
}

/*      pH Sensor             */
#define ESPADC 4096.0   // the esp Analog Digital Conversion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
float voltage_pH, phValue;
float temperature_pH = 20.0; // Fixed temperature value kan vervangen worden wanneer temp sensor gebruikt wordt

// wanneer je inf melding krijgt moet je caliberen lees hieronder om de code te laten werken is er een calibratie proces nodig type enterph
// doe ph sensor in 4ph en type calph
// doe ph sensor in 7ph en type calph
// type endcalph om compleet te maken.

float pH()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) // time interval: 1s
  {
    timepoint = millis();
    voltage_pH = analogRead(configPin.PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    // Serial.print("voltage_pH:");
    // Serial.println(voltage_pH, 4);

    phValue = ph.readPH(voltage_pH, temperature_pH); // convert voltage to pH with fixed temperature
                                                     // Serial.print("pH:");
    // Serial.println(phValue, 4);
  }
  ph.calibration(voltage_pH, temperature_pH); // calibration process by Serial CMD
  return phValue;
}

struct Configuration configuration;
struct ConfigNumeric configNumeric;
// Removed the commented-out version of processLine to avoid ambiguity


/* DHT22 */
uint8_t data[5];

/*!
 *  @brief  Read Humidity
 *  @param  force
 *					force read mode
 *	@return float value - humidity in percent
 */
float readHumidity() {
  float f = NAN;
      f = ((word)data[0]) << 8 | data[1];
      f *= 0.1;
  return f;
}

/*!
 *  @brief  Read temperature
 *  @param  S
 *          Scale. Boolean value:
 *					- true = Fahrenheit
 *					- false = Celcius
 *  @param  force
 *          true if in force mode
 *	@return Temperature value in selected scale
 */
float readTemperature(bool S, bool force) {
  float f = NAN;

      f = ((word)(data[2] & 0x7F)) << 8 | data[3];
      f *= 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }      
  return f;
}


void processLine(const String &line)
{
  if (line.isEmpty())
  {
    return;
  }
  if (line.startsWith("###"))
  {
    return;
  }
  size_t colonPos = line.indexOf(':');
  if (colonPos == -1)
  {
    Serial.println("Invalid line: no colon found");
    Serial.println(line);
    return;
  }

  String key = line.substring(0, colonPos);
  String valueStr = line.substring(colonPos + 1);

  key.trim();
  valueStr.trim();

  if (key.equalsIgnoreCase("ssid"))
  {
    valueStr.remove(0, 1);
    valueStr.remove(valueStr.length() - 1);
    configuration.ssid = valueStr;
  }
  else if (key.equalsIgnoreCase("password"))
  {
    valueStr.remove(0, 1);
    valueStr.remove(valueStr.length() - 1);
    configuration.password = valueStr;
  }
  else if (key.equalsIgnoreCase("apn"))
  {
    valueStr.remove(0, 1);
    valueStr.remove(valueStr.length() - 1);
    configuration.apn = valueStr;
  }
  else if (key.equalsIgnoreCase("apn_User"))
  {
    valueStr.remove(0, 1);
    valueStr.remove(valueStr.length() - 1);
    configuration.apn_User = valueStr;
  }
  else if (key.equalsIgnoreCase("apn_Pass"))
  {
    valueStr.remove(0, 1);
    valueStr.remove(valueStr.length() - 1);
    configuration.apn_Pass = valueStr;
  }
  else if (key.equalsIgnoreCase("httpapi"))
  {
    valueStr.remove(0, 1);
    valueStr.remove(valueStr.length() - 1);
    configuration.httpapi = valueStr;
  }
  else if (key.equalsIgnoreCase("ConnectionMode"))
  {
    if (valueStr.equalsIgnoreCase("GSM"))
    {
      configuration.connectionMode = "GSM";
      configuration.gsmMode = true;
      configuration.wifiMode = false;
    }
    else if (valueStr.equalsIgnoreCase("WiFi"))
    {
      configuration.connectionMode = "WiFi";
      configuration.wifiMode = true;
      configuration.gsmMode = false;
    }
    else
    {
      Serial.println("Invalid connection mode: " + valueStr);
      return;
    }
  }
  else if (key.equalsIgnoreCase("mobileNumber"))
  {
    if (valueStr.length() >= 12 && valueStr.startsWith("+"))
    {
      try
      {
        configuration.mobileNumber = valueStr;
        Serial.println("Mobile number set successfully");
      }
      catch (...)
      {
        Serial.println("Invalid mobile number format: " + valueStr);
        return;
      }
    }
    else
    {
      Serial.println("Invalid mobile number format: " + valueStr);
      return;
    }
  }
  else if (key.equalsIgnoreCase("GSM_RX_PIN"))
  {
    configPin.GSM_RX_PIN = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("GSM_TX_PIN"))
  {
    configPin.GSM_TX_PIN = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("Pin_MQ8"))
  {
    configPin.Pin_MQ8 = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("DHT_SENSOR_PIN"))
  {
    configPin.DHT_SENSOR_PIN = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("DS18B20_PIN"))
  {
    configPin.DS18B20_PIN = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("flowSensorPin"))
  {
    configPin.flowSensorPin = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("EC_PIN"))
  {
    configPin.EC_PIN = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("CurrentPin"))
  {
    configPin.CurrentPin = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("PH_PIN"))
  {
    configPin.PH_PIN = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("voltPin"))
  {
    configPin.voltPin = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("SW_420_Pin"))
  {
    configPin.SW_420_Pin = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("temperatureAmount"))
  {
    configNumeric.temperatureAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("phvalueAmount"))
  {
    configNumeric.phValueAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("humidityAmount"))
  {
    configNumeric.humidityAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("ecValueAmount"))
  {
    configNumeric.ecValueAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("flowRateAmount"))
  {
    configNumeric.flowRateAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("acsAmount"))
  {
    configNumeric.acsAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("ds18b20Amount"))
  {
    configNumeric.ds18b20Amount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("h2Amount"))
  {
    configNumeric.h2Amount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("voltAmount"))
  {
    configNumeric.voltAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("powerAmount"))
  {
    configNumeric.powerAmount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("SW420Amount"))
  {
    configNumeric.SW420Amount = valueStr.toInt();
  }
  else if (key.equalsIgnoreCase("flowSensorCalibration"))
  {
    configNumeric.flowSensorCalibration = valueStr.toFloat();
  }
  else if (!key.isEmpty())
  {
    Serial.println("Unknown/empty key: " + key + " on line: " + line);
    return;
  }
  else if (valueStr.isEmpty())
  {
    Serial.println("Key found but no value provided on line: " + line);
    Serial.println("Value found: " + valueStr);
    return;
  }
  else
  {
    Serial.println("Invalid key: " + key + " on line: " + line);
    return;
  }
}

bool read_configuration()
{
  File file = SD.open("/config.txt");
  if (file)
  {
    Serial.println("Config file opened successfully.");
  }
  else
  {
    Serial.println("Failed to open config file from Micro SD card, trying SPIFFS.");
    file = SPIFFS.open("/config.txt");
    if (!file)
    {
      Serial.println("Failed to open config file from SPIFFS also. Please add it manually to SD Card.");
      return false;
    }
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);

  while (file.available())
  {
    String line = file.readStringUntil('\n');
    processLine(line);
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);

  file.close();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  return true;
}

void printVariables()
{
  Serial.print("ssid: ");
  Serial.println(configuration.ssid);
  Serial.print("password: ");
  Serial.println(configuration.password);
  Serial.print("httpapi: ");
  Serial.println(configuration.httpapi);
  Serial.print("connectionMode: ");
  Serial.println(configuration.connectionMode);
  Serial.print("GSM_RX_PIN: ");
  Serial.println(configPin.GSM_RX_PIN);
  Serial.print("GSM_TX_PIN: ");
  Serial.println(configPin.GSM_TX_PIN);
  Serial.print("mobileNumber: ");
  Serial.println(configuration.mobileNumber);
  Serial.print("Pin_MQ8: ");
  Serial.println(configPin.Pin_MQ8);
  Serial.print("DHT_SENSOR_PIN: ");
  Serial.println(configPin.DHT_SENSOR_PIN);
  Serial.print("DS18B20_PIN: ");
  Serial.println(configPin.DS18B20_PIN);
  Serial.print("flowSensorPin: ");
  Serial.println(configPin.flowSensorPin);
  Serial.print("EC_PIN: ");
  Serial.println(configPin.EC_PIN);
  Serial.print("CurrentPin: ");
  Serial.println(configPin.CurrentPin);
  Serial.print("PH_PIN: ");
  Serial.println(configPin.PH_PIN);
  Serial.print("voltPin: ");
  Serial.println(configPin.voltPin);
  Serial.print("SW_420_Pin: ");
  Serial.println(configPin.SW_420_Pin);

  Serial.print("h2Amount: ");
  Serial.println(configNumeric.h2Amount);
  Serial.print("flowRateAmount: ");
  Serial.println(configNumeric.flowRateAmount);
  Serial.print("temperatureAmount: ");
  Serial.println(configNumeric.temperatureAmount);
  Serial.print("humidityAmount: ");
  Serial.println(configNumeric.humidityAmount);
  Serial.print("phValueAmount: ");
  Serial.println(configNumeric.phValueAmount);
  Serial.print("ecValueAmount: ");
  Serial.println(configNumeric.ecValueAmount);
  Serial.print("ds18b20Amount: ");
  Serial.println(configNumeric.ds18b20Amount);
  Serial.print("voltAmount: ");
  Serial.println(configNumeric.voltAmount);
  Serial.print("acsAmount: ");
  Serial.println(configNumeric.acsAmount);
  Serial.print("powerAmount: ");
  Serial.println(configNumeric.powerAmount);
  Serial.print("SW_420Amount: ");
  Serial.println(configNumeric.SW420Amount);

  Serial.print("Phone Number: ");
  Serial.println(configuration.mobileNumber);
}

void initializeConfigInterval()
{
  if (configNumeric.temperatureAmount == 0)
  {
    Serial.println("Temperature amount is 0, setting to 1");
    configNumeric.temperatureAmount = 1;
  }
  if (configNumeric.ds18b20Amount == 0)
  {
    Serial.println("DS18B20 amount is 0, setting to 1");
    configNumeric.ds18b20Amount = 1;
  }
  if (configNumeric.acsAmount == 0)
  {
    Serial.println("ACS amount is 0, setting to 1");
    configNumeric.acsAmount = 1;
  }
  if (configNumeric.humidityAmount == 0)
  {
    Serial.println("Humidity amount is 0, setting to 1");
    configNumeric.humidityAmount = 1;
  }
  if (configNumeric.flowRateAmount == 0)
  {
    Serial.println("Flow rate amount is 0, setting to 1");
    configNumeric.flowRateAmount = 1;
  }
  if (configNumeric.h2Amount == 0)
  {
    Serial.println("H2 amount is 0, setting to 1");
    configNumeric.h2Amount = 1;
  }
  if (configNumeric.voltAmount == 0)
  {
    Serial.println("Volt amount is 0, setting to 1");
    configNumeric.voltAmount = 1;
  }
  if (configNumeric.powerAmount == 0)
  {
    Serial.println("Power amount is 0, setting to 1");
    configNumeric.powerAmount = 1;
  }
  if (configNumeric.phValueAmount == 0)
  {
    Serial.println("PH value amount is 0, setting to 1");
    configNumeric.phValueAmount = 1;
  }
  if (configNumeric.ecValueAmount == 0)
  {
    Serial.println("EC value amount is 0, setting to 1");
    configNumeric.ecValueAmount = 1;
  }
  if (configNumeric.SW420Amount == 0)
  {
    Serial.println("SW_420 value amount is 0, setting to 1");
    configNumeric.SW420Amount = 1;
  }
  
  numMeasurements = std::max({
    configNumeric.temperatureAmount,
    configNumeric.phValueAmount, 
    configNumeric.humidityAmount,
    configNumeric.ecValueAmount,
    configNumeric.flowRateAmount,
    configNumeric.acsAmount,
    configNumeric.ds18b20Amount,
    configNumeric.h2Amount,
    configNumeric.voltAmount,
    configNumeric.powerAmount,
    configNumeric.SW420Amount
});

  configInterval.dht22_tempInterval = numMeasurements / configNumeric.temperatureAmount;
  configInterval.ds18b20Interval = numMeasurements / configNumeric.ds18b20Amount;
  configInterval.acsInterval = numMeasurements / configNumeric.acsAmount;
  configInterval.dht22_humInterval = numMeasurements / configNumeric.humidityAmount;
  configInterval.flowRateInterval = numMeasurements / configNumeric.flowRateAmount;
  configInterval.h2Interval = numMeasurements / configNumeric.h2Amount;
  configInterval.voltInterval = numMeasurements / configNumeric.voltAmount;
  configInterval.powerInterval = numMeasurements / configNumeric.powerAmount;
  configInterval.phValueInterval = numMeasurements / configNumeric.phValueAmount;
  configInterval.ecValueInterval = numMeasurements / configNumeric.ecValueAmount;
  configInterval.SW420Interval = numMeasurements / configNumeric.SW420Amount;
}