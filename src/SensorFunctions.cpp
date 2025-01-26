#include "config.h"

/*void AGS02MA_init(){
  Serial.println(__FILE__);
  Serial.print("AGS02MA_LIB_VERSION: ");
  Serial.println(AGS02MA_LIB_VERSION);
  Serial.println();

  Wire.begin();

  bool b = AGS.begin();
  Serial.print("BEGIN:\t");
  Serial.println(b);

  Serial.print("VERSION:\t");
  Serial.println(AGS.getSensorVersion());

  //  pre-heating improves measurement quality
  //  can be skipped
  Serial.println("\nWarming up (120 seconds = 24 dots)");
  while (AGS.isHeated() == false)
  {
    delay(5000);
    Serial.print(".");
  }
  Serial.println();

  b = AGS.setPPBMode();
  uint8_t m = AGS.getMode();
  Serial.print("MODE:\t");
  Serial.print(b);
  Serial.print("\t");
  Serial.println(m);

  uint8_t version = AGS.getSensorVersion();
  Serial.print("VERS:\t");
  Serial.println(version);
}*/

void AGS02MA_Init(){
  Serial.println(F("Adafruit AGS20MA Test"));

  if (! ags.begin(&Wire, 0x1A)) {
  //if (! ags.begin(&Wire1, 0x1A)) { // or use Wire1 instead!
    Serial.println("Couldn't find AGS20MA sensor, check your wiring and pullup resistors!");
  }

  if (ags.getFirmwareVersion() == 0) {
    Serial.println(F("Could not read firmware, I2C communications issue?"));
  }

  Serial.print("Firmware version: 0x");
  Serial.println(ags.getFirmwareVersion(), HEX);
  ags.printSensorDetails();

  // uncomment to change address, will need to restart and update the begin() argument!
  //ags.setAddress(0x1A); while (1);
}

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
 * @brief Reads the temperature of the flow sensor
 * @param unit PCNT unit to configure
 * @var NUMSAMPLES: The number of analog readings to take and average.
 * @var NTC_PIN: The pin connected to the NTC thermistor.
 * @var serialResistance: The resistance of the serial resistor in the thermistor circuit.
 * @var nominalResistance: The nominal resistance of the thermistor at a reference temperature.
 * @var bCoefficient: The beta coefficient of the thermistor.
 * @var TEMPERATURENOMINAL: The reference temperature at which the nominal resistance is specified.
 * @return steinhart The  Steinhart-Hart equation
 */
uint16_t nominalResistance = 50000; // The nominal resistance of the thermistor at a reference temperature.
int serialResistance = 90700;       // The resistance of the serial resistor in the thermistor circuit.
// 3230; (met typefout 997000 wat automatisch veranderd werdt naar 55032 is de temperatuur 19.91) 
//(En schijnbaar kan 90700 ook niet, dit is verbeterd naar 25164 en nu is de temp 38.26 gaden)
uint16_t bCoefficient = 3950;       // The beta coefficient of the thermistor.
uint8_t TEMPERATURENOMINAL = 25;    // The reference temperature at which the nominal resistance is specified.
const uint8_t NUMSAMPLES = 100;     // The number of analog readings to take and average.
float temp_flow; // Global temperature reading
// #define VERBOSE_SENSOR_ENABLED


float Read_NTC()
{
  uint8_t i;
  uint16_t sample;
  float average = 0;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++)
  {
    sample = analogRead(NTC_PIN);
    if(sample < 0 || sample > 4095) {
      Serial.println("Invalid ADC reading");
      return 0;
    }
    average += sample;
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  average /= NUMSAMPLES;

#ifdef DEBUG_MODE_3
  Serial.print("1 sample analog reading ");
  Serial.println(sample);
  Serial.printf("Average analog reading: %.2f\n", average);
#endif

  // convert the value to resistance
  float resistance = 4095 / average - 1;
  resistance = serialResistance * resistance;

#ifdef DEBUG_MODE_3
  Serial.printf("Thermistor resistance: %.2f\n", resistance);
#endif

  // resistance  / nominalResistance = 10 graden
  // nominalResistance / resistance = 43.25 graden
  float steinhart;
  steinhart = nominalResistance / (resistance - nominalResistance); // (R/Ro)
  // steinhart = resistance  / nominalResistance;     // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= bCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C

#ifdef DEBUG_MODE_3
  Serial.printf("Temperature: %.3f *C\n", steinhart);
#endif

  return steinhart;
}

/**
 * @brief Reads the voltage
 * @return voltage 
 */
float readVoltage() {
  const int numSamples = 100;
  const float adcVoltageScale = 3.3 / 4095.0;
  float voltageSum = 0.0;
  float R1 = 1000.0;
  float R2 = 10000.0;

  for (int i = 0; i < numSamples; i++) {
    int adc = analogRead(voltPin);
    voltageSum += adc * adcVoltageScale;
    vTaskDelay(2 / portTICK_PERIOD_MS); // Small delay to allow for better averaging
  }

  float voltage = (voltageSum / numSamples) * (R2 / (R1 + R2));
  return voltage;
}

/*      DS18B20 sensor            */
OneWire oneWire(DS18B20_PIN);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
int numberOfDevices;                 // Number of temperature devices found
DeviceAddress tempDeviceAddress;     // We'll use this variable to store a found device address
// DS18B20 Find and print Address
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

/*      MQ-7 MQ-8 sensor            */
float RatioMQ7CleanAir = 27.5;
float RatioMQ8CleanAir = 70.0;
void mq7_init(MQUnifiedsensor &MQ7)
{
  // CO
  MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
  // MQ7.setA(521853); MQ7.setB(-3.821); // Configurate the ecuation values to get Benzene concentration
  MQ7.setA(99.042);
  MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value
  MQ7.init();
  vTaskDelay(5 / portTICK_PERIOD_MS);

  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ7.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    Serial.print(".");
  }
  MQ7.setR0(calcR0 / 10);
  Serial.println("Done calculating R0 for MQ7!.");
  /*
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ7.setRL(9.87);
  */
  if (isinf(calcR0))
  {
    Serial.println("MQ7 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  } // while(1);
  if (calcR0 == 0)
  {
    Serial.println("MQ7 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
  } // while(1);
  /*****************************  MQ CAlibration ********************************************/
  Serial.println("MQ7 initialized!");
  MQ7.serialDebug(true);
}

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
    int adc = analogReadMilliVolts(CurrentPin);
    adc_voltage_sum += adc;             // 3.3
    vTaskDelay(2 / portTICK_PERIOD_MS); // Small delay to allow for better averaging
  }
  // Serial.println("ADC voltage raw: " + String(analogReadRaw(CurrentPin)));
  // Serial.println("ADC mV Quick: " + String(analogReadMilliVolts(CurrentPin)));
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


void processLine(String line)
{
  line.trim(); // Remove any leading or trailing whitespace
  int colonIndex = line.indexOf(':');
  if (colonIndex == -1)
  {
    Serial.println("Invalid line: " + line);
    return; // Skip if no colon found
  }
  String pinName = line.substring(0, colonIndex);
  String pinValueStr = line.substring(colonIndex + 1);
  pinValueStr.trim(); // Remove any leading or trailing whitespace
  uint8_t pinValue = pinValueStr.toInt();

  if (pinName == "GSM_RX_PIN" || pinName == "GSM_TX_PIN" || pinName == "GSM_RST_PIN" || pinName == "Pin_MQ7" || pinName == "Pin_MQ8" || pinName == "DHT_SENSOR_PIN" || pinName == "DS18B20_PIN" || pinName == "flowSensorPin" || pinName == "flowSensor2Pin" || pinName == "buttonbigOled" || pinName == "EC_PIN" || pinName == "CurrentPin" || pinName == "PH_PIN")
  {
    if (pinValue < 0 || pinValue > 39)
    {
      Serial.println("Invalid pin value: " + pinValueStr);
      Serial.println("For pin: " + pinName);
      return; // Validate pin number range for ESP32
    }
    if (pinName == "GSM_RX_PIN")
    {
      GSM_RX_PIN = pinValue;
    }
    else if (pinName == "GSM_TX_PIN")
    {
      GSM_TX_PIN = pinValue;
    }
    else if (pinName == "Pin_MQ8")
    {
      Pin_MQ8 = pinValue;
    }
    else if (pinName == "DHT_SENSOR_PIN")
    {
      DHT_SENSOR_PIN = pinValue;
    }
    else if (pinName == "DS18B20_PIN")
    {
      DS18B20_PIN = pinValue;
    }
    else if (pinName == "flowSensorPin")
    {
      flowSensorPin = pinValue;
    }
    else if (pinName == "EC_PIN")
    {
      EC_PIN = pinValue;
    }
    else if (pinName == "CurrentPin")
    {
      CurrentPin = pinValue;
    }
    else if (pinName == "PH_PIN")
    {
      PH_PIN = pinValue;
    }
    else if (pinName == "NTC_PIN")
    {
      NTC_PIN = pinValue;
    }
    else if (pinName == "voltPin")
    {
      voltPin = pinValue;
    }
  }

  if (pinName == "temperatureAmount" || pinName == "phValueAmount" || pinName == "humidityAmount" || pinName == "ecValueAmount" || pinName == "flowRateAmount" || pinName == "ds18b20Amount" || pinName == "acsAmount" || pinName == "h2Amount" || pinName == "voltAmount" || pinName == "powerAmount" || pinName == "TempFlowAmount")
  {
    if (pinValue < 0 || pinValue > 500)
    {
      Serial.println("Invalid pin value: " + pinValueStr);
      Serial.println("For pin: " + pinName);
      return; // Validate range for ESP32
    }
    else if (pinValue == 0)
    {
      Serial.println("Measurement for pin: " + pinName + " is disabled");
      Serial.println("Pin value: " + pinValueStr);
      // return;
    }
    if (pinName == "temperatureAmount")
    {
      temperatureAmount = pinValue;
    }
    else if (pinName == "phValueAmount")
    {
      phValueAmount = pinValue;
    }
    else if (pinName == "humidityAmount")
    {
      humidityAmount = pinValue;
    }
    else if (pinName == "ecValueAmount")
    {
      ecValueAmount = pinValue;
    }
    else if (pinName == "flowRateAmount")
    {
      flowRateAmount = pinValue;
    }
    else if (pinName == "acsAmount")
    {
      acsAmount = pinValue;
    }
    else if (pinName == "ds18b20Amount")
    {
      ds18b20Amount = pinValue;
    }
    else if (pinName == "h2Amount")
    {
      h2Amount = pinValue;
    }
    else if (pinName == "voltAmount")
    {
      voltAmount = pinValue;
    }
    else if (pinName == "powerAmount")
    {
      powerAmount = pinValue;
    }
    else if (pinName == "TempFlowAmount")
    {
      TempFlowAmount = pinValue;
    }
    else if (pinName == "flowSensorCalibration")
    {
      flowSensorCalibration = pinValue;
    }
  }

  if (pinName == "mobileNumber")
  {
    if (pinValueStr.length() == 12 && pinValueStr.startsWith("+"))
    {
      mobileNumber = pinValueStr;
    }
    else
    {
      Serial.println("Invalid mobile number format: " + pinValueStr);
      Serial.println("Number length: " + pinValueStr.length());
      return;
    }
  }
}

void read_configuration()
{
  File file = SD.open("/config.txt");
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
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
}

void printVariables()
{
  Serial.print("GSM_RX_PIN: ");
  Serial.println(GSM_RX_PIN);
  Serial.print("GSM_TX_PIN: ");
  Serial.println(GSM_TX_PIN);
  Serial.print("mobileNumber: ");
  Serial.println(mobileNumber);
  Serial.print("Pin_MQ8: ");
  Serial.println(Pin_MQ8);
  Serial.print("DHT_SENSOR_PIN: ");
  Serial.println(DHT_SENSOR_PIN);
  Serial.print("DS18B20_PIN: ");
  Serial.println(DS18B20_PIN);
  Serial.print("flowSensorPin: ");
  Serial.println(flowSensorPin);
  Serial.print("NTC_PIN: ");
  Serial.println(NTC_PIN);
  Serial.print("EC_PIN: ");
  Serial.println(EC_PIN);
  Serial.print("CurrentPin: ");
  Serial.println(CurrentPin);
  Serial.print("PH_PIN: ");
  Serial.println(PH_PIN);
  Serial.print("voltPin: ");
  Serial.println(voltPin);

  Serial.print("h2Amount: ");
  Serial.println(h2Amount);
  Serial.print("flowRateAmount: ");
  Serial.println(flowRateAmount);
  Serial.print("temperatureAmount: ");
  Serial.println(temperatureAmount);
  Serial.print("humidityAmount: ");
  Serial.println(humidityAmount);
  Serial.print("phValueAmount: ");
  Serial.println(phValueAmount);
  Serial.print("ecValueAmount: ");
  Serial.println(ecValueAmount);
  Serial.print("ds18b20Amount: ");
  Serial.println(ds18b20Amount);
  Serial.print("voltAmount: ");
  Serial.println(voltAmount);
  Serial.print("acsAmount: ");
  Serial.println(acsAmount);
  Serial.print("TempFlowAmount: ");
  Serial.println(TempFlowAmount);

  Serial.print("Phone Number: ");
  Serial.println(mobileNumber);
}



/*      Conductivity Sensor   */
DFRobot_ESP_EC ec;
volatile float voltage_cond, temperature_cond = 25; // variable for storing the potentiometer value
// extern int EC_PIN;
float ecValueFloat = 0;
float Cond()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) // time interval: 1s
  {
    timepoint = millis();
    voltage_cond = analogRead(EC_PIN) / 4095.0 * 3300;
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
// extern int PH_PIN;
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
    voltage_pH = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    // Serial.print("voltage_pH:");
    // Serial.println(voltage_pH, 4);

    phValue = ph.readPH(voltage_pH, temperature_pH); // convert voltage to pH with fixed temperature
                                                     // Serial.print("pH:");
    // Serial.println(phValue, 4);
  }
  ph.calibration(voltage_pH, temperature_pH); // calibration process by Serial CMD
  return phValue;
}
