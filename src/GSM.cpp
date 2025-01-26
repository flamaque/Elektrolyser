#include "config.h"

String response, date, time_gsm, jsonPayload, datetime_gsm, date_getTime = "";

volatile uint8_t GSMOutputToOLED = 1;
bool Posting = false;

void readGsmResponse()
{
    char c;
    response = "";
    unsigned long startTime = millis();
    unsigned long lastReadTime = millis();
    const unsigned long readTimeout = 500; // Time to wait for new data in milliseconds

    while (1)
    {
        if (gsmSerial.available() > 0)
        {
            uint8_t byteFromSerial = gsmSerial.read();
            c = char(byteFromSerial);
            response += c;
            // printf("%c", c);
            // Serial.write(byteFromSerial);
            if (GSMOutputToOLED == 1)
            {
                u8g2log.print(c);
            }
            lastReadTime = millis(); // Update last read time
        }
        if (millis() - lastReadTime > readTimeout)
        {
            Serial.println("readGsmResponse timeout");
            break;
        }
    }
    printf("(printf) Response: %s\n", response.c_str());
}

String readGsmResponse3()
{
    char c;
    response = "";
    unsigned long startTime = millis();
    unsigned long lastReadTime = millis();
    const unsigned long readTimeout = 2000; // Time to wait for new data in milliseconds

    while (millis() - startTime < 10000)
    {
        while (gsmSerial.available() > 0)
        {
            uint8_t byteFromSerial = gsmSerial.read();
            c = char(byteFromSerial);
            response += c;
            Serial.write(byteFromSerial);
            if (GSMOutputToOLED == 1)
            {
                if (Posting == false)
                {
                    u8g2log.print(c);
                }
            }
            lastReadTime = millis();
        }

        // Check if there's been no data read for the readTimeout duration
        if (millis() - lastReadTime > readTimeout)
        {
            break;
        }

        // Check if "+CIPGSMLOC:" is found in the response
        size_t foundIndex = response.indexOf("+CIPGSMLOC:");
        if (foundIndex != std::string::npos)
        {
            // Remove everything before "+CIPGSMLOC:"
            response = response.substring(foundIndex);
        }
    }

    // Remove any trailing characters after the newline
    int newlineIndex = response.indexOf("\r\n");
    if (newlineIndex != -1)
    {
        response = response.substring(0, newlineIndex + 2);
    }
    return response;
}

uint64_t unixTimestamp;

void IsGSMConnected()
{
    gsmSerial.println("AT+CREG?");
    readGsmResponse();
    unsigned long startTime = millis();
    while (millis() - startTime < 2000)
    {
    if (gsmSerial.available())
        {
            response += (char)gsmSerial.read();
            if (response.indexOf("OK") != -1)
                break;
        }
    }
    if (response.indexOf("+CREG: 2,1") != -1 || response.indexOf("+CREG: 2,5") != -1)
    {
      gsmConnected = true;
    }
    else
    {
      gsmConnected = false;
    }
    Serial.println("Received response: " + response);
}

uint64_t getDateTime_SIM7600()
{
    GSMOutputToOLED = 1;
    
    while(gsmSerial.available()) {
        gsmSerial.read();
    }

    // Request network time from module - SIM7600 uses CCLK command
    gsmSerial.println("AT+CCLK?");
    String response = readGsmResponse3();
    Serial.println("response in getDateTime: " + response);
    String response2 = readGsmResponse3();
    Serial.println("Second response in getDateTime: " + response2);
    String response3 = readGsmResponse3();
    Serial.println("Third response in getDateTime: " + response3);

/*
    // Read response with timeout
    unsigned long startTime = millis();
    while (millis() - startTime < 2000)
    {
        if (gsmSerial.available())
        {
            response += (char)gsmSerial.read();
            if (response.indexOf("OK") != -1)
                break;
        }
    } */
    
    GSMOutputToOLED = 0;
    // Parse response format: +CCLK: "yy/MM/dd,HH:mm:ss+zz"
    int startIdx = response.indexOf('"') + 1;
    int endIdx = response.indexOf('"', startIdx);

    if (startIdx > 0 && endIdx > startIdx)
    {
        String dateTime = response.substring(startIdx, endIdx);

        // Parse components with timezone handling
        int year = 2000 + dateTime.substring(0, 2).toInt();
        int month = dateTime.substring(3, 5).toInt();
        int day = dateTime.substring(6, 8).toInt();
        int hour = dateTime.substring(9, 11).toInt();
        int min = dateTime.substring(12, 14).toInt();
        int sec = dateTime.substring(15, 17).toInt();

        /*
        // Handle timezone offset if present
        int tzOffset = 0;
        if (dateTime.length() > 17)
        {
            tzOffset = dateTime.substring(18, 20).toInt();
            if (dateTime.charAt(17) == '-')
                tzOffset = -tzOffset;
        }
        // Adjust hour for timezone
        hour += tzOffset;
        */
       
        // Create formatted datetime string
        char formattedDateTime[32];
        Serial.printf("Parsed values: %d-%02d-%02d %02d:%02d:%02d\n", 
                     year, month, day, hour, min, sec);

        snprintf(formattedDateTime, sizeof(formattedDateTime),
                 "%04d-%02d-%02d %02d:%02d:%02d",
                 year, month, day, hour, min, sec);

        // Convert to unix timestamp with timezone consideration
        struct tm timeinfo;
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = min;
        timeinfo.tm_sec = sec;
        timeinfo.tm_isdst = -1;

        time_t timestamp = mktime(&timeinfo);
        unixTimestamp = (uint64_t)timestamp * 1000; // Convert to milliseconds
        Serial.println("Timestamp received from SIM7600: " + String(unixTimestamp));
        // Save timestamp
        saveTimestamp(unixTimestamp);

        return unixTimestamp;
    }
    else
    {
        String dateTime = response.substring(startIdx, endIdx);

        // Parse components with timezone handling
        int year = 2000 + dateTime.substring(0, 2).toInt();
        int month = dateTime.substring(3, 5).toInt();
        int day = dateTime.substring(6, 8).toInt();
        int hour = dateTime.substring(9, 11).toInt();
        int min = dateTime.substring(12, 14).toInt();
        int sec = dateTime.substring(15, 17).toInt();

        /*
        // Handle timezone offset if present
        int tzOffset = 0;
        if (dateTime.length() > 17)
        {
            tzOffset = dateTime.substring(18, 20).toInt();
            if (dateTime.charAt(17) == '-')
                tzOffset = -tzOffset;
        }
        // Adjust hour for timezone
        hour += tzOffset;
        */
       
        // Create formatted datetime string
        char formattedDateTime[32];
        snprintf(formattedDateTime, sizeof(formattedDateTime),
                 "%04d-%02d-%02d %02d:%02d:%02d",
                 year, month, day, hour, min, sec);

        // Convert to unix timestamp with timezone consideration
        struct tm timeinfo;
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = min;
        timeinfo.tm_sec = sec;
        timeinfo.tm_isdst = -1;

        time_t timestamp = mktime(&timeinfo);
        unixTimestamp = (uint64_t)timestamp * 1000; // Convert to milliseconds
        Serial.println("Timestamp received from SIM7600: " + String(unixTimestamp));
        // Save timestamp
        saveTimestamp(unixTimestamp);

        return unixTimestamp;
    }
    
    return 0;
}

void getTime()
{
    bool validResponse = false;
    while (!validResponse)
    {
        // Setup GPRS
        // gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\""); // Sets the mode to GPRS
        // readGsmResponse();
        // vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn); // Set APN parameters
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User); // Set APN username
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=3,1,\"PWD\"," + apn_Pass); // Set APN password
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=1,1"); // Open the carrier with previously defined parameters "start command"
        readGsmResponse();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gsmSerial.println("AT+SAPBR=2,1"); // Query the status of previously opened GPRS carrier
        readGsmResponse();

        // vTaskDelay(2000 / portTICK_PERIOD_MS);

        while (!validResponse)
        {
            printf("SIM800 requesting datetime...\n");
            gsmSerial.println("AT+CIPGSMLOC=2,1");
            String timeTest = readGsmResponse3();
            Serial.println("timeTest= " + timeTest);
            vTaskDelay(500 / portTICK_PERIOD_MS);

            gsmSerial.println("AT+CIPGSMLOC=2,1");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            String timeTestChar = response;
            Serial.println("timeTestChar= " + timeTestChar);
            vTaskDelay(500 / portTICK_PERIOD_MS);

            gsmSerial.println("AT+CIPGSMLOC=2,1");
            String resp = readGsmResponse3();
            vTaskDelay(500 / portTICK_PERIOD_MS);

            int startIndex = timeTestChar.indexOf("+CIPGSMLOC: ");
            if (startIndex == -1)
            {
                printf("Error: Invalid time, trying again...\n");
                printf("Response: %s\n", timeTestChar.c_str());
                break; // Exit the inner loop and retry GPRS setup
            }

            int endIndex = timeTestChar.indexOf("\r\n", startIndex);
            String data = timeTestChar.substring(startIndex + 12, endIndex);

            // Split the response string by commas
            int firstComma = data.indexOf(',');
            int secondComma = data.indexOf(',', firstComma + 1);

            if (firstComma == -1 || secondComma == -1)
            {
                Serial.println("Error: Malformed response line 244, trying again...");
                printf("(printf) Malformed response: %s\n", data.c_str());
                printf("(printf) resp: %s\n", timeTestChar.c_str());
                printf("firstComma: %d, secondComma: %d, \n", firstComma, secondComma);
                break; // Exit the inner loop and retry GPRS setup
            }

            date_getTime = data.substring(firstComma + 1, secondComma);
            time_gsm = data.substring(secondComma + 1);

            // Check if the date and time are valid
            if (date_getTime.toDouble() == 0.000000 || time_gsm.toDouble() == 0.000000)
            {
                Serial.println("Error: Invalid date/time, trying again...");
                Serial.println("Parsed Date: " + date_getTime);
                Serial.println("Parsed Time: " + time_gsm);
                break; // Exit the inner loop and retry GPRS setup
            }

            // If all validations pass, set validResponse to true
            validResponse = true;
        }
    }

    datetime_gsm = date_getTime + " " + time_gsm;
    Serial.println("Valid datetime received. " + datetime_gsm);
    Serial.println("Date: " + date_getTime);
    Serial.println("Time: " + time_gsm);
    saveTimestamp(unixTimestamp);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void saveTimestamp(uint64_t timestamp_ms)
{
    nvs_flash_init();
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_set_u64(my_handle, "timestamp_ms", timestamp_ms);
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

uint64_t getSavedTimestamp_GSM()
{
    uint64_t timestamp_ms = 0;
    nvs_flash_init();
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_get_u64(my_handle, "timestamp_ms", &timestamp_ms);
    nvs_close(my_handle);
    return timestamp_ms;
}

uint64_t convertToUnixTimestamp(String date, String time)
{
    uint64_t timestamp_ms = 0;
    int year, month, day = 0;

    // Extract year, month, day from date

    year = date.substring(0, 4).toInt();
    // Ensure correct substring positions based on your date format "DD/MM/YY"
    month = date.substring(3, 5).toInt();
    day = date.substring(0, 2).toInt(); // Extract day from the start

    // Extract hour, minute, second from time
    int hour = time.substring(0, 2).toInt();
    int minute = time.substring(3, 5).toInt();
    int second = time.substring(6, 8).toInt();

    // Create a tm struct
    struct tm t;
    memset(&t, 0, sizeof(t)); // Initialize to zero
    t.tm_year = year - 1900;  // tm_year is years since 1900
    t.tm_mon = month - 1;     // tm_mon is 0-11
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = -1; // Not set by default

    // Convert to time_t (UNIX timestamp)
    time_t timestamp = mktime(&t);
    if (timestamp == -1)
    {
        Serial.println("Failed to convert time using mktime.");
        return -1;
    }
    Serial.println("Data in convertToUnixTimestamp: ");
    Serial.println("Parsed day: " + String(day));
    Serial.println("Parsed month: " + String(month));
    Serial.println("Parsed year: " + String(year));
    Serial.println("Parsed Date: " + date);
    Serial.println("Parsed hour: " + String(hour));
    Serial.println("Parsed minute: " + String(minute));
    Serial.println("Parsed second: " + String(second));
    Serial.println("Parsed Time: " + time);
    // Print the intermediate timestamp
    Serial.println("Intermediate timestamp: " + String(timestamp));

    // Calculate the timestamp in milliseconds
    timestamp_ms = timestamp * 1000LL; // No milliseconds provided in time string

    // Print the final timestamp in milliseconds
    Serial.println("Final timestamp with milliseconds: " + String(timestamp_ms));
    return timestamp_ms;
}

enum GsmState
{
    GSM_INIT,
    GSM_SET_CONTYPE,
    GSM_SET_APN,
    GSM_SET_USER,
    GSM_SET_PASS,
    GSM_ACTIVATE_GPRS,
    GSM_QUERY_STATUS,
    GSM_HTTP_INIT,
    GSM_HTTP_PARA_CID,
    GSM_HTTP_PARA_URL,
    GSM_HTTP_PARA_CONTENT,
    GSM_CONFIG_DONE
};

GsmState gsmState = GSM_INIT;
unsigned long previousMillis = 0;
const long interval1 = 1000; // 5000
const long interval2 = 1000; // 2000
const long interval3 = 1000; // 3000
/*
void initialize_gsm()
{
    unsigned long currentMillis = millis();

    switch (gsmState)
    {
    case GSM_INIT:
        Serial.println("Configure APN settings.");
        gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\",\"IP\"");
        previousMillis = currentMillis;
        gsmState = GSM_SET_CONTYPE;
        break;

    case GSM_SET_CONTYPE:
        if (currentMillis - previousMillis >= interval1)
        {
            gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn);
            previousMillis = currentMillis;
            gsmState = GSM_SET_APN;
        }
        break;


    case GSM_SET_APN:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User);
            previousMillis = currentMillis;
            gsmState = GSM_SET_USER;
        }
        break;
    case GSM_SET_USER:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+SAPBR=3,1,\"PWD\"," + apn_Pass);
            previousMillis = currentMillis;
            gsmState = GSM_SET_PASS;
        }
        break;
    case GSM_SET_PASS:
        if (currentMillis - previousMillis >= interval2)
        {
            Serial.println("APN settings configured.");
            Serial.println("Configure GPRS settings.");
            gsmSerial.println("AT+SAPBR=1,1");
            previousMillis = currentMillis;
            gsmState = GSM_ACTIVATE_GPRS;
        }
        break;
    case GSM_ACTIVATE_GPRS:
        if (currentMillis - previousMillis >= interval3)
        {
            gsmSerial.println("AT+SAPBR=2,1");
            previousMillis = currentMillis;
            gsmState = GSM_QUERY_STATUS;
        }
        break;
    case GSM_QUERY_STATUS:
        if (currentMillis - previousMillis >= interval3)
        {
            Serial.println("GPRS settings configured.");
            gsmSerial.println("AT+HTTPINIT"); // Initialize HTTP service
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_INIT;
        }
        break;

    case GSM_HTTP_INIT:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"CID\",1"); // Define carrier profile
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_CID;
        }
        break;

    case GSM_HTTP_PARA_CID:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"URL\", " + String(httpapi)); // Set the URL
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_URL;
        }
        break;

    case GSM_HTTP_PARA_URL:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\""); // Set the content type
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_CONTENT;
        }
        break;

    case GSM_HTTP_PARA_CONTENT:
        if (currentMillis - previousMillis >= interval2)
        {
            Serial.println("HTTP settings configured.");
            gsmState = GSM_CONFIG_DONE;
        }
        break;

    case GSM_CONFIG_DONE:
    Serial.println("GSM configuration done.");
    gsmSerial.println("AT+CFUN?");
    readGsmResponse();
    gsmSerial.println("AT+CSQ");
    readGsmResponse();
        gsmSerial.println("AT+CIFSR");
    readGsmResponse();
        // Initialization done
        break;
    }
}
*/
void initialize_gsm()
{
    unsigned long currentMillis = millis();

    switch (gsmState)
    {
    case GSM_INIT:
        Serial.println("Initializing SIM7600");
        gsmSerial.println("AT+CFUN=1"); // Set full functionality
        readGsmResponse();
        gsmSerial.println("AT+CMEE=2"); // Enable verbose error reporting
        readGsmResponse();
        previousMillis = currentMillis;
        gsmState = GSM_SET_CONTYPE;
        break;

    case GSM_SET_CONTYPE:
        if (currentMillis - previousMillis >= interval1)
        {
            // Configure Network Mode to LTE only
            gsmSerial.println("AT+CNMP=38");
            readGsmResponse();
            // Configure LTE Category
            gsmSerial.println("AT+CNACT=0,1");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_SET_APN;
        }
        break;

    case GSM_SET_APN:
        if (currentMillis - previousMillis >= interval2)
        {
            // Set PDP context
            gsmSerial.println("AT+CGDCONT=1,\"IP\"," + apn);
            readGsmResponse();
            // Activate PDP context
            gsmSerial.println("AT+CGACT=1,1");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_SET_USER;
        }
        break;

    case GSM_SET_USER:
        if (currentMillis - previousMillis >= interval2)
        {
            // Set authentication parameters
            gsmSerial.println("AT+CGAUTH=1,1,\"" + apn_User + "\",\"" + apn_Pass + "\"");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_ACTIVATE_GPRS;
        }
        break;

    case GSM_ACTIVATE_GPRS:
        if (currentMillis - previousMillis >= interval2)
        {
            // Enable network time sync
            gsmSerial.println("AT+CTZU=1");
            readGsmResponse();
            gsmSerial.println("AT+CTZR=1");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_QUERY_STATUS;
        }
        break;

    case GSM_QUERY_STATUS:
        if (currentMillis - previousMillis >= interval3)
        {
            // Check network registration
            gsmSerial.println("AT+CREG?");
            readGsmResponse();
            // Check signal quality
            gsmSerial.println("AT+CSQ");
            readGsmResponse();
            // Check PDP context status
            gsmSerial.println("AT+CGACT?");
            readGsmResponse();
            // Check IP address
            gsmSerial.println("AT+CGPADDR=1");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_INIT;
        }
        break;

    case GSM_HTTP_INIT:
        if (currentMillis - previousMillis >= interval2)
        {
            // Initialize HTTP service
            gsmSerial.println("AT+HTTPINIT");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_CID;
        }
        break;

    case GSM_HTTP_PARA_CID:
        if (currentMillis - previousMillis >= interval2)
        {
            // Set HTTP parameters
            gsmSerial.println("AT+HTTPPARA=\"CID\",1");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_URL;
        }
        break;

    case GSM_HTTP_PARA_URL:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"URL\"," + String(httpapi));
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_HTTP_PARA_CONTENT;
        }
        break;

    case GSM_HTTP_PARA_CONTENT:
        if (currentMillis - previousMillis >= interval2)
        {
            gsmSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
            readGsmResponse();
            previousMillis = currentMillis;
            gsmState = GSM_CONFIG_DONE;
        }
        break;

    case GSM_CONFIG_DONE:
        Serial.println("SIM7600 configuration complete");
        gsmSerial.println("AT+CFUN?");
        readGsmResponse();
        gsmSerial.println("AT+CSQ");
        readGsmResponse();
        gsmSerial.println("AT+CGPADDR");
        readGsmResponse();
        break;
    }
}