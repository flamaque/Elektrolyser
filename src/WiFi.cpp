#include "config.h"

/*
void getTime_WiFi()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    // return(0);
  }
  time(&now);
  uint64_t timestamp_ms = (uint64_t)now * 1000;  // Safely calculate milliseconds since Unix epoch

  // Initialize NVS storage
  if (nvs_flash_init() != ESP_OK) {
    Serial.println("NVS initialization failed!");
    return;
  }
  nvs_handle_t my_handle;
  if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
    // Save the timestamp in milliseconds
    nvs_set_u64(my_handle, "timestamp_ms", timestamp_ms);
    nvs_commit(my_handle);
    nvs_close(my_handle);

    Serial.println("Date time received: ");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    Serial.println("Saved timestamp in milliseconds: " + String(timestamp_ms));
  } else {
    Serial.println("Failed to open NVS handle.");
  }
}
*/
void getTime_WiFi()
{
  time_t now;
  struct tm timeinfo;
  int retryCount = 0;
  const int maxRetries = 3;

  while (retryCount < maxRetries) {
    if (getLocalTime(&timeinfo)) {
      time(&now);
      uint64_t timestamp_ms = (uint64_t)now * 1000;  // Safely calculate milliseconds since Unix epoch

      // Initialize NVS storage
      if (nvs_flash_init() != ESP_OK) {
        Serial.println("NVS initialization failed!");
        return;
      }
      nvs_handle_t my_handle;
      if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        // Save the timestamp in milliseconds
        nvs_set_u64(my_handle, "timestamp_ms", timestamp_ms);
        nvs_commit(my_handle);
        nvs_close(my_handle);

        Serial.println("Date time received: ");
        Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        Serial.println("Saved timestamp in milliseconds: " + String(timestamp_ms));
        return;  // Success - exit function
      } else {
        Serial.println("Failed to open NVS handle.");
      }
    }
    
    retryCount++;
    Serial.println("Failed to obtain time, attempt " + String(retryCount) + " of " + String(maxRetries));
    delay(1000);  // Wait 1 second before next attempt
  }
  
  Serial.println("Failed to obtain time after " + String(maxRetries) + " attempts");
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

uint64_t getSavedTimestamp_WiFi()
{
  uint64_t saved_timestamp = 0;
  // Open NVS storage
  nvs_handle_t my_handle;
  if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
    // Read the timestamp in milliseconds
    if (nvs_get_u64(my_handle, "timestamp_ms", &saved_timestamp) != ESP_OK) {
      Serial.println("Failed to read timestamp from NVS.");
    }
    nvs_close(my_handle);
  } else {
    Serial.println("Failed to open NVS handle for reading.");
  }

  return saved_timestamp;
}
