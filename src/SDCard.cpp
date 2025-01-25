#include "config.h"

void initSD() {
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS_PIN, SPI, 25000000)) {
        Serial.println("SD Card initialization failed!");
        uint8_t cardType = SD.cardType();
        Serial.print("Card Type: ");
        switch(cardType) {
            case CARD_NONE: Serial.println("No card attached"); break;
            case CARD_MMC: Serial.println("MMC"); break;
            case CARD_SD: Serial.println("SDSC"); break;
            case CARD_SDHC: Serial.println("SDHC"); break;
            default: Serial.println("Unknown"); break;
        }
        return;
    }
    Serial.println("SD Card initialized successfully");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

/*
void SD_init()
{
  if(!spi.begin())
  {
    Serial.println("Card Mount Failed");
    return;
  }
  if (SD.begin(CS_PIN, spi, 25000000))
  {
    Serial.println("Reading config file.");
    // read_configuration();
    // Serial.println("All values after setup:");
    // printVariables();
    // Serial.println("");
    File root;
    root = SD.open("/");
    printDirectory(root, 0);
    root.close();
  }
  else
  {
    Serial.println("SD initialization failed.");
  }

  Serial.println("SD Card initialized.");
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    // Light up RED LED
    // return;
  }

  Serial.print("SD Card Type: ");
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

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  File file = SD.open("/log.txt");
  if (!file)
  {
    Serial.println("Log file doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/log.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else
  {
    Serial.println("Log.txt file already exists");
  }
  file.close();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  file = SD.open("/One_Measurement.txt");
  if (!file)
  {
    Serial.println("One_Measurement file doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/One_Measurement.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else
  {
    Serial.println("One_Measurement.txt file already exists");
  }
  file.close();

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}*/