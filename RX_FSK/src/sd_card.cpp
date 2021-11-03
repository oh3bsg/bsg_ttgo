// sd_card related functions
// https://randomnerdtutorials.com/esp32-microsd-card-arduino/#sdcardcustompins

#include <math.h>
#include <SPI.h>
#include "SD.h"
#include "sonde.h"
#include "sd_card.h"

SPIClass spi_sd = SPIClass(HSPI);

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void sd_card_init(void) {
    Serial.println("sd_card_init()");
    spi_sd.begin(14, 2, 15);
    if(!SD.begin(13, spi_sd)){
        Serial.println("Card Mount Failed");
        return;
    }
    else {
        Serial.println("Card Mounted");
    }

    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void sd_card_log(SondeInfo *s) {
  char file[18];
  char log_msg[150];
  struct tm timeinfo;
  time_t now;
  time(&now);
  gmtime_r(&now, &timeinfo);
  if (timeinfo.tm_year <= (2016 - 1900)) {
    Serial.println("Failed to obtain time");
    return;
  }

  sprintf(file, "/log/%s.txt",s->d.ser);
  // timestamp,serial,frame,lat,lon,alt,vel_v,vel_h,heading,temp,humidity,pressure,type,freq_mhz,snr,f_error_hz,sats,batt_v,burst_timer,aux_data
  // 2021-10-30T00:58:41.000Z,S2250304,6826,59.48863,26.47421,9997.6,-25.5,23.0,82.6,-50.7,25.0,-1.0,RS41-SG,404.001,8.3,-187,9,2.5,08:28:27,-1
  sprintf(log_msg, 
    "%04d-%02d-%02dT%02d:%02d:%02d.000Z,%s,%d,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%.3f,%d,-1,%d,%.1f,%d,-1\n",
    timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
    s->d.ser, s->d.vframe, (float)s->d.lat, (float)s->d.lon, (float)s->d.alt, (float)s->d.vs, (float)s->d.hs, (float)s->d.dir,
    isnan((float)s->d.temperature) ? -273: (float)s->d.temperature, isnan((float)s->d.relativeHumidity) ? -1: (float)s->d.relativeHumidity, isnan((float)s->d.pressure) ? -1: (float)s->d.pressure, s->d.typestr != 0 ? s->d.typestr: " ", (float)s->freq, s->rssi, s->d.sats, 
    s->d.batteryVoltage, s->d.burstKT
    );
    Serial.println(log_msg);

    if (SD.exists(file)) {
      // append data
      Serial.println("append new file");
      appendFile(SD, file, log_msg);
    }
    else {
      // new file
      Serial.println("new file");
      writeFile(SD, file, "timestamp,serial,frame,lat,lon,alt,vel_v,vel_h,heading,temp,humidity,pressure,type,freq_mhz,snr,f_error_hz,sats,batt_v,burst_timer,aux_data\n");
      appendFile(SD, file, log_msg);
    }
}